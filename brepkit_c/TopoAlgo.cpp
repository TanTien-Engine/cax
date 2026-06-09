#include "TopoAlgo.h"
#include "TopoShape.h"
#include "occt_adapter.h"

#include "ShapeBuilder.h"
#include "brepgraph_c/history/TopoNaming.h"
#include "brepdb_c/WorldSender.h"
#include "brepdb_c/VersionTree.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>

// OCCT
#include <OSD.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <ChFi3d_FilletShape.hxx>
#include <BRepOffsetAPI_MakeDraft.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffset_Mode.hxx>
#include <GeomAbs_JoinType.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <BOPAlgo_Splitter.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <TopExp.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <BRepTools_History.hxx>
#include <Standard_Failure.hxx>
#include <gp_Ax2.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <BRep_Tool.hxx>
#include <Precision.hxx>

#ifdef _MSC_VER
#  include <excpt.h>
#endif

// Verbose geometry-eval diagnostics ([FILLET] / [CHAMFER] / [SPLIT_BODY]
// / [face_picks] / [edge_diff] traces) are OFF by default. They fire per
// edge / per op during eval; on the parallel per-part load path N worker
// threads then contend on the single stderr lock (and a Windows console
// flushes each line synchronously), which serialises the otherwise
// independent replays and dominates load time on big assemblies. Set
// CAX_GEO_LOG=1 to re-enable them for debugging (same switch gates the
// reader's [edge_diff]/[face_picks] traces). The env is read once
// (function-local static -> thread-safe init), so the steady-state cost
// when off is a single predictable branch per call site.
static bool ta_log_on()
{
    static const bool on = [] {
        const char* e = std::getenv("CAX_GEO_LOG");
        return e && e[0] && e[0] != '0';
    }();
    return on;
}
#define TA_LOG(...) do { if (ta_log_on()) std::fprintf(stderr, __VA_ARGS__); } while (0)

// Per-boolean cost profiler. BREPKIT_BOP_PROF=1 emits one stderr line per
// Cut / Fuse / Common recording the OCCT boolean Build() time vs the
// RefineBopResult (ShapeUpgrade_UnifySameDomain) time, with running
// cumulative totals so the LAST line is the whole-replay summary. This
// settles "of the ~Xms/boolean, how much is the BOP itself vs the refine
// pass, and how many booleans run". Off by default = one branch per call.
// Read once (function-local static -> thread-safe init).
static bool bop_prof_on()
{
    static const bool on = [] {
        const char* e = std::getenv("BREPKIT_BOP_PROF");
        return e && e[0] && e[0] != '0';
    }();
    return on;
}

// Cheap face count for the prof line (only called when prof is on).
static int BopProfFaceCount(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0;
    int n = 0;
    for (TopExp_Explorer ex(s, TopAbs_FACE); ex.More(); ex.Next()) ++n;
    return n;
}

// Accumulate + log one boolean's split. Cumulative state is mutex-guarded
// because a single part's calc-graph eval may run parallel
// (CalcGraph::set_parallel); the lock also keeps stderr lines un-interleaved.
static void bop_prof_log(const char* op, double build_ms, double refine_ms,
                         int faces_in, int faces_out)
{
    static std::mutex mtx;
    static int        n = 0;
    static double     cum_build = 0.0, cum_refine = 0.0;
    std::lock_guard<std::mutex> lk(mtx);
    ++n;
    cum_build  += build_ms;
    cum_refine += refine_ms;
    std::fprintf(stderr,
        "[bop_prof] #%d op=%-6s build=%8.1f refine=%8.1f faces=%d->%d "
        "| cum build=%9.1f refine=%9.1f total=%9.1f\n",
        n, op, build_ms, refine_ms, faces_in, faces_out,
        cum_build, cum_refine, cum_build + cum_refine);
    std::fflush(stderr);
}

// Fuse fuzzy-retry instrumentation + kill-switch. The retry block in
// TopoAlgo::Fuse re-runs the FULL boolean up to 3x (fuzzy 1e-5/1e-4/1e-3)
// whenever the 1e-6 fuse yields a multi-solid COMPOUND with overlapping
// operand bboxes. On a legitimately-multi-solid result no larger fuzzy
// reduces the count (or only by melting volume, which the guard rejects),
// so all 3 retries are wasted full booleans + volume props. The per-entry
// [bop_retry] line counts how often it fires and how much it costs;
// BREPKIT_BOP_RETRY=0 disables the block entirely so we can A/B the cost
// AND the result impact (does the r2900_20 geo golden still match?).
// Modes via env BREPKIT_BOP_RETRY:
//   unset / "2"/"fast" -> 2 = FAST (DEFAULT): stop escalating as soon as a
//                             fuzzy step fails to produce an accepted merge.
//                             The waste case is "legit-disjoint multi-solid,
//                             no fuzzy ever merges, all 3 rejected"; helpful
//                             retries all succeed at the first step (1e-5),
//                             so early-break is result-preserving (verified:
//                             172/172 geo goldens identical FULL vs FAST) and
//                             cuts ~82% of retry cost.
//   "1"               -> 1 = FULL: legacy behavior, try all of 1e-5/1e-4/1e-3.
//   "0"               -> 0 = OFF:  skip the retry block entirely.
static int bop_retry_mode()
{
    static const int mode = [] {
        const char* e = std::getenv("BREPKIT_BOP_RETRY");
        if (!e || !e[0]) return 2;   // default FAST
        if (e[0] == '0') return 0;
        if (e[0] == '1') return 1;
        return 2;
    }();
    return mode;
}

// Wrap one OCCT Build() call in Win32 SEH so a ChFi3d access
// violation (Page_045_Exercise2D-37_byHannu's chamfer hits one
// in ChFi3d_IsInFront on vertex-corner stripes) becomes a "Build
// failed" return instead of tearing down the editor.
//
// Why we need __try at all despite OSD::SetSignal: SetSignal
// installs a _set_se_translator that converts SEH to a C++ throw,
// but the translator only fires under MSVC /EHa. cax compiles
// with MSVC defaults (/EHsc), so the C++ catch (...) sees nothing
// and the raw AV reaches the unhandled-exception filter -> exit.
//
// Why this is split into an extern "C" thunk + a templated outer
// shim: __try / __except cannot live in a function that requires
// C++ object unwinding (MSVC C2712). Calling a possibly-throwing
// C++ method (op->Build()) directly inside __try counts as
// requiring unwinding under /EHsc. The fix is to host __try in
// an extern "C" function (no C++ unwind expectation), pass the
// op via a void* + function-pointer thunk, and absorb any C++
// Standard_Failure inside the thunk so nothing C++-y escapes
// the extern "C" frame.
//
// Some OCCT internal allocations leak on the SEH path because C++
// unwind is bypassed -- accepted as the cost of staying alive.
#ifdef _MSC_VER
extern "C" {
// `static` keeps this TU-local; `extern "C"` strips the C++
// unwind expectation that would otherwise trigger C2712 when
// __try sits next to a possibly-throwing call.
static int seh_call_void(void (*fn)(void*), void* arg)
{
    __try {
        fn(arg);
        return 0;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return -1;
    }
}
} // extern "C"
#endif

namespace
{

// Returns:
//   1  -- Build() finished, IsDone() == true
//   0  -- Build() finished but IsDone() == false (clean OCCT
//         failure -- no exception, just refused to produce a shape).
//  -1  -- Build() raised a Win32 SEH (AV, FP, stack overflow) that
//         was caught by the __except in seh_call_void. This is the
//         "raw" SEH path: OCCT's _set_se_translator either wasn't
//         installed or didn't fire.
//  -2  -- A C++ throw (Standard_Failure or similar) propagated out
//         of Build() and was absorbed by the catch in the runner.
//         In practice the most common source on /EHsc is OSD's
//         _set_se_translator converting an SEH into a C++ throw --
//         so r==-2 on this build means "underlying cause was a
//         Win32 fault that OCCT translated for us." Distinguishing
//         -2 from 0 matters for diagnostics: 0 is "OCCT politely
//         refused", -2 is "we narrowly avoided a process death."
template <class Op>
int seh_safe_build(Op* op)
{
#ifdef _MSC_VER
    struct Ctx { Op* op; int result; };
    Ctx ctx{op, 0};
    // Stateless lambda -> C-style function pointer. Absorbs any
    // C++ throw locally so it cannot propagate into the extern
    // "C" frame (where C++ exceptions are UB).
    auto runner = +[](void* p) -> void {
        auto* c = static_cast<Ctx*>(p);
        try {
            c->op->Build();
            c->result = c->op->IsDone() ? 1 : 0;
        } catch (...) {
            c->result = -2;
        }
    };
    if (seh_call_void(runner, &ctx) < 0) {
        return -1;
    }
    return ctx.result;
#else
    try {
        op->Build();
        return op->IsDone() ? 1 : 0;
    } catch (...) {
        return -2;
    }
#endif
}

// Human-readable label for a seh_safe_build return code. Used in
// log lines so "r=-2" doesn't require cross-referencing the table
// above to decode.
inline const char* seh_safe_build_label(int r)
{
    switch (r) {
        case  1: return "ok";
        case  0: return "IsDone=false";
        case -1: return "SEH caught";
        case -2: return "C++ throw caught";
        default: return "unknown";
    }
}

// Single-body op: commit as child of input shape's version node.
// Stores the new version_id back into dst.
void commit_to_vt(const std::shared_ptr<brepgraph::TopoNaming>& tn,
                  const std::shared_ptr<brepdb::VersionTree>& vt,
                  const std::shared_ptr<brepkit::TopoShape>& src,
                  const std::shared_ptr<brepkit::TopoShape>& dst,
                  const brepgraph::TopoNaming::PidMap& pid_map,
                  const std::string& op_name)
{
    if (!tn || !vt) {
        return;
    }
    brepdb::WorldSender sender(tn);
    brepdb::BRepWorld world;
    sender.Serialize(dst->GetShape(), world);

    uint32_t parent_id = src->GetVersionId();
    uint32_t new_id;
    if (parent_id == brepkit::TopoShape::NO_VERSION) {
        uint32_t root_id = vt->AddRoot(world, op_name);
        new_id = root_id;
    } else {
        uint32_t root_id = vt->FindRootOf(parent_id);
        vt->Checkout(root_id, parent_id);
        new_id = vt->Commit(root_id, world, pid_map, op_name);
    }
    dst->SetVersionId(new_id);
}

// Boolean-op variant: creates a merge node.
// primary_parent comes from the main body's version_id,
// the tool body's version_id becomes an auxiliary parent.
//
// tool_world must be serialized BEFORE tn->Update(), because Update()
// unbinds the old shapes (including the tool body) from the HistGraph.
void merge_to_vt(const std::shared_ptr<brepgraph::TopoNaming>& tn,
                 const std::shared_ptr<brepdb::VersionTree>& vt,
                 const std::shared_ptr<brepkit::TopoShape>& main_src,
                 const std::shared_ptr<brepkit::TopoShape>& tool_src,
                 const std::shared_ptr<brepkit::TopoShape>& dst,
                 brepdb::BRepWorld&& tool_world,
                 const brepgraph::TopoNaming::PidMap& pid_map,
                 const std::string& op_name)
{
    if (!tn || !vt) {
        return;
    }

    uint32_t tool_vid = tool_src->GetVersionId();
    if (tool_vid == brepkit::TopoShape::NO_VERSION) {
        tool_vid = vt->AddRoot(tool_world, op_name + "_tool");
    }

    brepdb::WorldSender sender(tn);
    brepdb::BRepWorld world;
    sender.Serialize(dst->GetShape(), world);

    uint32_t primary_id = main_src->GetVersionId();
    uint32_t new_id = vt->Merge(primary_id, { tool_vid },
                                world, pid_map, op_name);
    dst->SetVersionId(new_id);
}

brepdb::BRepWorld serialize_world(const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                  const TopoDS_Shape& shape)
{
    brepdb::WorldSender sender(tn);
    brepdb::BRepWorld world;
    sender.Serialize(shape, world);
    return world;
}

}

namespace brepkit
{

namespace
{

// If `s` is a COMPOUND that wraps exactly one SOLID, return that
// SOLID. Otherwise return `s` untouched. Downstream BOPs prefer
// SOLID args; OCCT routinely wraps Fuse / Fillet results in a
// COMPOUND that callers then have to peel.
TopoDS_Shape UnwrapSingleSolid(const TopoDS_Shape& s)
{
    if (s.IsNull() || s.ShapeType() != TopAbs_COMPOUND) {
        return s;
    }
    TopExp_Explorer ex(s, TopAbs_SOLID);
    if (!ex.More()) {
        return s;
    }
    TopoDS_Shape first = ex.Current();
    ex.Next();
    if (ex.More()) {
        return s;   // multiple solids -> keep as compound
    }
    return first;
}

int CountSolids(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0;
    int n = 0;
    for (TopExp_Explorer e(s, TopAbs_SOLID); e.More(); e.Next()) ++n;
    return n;
}

// True if the loose bboxes of two shapes touch or overlap. Used to
// guard the Fuse retry-with-larger-fuzzy fallback: only escalate
// fuzzy when the operands are actually expected to merge (their
// bboxes touch), never on disjoint inputs where a multi-solid
// COMPOUND result is the correct answer.
bool BboxesOverlap(const TopoDS_Shape& a, const TopoDS_Shape& b)
{
    if (a.IsNull() || b.IsNull()) return false;
    Bnd_Box ba, bb;
    BRepBndLib::Add(a, ba);
    BRepBndLib::Add(b, bb);
    if (ba.IsVoid() || bb.IsVoid()) return false;
    return !ba.IsOut(bb);
}

// Boolean fuzzy scaled to operand size. OCCT's fuzzy is ABSOLUTE; the fixed
// 1e-6 is ~1% of a metre-scale feature (enough to glue thin coplanar faces)
// but only ~0.001% of a millimetre-scale feature -- there the BOP leaves
// gaps and a patterned 0.1 mm boss fused onto the body silently melts it
// (R2900_30 Pattern3: works at metre scale, destroys the body at mm). Scale
// the fuzzy with the operands' bbox diagonal so behaviour is constant across
// import scales. The diag>1.0 gate keeps EVERY metre-scale shape (all
// goldens, FreeCAD + golden-harness ZW) at exactly 1e-6 -> zero golden churn;
// only mm-scale parts (the editor's unit_scale=1 ZW import, diag in the
// 10s-1000s) get the larger fuzzy. Returns the base fuzzy; the Fuse retry
// escalates in multiples of it (so metre retries stay 1e-5/1e-4/1e-3).
double ScaledBopFuzzy(const TopoDS_Shape& a, const TopoDS_Shape& b)
{
    Bnd_Box box;
    if (!a.IsNull()) BRepBndLib::Add(a, box);
    if (!b.IsNull()) BRepBndLib::Add(b, box);
    if (box.IsVoid()) return 1e-6;
    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    const double dx = xmax - xmin, dy = ymax - ymin, dz = zmax - zmin;
    const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
    return (diag > 1.0) ? (diag * 5e-6) : 1e-6;
}

// Refine the raw output of a BOP (Cut / Fuse / Common):
//
//   1. ShapeUpgrade_UnifySameDomain merges coplanar / cosurface face
//      fragments that the BOP leaves behind. A circular hole through
//      a flat face exits the BOP as "annulus + outer remainder" two
//      faces; FreeCAD's PartDesign default Refine=true folds them
//      back into one face-with-hole. Without this cax produces +1/+1
//      faces per Cut against a flat target (Page_051 Pocket alone
//      adds +1 face; Pocket001 cylindrical-through-Z adds +4).
//
//   2. UnwrapSingleSolid peels the COMPOUND wrapper OCCT routinely
//      leaves on a Cut / Fuse / Common result that is geometrically a
//      single SOLID. Downstream BOPs / dressup prefer a SOLID arg;
//      a stray COMPOUND wrap also makes the FreeCAD-vs-cax brp_diff
//      report a spurious ShapeType mismatch.
//
//   3. The unify step's BRepTools_History is merged into `out_hist`
//      so TopoNaming::Update sees the composed lineage (orig args
//      -> bop output -> unified output) in one call. Callers that
//      don't track history pass a null `out_hist` and only see the
//      shape effect.
//
// Linear tolerance is set to 1e-6 m, matching the BOP fuzzy used
// across Cut / Fuse / Common, so the merge catches fragments that
// the BOP itself considers coincident. Angular stays at the OCCT
// default to avoid collapsing near-parallel-but-distinct planes.
TopoDS_Shape RefineBopResult(const TopoDS_Shape& raw,
                             opencascade::handle<BRepTools_History>& out_hist)
{
    if (raw.IsNull()) {
        return raw;
    }
    // A/B gate: BREPKIT_BOP_REFINE=0 skips the ShapeUpgrade_UnifySameDomain
    // pass, which the RWDI profile showed is ~1.95s -- the dominant first-party
    // boolean post-cost. Refine merges coplanar BOP fragments (FreeCAD
    // Refine=true parity) and keeps the face count down, so skipping it changes
    // topology -> validate against the geo golden (face count/volume) before
    // making anything permanent. Default ON = current behaviour.
    static const bool refine_enabled = [] {
        const char* e = std::getenv("BREPKIT_BOP_REFINE");
        return !(e && e[0] == '0');
    }();
    if (!refine_enabled) {
        return UnwrapSingleSolid(raw);
    }
    TopoDS_Shape result = raw;
    try {
        ShapeUpgrade_UnifySameDomain unify(raw,
                                            /*UnifyEdges=*/Standard_True,
                                            /*UnifyFaces=*/Standard_True,
                                            /*ConcatBSplines=*/Standard_False);
        unify.SetLinearTolerance(1e-6);
        unify.Build();
        TopoDS_Shape unified = unify.Shape();
        if (!unified.IsNull()) {
            result = unified;
            if (!out_hist.IsNull()) {
                out_hist->Merge(unify.History());
            }
        }
    } catch (...) {
        // Leave result as the unrefined BOP output -- still valid,
        // just keeps the extra fragments / COMPOUND wrap.
    }
    return UnwrapSingleSolid(result);
}

// Collect the leaf TopoDS_Edges from a list that may contain plain
// TopoDS_Edge sub-shapes or compounds. Used by both fillet entry
// paths so the retry-one-at-a-time fallback sees the same units
// the batch attempt did.
std::vector<TopoDS_Edge> CollectLeafEdges(
    const std::vector<std::shared_ptr<brepkit::TopoShape>>& edges)
{
    std::vector<TopoDS_Edge> out;
    for (auto& edge : edges) {
        if (!edge) continue;
        const TopoDS_Shape& s = edge->GetShape();
        if (s.ShapeType() == TopAbs_EDGE) {
            out.push_back(TopoDS::Edge(s));
        } else {
            for (TopExp_Explorer ex(s, TopAbs_EDGE); ex.More(); ex.Next()) {
                out.push_back(TopoDS::Edge(ex.Current()));
            }
        }
    }
    return out;
}

} // anonymous namespace

std::shared_ptr<TopoShape> TopoAlgo::Fillet(const std::shared_ptr<TopoShape>& shape, double radius,
                                            const std::vector<std::shared_ptr<TopoShape>>& edges, uint32_t op_id,
                                            const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                            const std::shared_ptr<brepdb::VersionTree>& vt)
{
    // Convert Windows SEH (access violations, divide-by-zero, FP)
    // from OCCT's inner loops into Standard_Failure so catch(...)
    // below actually catches them. Standard_True enables FP signals
    // too (CHFI3D's circle-circle intersection can divide by zero
    // when two pattern stripes overlap). Idempotent; cheap to call
    // again. MSVC also needs /EHa for catch(...) to wrap SEH; if
    // your build uses /EHsc the catch below misses raw SEH even
    // after this call -- in that case force the per-edge path with
    // the env var below.
    OSD::SetSignal(Standard_True);

    // Escape hatch: BRepKit_FILLET_FORCE_SINGLE=1 skips the batch
    // attempt entirely. ChFi3d's multi-stripe path is what blows
    // up on Page_017_Exercise2D-09 (StripeEdgeInter dereferences
    // when two stripes overlap geometrically); per-edge mode never
    // builds more than one stripe, sidestepping the bug.
    const char* force_single_env = std::getenv("BREPKIT_FILLET_FORCE_SINGLE");
    bool        force_single     = force_single_env && force_single_env[0] != '0';

    // ---- Collect the input edges ----
    std::vector<TopoDS_Edge> leaf_edges;
    if (edges.empty())
    {
        for (TopExp_Explorer ex(shape->GetShape(), TopAbs_EDGE); ex.More(); ex.Next()) {
            leaf_edges.push_back(TopoDS::Edge(ex.Current()));
        }
    }
    else
    {
        leaf_edges = CollectLeafEdges(edges);
    }

    // Dedup by (TShape + Location). When two TopoRefIR refs resolve
    // to the same cax edge (typical when cax merges what FreeCAD
    // kept as two adjacent edges -- the merged cax edge is the
    // nearest match for both refs), feeding both copies to
    // BRepFilletAPI::Add at best wastes work and at worst confuses
    // ChFi3d into a stripe-collision throw. Page_015 Fillet002 saw
    // 12 refs reduce to ~10 unique leaf edges this way.
    {
        TopTools_IndexedMapOfShape seen;
        std::vector<TopoDS_Edge>   uniq;
        uniq.reserve(leaf_edges.size());
        for (const auto& e : leaf_edges) {
            if (seen.Add(e)) {
                uniq.push_back(e);
            }
        }
        if (uniq.size() != leaf_edges.size()) {
            TA_LOG(
                "[FILLET] op_id=%u dedup'd leaf_edges %zu -> %zu\n",
                op_id, leaf_edges.size(), uniq.size());
        }
        leaf_edges = std::move(uniq);
    }

    if (leaf_edges.empty()) {
        // All refs failed to resolve, or no real edges. Skip the
        // op rather than calling fillet.Shape(), which would throw
        // StdFail_NotDone when NbContours()==0. A SKIPPED here is
        // worth the breadcrumb -- silently filleting nothing reads
        // as "fillet did nothing" downstream and you spend an hour
        // looking in the wrong place.
        TA_LOG(
            "[FILLET] op_id=%u SKIPPED -- no leaf edges to fillet\n",
            op_id);
        return shape;
    }

    // ---- Batch attempts: QuasiAngular first, then Rational ----
    //
    // Try each fillet-shape mode as a single MakeFillet call so we
    // keep the chain-fillet topology (corner blends auto-merge
    // when edges share endpoints, etc).
    //
    // QuasiAngular emits ANALYTICAL blend surfaces (gp_Torus for
    // arc edges, gp_Cylinder for line edges) where it can; Rational
    // always emits a BSpline approximation. Visually the analytical
    // form mesh-tessellates uniformly along U/V, giving a smooth
    // toroidal ring; the BSpline approximation tessellates unevenly
    // and shows up as "carved triangular patches" in the viewer
    // when Pad001's circular profile gets filleted. FreeCAD's
    // PartDesign::Fillet defaults to QuasiAngular for the same
    // reason, so we mirror that preference. Rational stays as the
    // robustness fallback: it succeeds on closed BSplines and high-
    // curvature edges where QuasiAngular's analytical branch refuses.
    auto try_batch = [&](ChFi3d_FilletShape       mode,
                         std::unique_ptr<BRepFilletAPI_MakeFillet>& out,
                         TopoDS_Shape&            out_shape) -> bool
    {
        out.reset(new BRepFilletAPI_MakeFillet(shape->GetShape()));
        out->SetFilletShape(mode);
        for (const auto& e : leaf_edges) {
            out->Add(radius, e);
        }
        int r = -1;
        try {
            // OCCT throws Standard_Failure subclasses on numeric
            // pathologies (caught here); SEH access violations
            // from ChFi3d (StripeEdgeInter etc.) are caught by
            // seh_safe_build because /EHsc would let them escape
            // catch (...) even with OSD::SetSignal installed.
            r = seh_safe_build(out.get());
        } catch (...) {
            r = -1;
        }
        if (r == 1) {
            out_shape = out->Shape();
            return true;
        }
        return false;
    };

    // ChFi3d sometimes "succeeds" on fragmented bodies (lots of small
    // co-planar / co-tangent neighbour faces, typical of a Mirror or
    // Pattern fuse seam) by building a self-intersecting blend BSpline
    // with control points orders of magnitude outside the input bbox.
    // Build() returns Done, the result is even a valid SOLID by
    // sub-shape count, but its bbox / volume are nonsense (Page_020:
    // input bbox ~0.13 m, batch fillet bbox ~8800 m, volume sign
    // flipped from +6e-4 m^3 to -0.38 m^3 because the corrupted blend
    // surface inverts the shell orientation). Per-edge mode dodges
    // this because it builds only one stripe at a time and ChFi3d's
    // cross-stripe interaction is what overflows -- so any batch
    // result whose bbox blew up vs the input gets rejected here and
    // re-tried per-edge.
    auto bbox_extent = [](const TopoDS_Shape& s) -> double {
        Bnd_Box b;
        BRepBndLib::Add(s, b);
        if (b.IsVoid()) return 0.0;
        double xmin, ymin, zmin, xmax, ymax, zmax;
        b.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        double dx = xmax - xmin, dy = ymax - ymin, dz = zmax - zmin;
        return std::max({dx, dy, dz});
    };
    const double input_extent = bbox_extent(shape->GetShape());

    std::unique_ptr<BRepFilletAPI_MakeFillet> batch_holder;
    TopoDS_Shape  result;
    bool          batch_ok = false;
    if (!force_single)
    {
        // Rational first: produces a BSpline-approximation blend
        // surface that tessellates uniformly in the viewer. The
        // alternative QuasiAngular emits analytical Torus / Cylinder
        // patches which the picker chained more finely on
        // Page_015's tangent-discontinuous contour, showing up as
        // carved triangular sub-faces in the viewer. QuasiAngular
        // stays as the fallback because it succeeds on closed
        // BSplines and high-curvature edges that Rational refuses.
        if (!try_batch(ChFi3d_Rational, batch_holder, result)) {
            if (try_batch(ChFi3d_QuasiAngular, batch_holder, result)) {
                batch_ok = true;
            }
        } else {
            batch_ok = true;
        }
    }

    // Sanity check the batch result: a fillet of radius r can grow
    // the input bbox by at most ~r per axis (the new toroidal blend
    // stays within r of the original edge). A result whose max extent
    // jumped 10x past (input + 2r) is ChFi3d's self-intersecting
    // blend pathology -- Build() returned Done and the SOLID has
    // sane sub-shape counts, but the new blend BSpline has control
    // points hundreds of metres outside the body. Caused by Mirror /
    // Pattern fuse seams that leave dozens of co-tangent neighbour
    // faces; ChFi3d's stripe builder overshoots when it tries to
    // chain across them. Page_020: input ~0.13 m, batch fillet bbox
    // ~8800 m, volume sign-flipped from +6e-4 to -0.38 m^3. 10x
    // leaves plenty of headroom for the legitimate end-cap growth
    // chained fillets occasionally add.
    const double max_legit_extent = (input_extent + 2.0 * radius) * 10.0;
    auto batch_result_sane = [&](const TopoDS_Shape& s) -> bool {
        if (input_extent <= 0.0) return true;
        return bbox_extent(s) <= max_legit_extent;
    };

    if (batch_ok)
    {
        result = UnwrapSingleSolid(result);
        if (!batch_result_sane(result))
        {
            TA_LOG(
                "[FILLET] op_id=%u batch built corrupt blend "
                "(input extent=%.4f m, result extent=%.4f m); "
                "retrying on unified-domain input\n",
                op_id, input_extent, bbox_extent(result));
            batch_ok = false;
        }
    }

    // Recovery attempt: refine the input with UnifySameDomain (merges
    // co-planar / co-tangent neighbour faces left over from Mirror /
    // Pattern fuse seams) and re-run batch fillet against the cleaner
    // body. Leaf edges get re-mapped by midpoint + tangent + length so
    // they land on the post-merge equivalents. Per-edge mode is NOT a
    // fallback here because it also calls ChFi3d on the same
    // fragmented body and reproduces the same corruption (Page_020
    // per-edge attempt also produced bbox ~1000 m). Only refining the
    // input rescues these.
    TopoDS_Shape  refined_input;
    std::unique_ptr<BRepFilletAPI_MakeFillet> refined_holder;
    bool          refined_ok = false;
    if (!batch_ok && !force_single)
    {
        ShapeUpgrade_UnifySameDomain unify(shape->GetShape(),
                                            Standard_True,  // unify edges
                                            Standard_True,  // unify faces
                                            Standard_False); // no concat bsplines
        try { unify.Build(); }
        catch (...) {}
        refined_input = unify.Shape();
        if (!refined_input.IsNull() && refined_input.NbChildren() != 0)
        {
            // Re-map each leaf edge to its surviving counterpart on
            // the refined body via (midpoint, tangent, length). An
            // edge that UnifySameDomain merged away (a phantom seam --
            // the original cause of the corruption) is silently
            // dropped; what survives is the edge ChFi3d should have
            // been filleting all along.
            std::vector<TopoDS_Edge> refined_edges;
            refined_edges.reserve(leaf_edges.size());
            for (const auto& e : leaf_edges)
            {
                if (BRep_Tool::Degenerated(e)) continue;
                gp_Pnt mid0; gp_Dir tan0; double len0 = 0.0;
                try {
                    BRepAdaptor_Curve curve(e);
                    double f = curve.FirstParameter();
                    double l = curve.LastParameter();
                    double m = 0.5 * (f + l);
                    gp_Pnt p; gp_Vec v;
                    curve.D1(m, p, v);
                    if (v.Magnitude() < 1e-12) continue;
                    mid0 = p;
                    tan0 = gp_Dir(v);
                    GProp_GProps props;
                    BRepGProp::LinearProperties(e, props);
                    len0 = props.Mass();
                } catch (...) { continue; }

                double      pt_tol = std::max(0.1 * radius, 1e-6);
                double      best_d = pt_tol;
                TopoDS_Edge best;
                for (TopExp_Explorer ex(refined_input, TopAbs_EDGE);
                     ex.More(); ex.Next())
                {
                    const TopoDS_Edge& re = TopoDS::Edge(ex.Current());
                    if (BRep_Tool::Degenerated(re)) continue;
                    try {
                        BRepAdaptor_Curve rc(re);
                        double rf = rc.FirstParameter();
                        double rl = rc.LastParameter();
                        double rm = 0.5 * (rf + rl);
                        gp_Pnt rp; gp_Vec rv;
                        rc.D1(rm, rp, rv);
                        if (rv.Magnitude() < 1e-12) continue;
                        double d = rp.Distance(mid0);
                        if (d >= best_d) continue;
                        if (std::abs(gp_Dir(rv).Dot(tan0)) < 0.996) continue;
                        GProp_GProps rprops;
                        BRepGProp::LinearProperties(re, rprops);
                        if (std::abs(rprops.Mass() - len0) >
                            0.05 * std::max(len0, 1e-6)) continue;
                        best_d = d;
                        best   = re;
                    } catch (...) { continue; }
                }
                if (!best.IsNull()) {
                    refined_edges.push_back(best);
                }
            }

            TA_LOG(
                "[FILLET] op_id=%u refined-input remap: "
                "%zu/%zu edges survived\n",
                op_id, refined_edges.size(), leaf_edges.size());

            auto try_refined = [&](ChFi3d_FilletShape mode) -> bool {
                refined_holder.reset(new BRepFilletAPI_MakeFillet(refined_input));
                refined_holder->SetFilletShape(mode);
                for (const auto& e : refined_edges) {
                    refined_holder->Add(radius, e);
                }
                int r = -1;
                try { r = seh_safe_build(refined_holder.get()); }
                catch (...) { r = -1; }
                if (r != 1) return false;
                TopoDS_Shape rr = UnwrapSingleSolid(refined_holder->Shape());
                if (!batch_result_sane(rr)) return false;
                result = rr;
                return true;
            };

            if (!refined_edges.empty())
            {
                if (try_refined(ChFi3d_Rational) ||
                    try_refined(ChFi3d_QuasiAngular))
                {
                    refined_ok = true;
                    TA_LOG(
                        "[FILLET] op_id=%u refined-input batch OK "
                        "(extent=%.4f m)\n", op_id, bbox_extent(result));
                }
            }
        }
    }

    if (batch_ok || refined_ok)
    {
        brepgraph::TopoNaming::PidMap pid_map;
        // History bookkeeping is skipped on the refined-input path:
        // chaining UnifySameDomain + MakeFillet histories together
        // needs more plumbing than the rescue is worth right now,
        // and the caller cares about geometry here, not ref stability.
        if (tn && batch_ok) {
            pid_map = tn->Update(*batch_holder, result, shape->GetShape(), op_id);
        }
        auto dst = std::make_shared<brepkit::TopoShape>(result);
        commit_to_vt(tn, vt, shape, dst, pid_map, "fillet");
        return dst;
    }

    // ---- Per-edge fallback ----
    //
    // Apply edges one at a time. Bad edges are logged and skipped;
    // good edges accumulate into `running`. Not topologically
    // identical to a batch fillet (corner blends would chain
    // differently), but the safest way to keep the rest of the
    // model alive when one edge has an OCCT pathology -- and the
    // log names the offender so it can be inspected separately.
    //
    // TopoNaming is bypassed in this path: chaining history through
    // N MakeFillet instances correctly is non-trivial, and the user
    // is checking visual geometry here, not ref stability across
    // saves.
    TA_LOG(
        "[FILLET] op_id=%u batch failed, retrying %zu edges singly\n",
        op_id, leaf_edges.size());

    auto edge_geom_str = [](const TopoDS_Edge& e) -> std::string {
        char buf[256];
        if (BRep_Tool::Degenerated(e)) { return "DEGENERATE"; }
        try {
            BRepAdaptor_Curve curve(e);
            double f = curve.FirstParameter();
            double l = curve.LastParameter();
            gp_Pnt p_start = curve.Value(f);
            gp_Pnt p_mid   = curve.Value(0.5 * (f + l));
            gp_Pnt p_end   = curve.Value(l);
            GProp_GProps props;
            BRepGProp::LinearProperties(e, props);
            std::snprintf(buf, sizeof(buf),
                "mid=(%.4f,%.4f,%.4f) start=(%.4f,%.4f,%.4f) "
                "end=(%.4f,%.4f,%.4f) len=%.4f",
                p_mid.X(), p_mid.Y(), p_mid.Z(),
                p_start.X(), p_start.Y(), p_start.Z(),
                p_end.X(), p_end.Y(), p_end.Z(),
                props.Mass());
            return buf;
        } catch (...) {
            return "UNREADABLE";
        }
    };

    // Try one edge with one fillet-shape mode. Returns true on a
    // clean Build()+IsDone(), false if it throws OR IsDone=false.
    // On success the caller advances `running` to the new shape.
    auto try_one = [&](TopoDS_Shape&        working,
                       const TopoDS_Edge&   e,
                       ChFi3d_FilletShape   mode,
                       TopoDS_Shape&        out_result) -> bool
    {
        try {
            BRepFilletAPI_MakeFillet f(working);
            f.SetFilletShape(mode);
            f.Add(radius, e);
            int r = -1;
            try {
                r = seh_safe_build(&f);
            } catch (...) {
                r = -1;
            }
            if (r == 1) {
                out_result = UnwrapSingleSolid(f.Shape());
                return true;
            }
        } catch (...) {
            // fall through to false
        }
        return false;
    };

    TopoDS_Shape running = shape->GetShape();

    auto count_faces = [](const TopoDS_Shape& s) -> int {
        int n = 0;
        for (TopExp_Explorer ex(s, TopAbs_FACE); ex.More(); ex.Next()) ++n;
        return n;
    };
    const int base_face_count = count_faces(running);

    // Midpoint + tangent + length of an edge. Returns false for
    // degenerate / unreadable edges. Used both for the failure-list
    // log line and to rematch a leaf edge against the post-fillet
    // running body in the retry pass.
    auto edge_geom = [](const TopoDS_Edge& e,
                        gp_Pnt& mid, gp_Dir& tan, double& len) -> bool {
        if (BRep_Tool::Degenerated(e)) return false;
        try {
            BRepAdaptor_Curve curve(e);
            double f = curve.FirstParameter();
            double l = curve.LastParameter();
            double m = 0.5 * (f + l);
            gp_Pnt p;  gp_Vec v;
            curve.D1(m, p, v);
            if (v.Magnitude() < 1e-12) return false;
            mid = p;
            tan = gp_Dir(v);
            GProp_GProps props;
            BRepGProp::LinearProperties(e, props);
            len = props.Mass();
            return true;
        } catch (...) {
            return false;
        }
    };

    // On a possibly-mutated body, find the edge whose midpoint sits
    // close to pt0, whose unit tangent is parallel (or anti-parallel)
    // to tan0, and whose length fits the "could have shrunk from len0"
    // envelope. A neighbour fillet chamfering one endpoint shrinks
    // this edge by `radius`; chamfering both endpoints shrinks it by
    // 2*radius. The midpoint shifts by radius/2 when only ONE end
    // shrinks (asymmetric); the old 0.1*radius position budget was
    // tighter than that and silently mislabeled any surviving-but-
    // shortened neighbour as "consumed by prior chain", leaving sharp
    // edges where the user asked for fillets. Allow midpoint drift up
    // to `radius` and length shrinkage up to 2*radius. Forbid growth
    // -- a real surviving edge can only get shorter than its original.
    auto find_surviving_edge = [&](const TopoDS_Shape& body,
                                   const gp_Pnt&        pt0,
                                   const gp_Dir&        tan0,
                                   double               len0,
                                   TopoDS_Edge&         out_edge) -> bool {
        double pt_tol = std::max(radius, 1e-6);
        const double tan_dot_min = 0.996; // ~5 degrees
        const double len_slack   = std::max(1e-3 * radius, 1e-9);
        const double len_min     = len0 - 2.0 * radius - len_slack;
        const double len_max_v   = len0 + len_slack;

        double      best_d = pt_tol;
        TopoDS_Edge best;
        for (TopExp_Explorer ex(body, TopAbs_EDGE); ex.More(); ex.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(ex.Current());
            gp_Pnt mid; gp_Dir tan; double len;
            if (!edge_geom(e, mid, tan, len)) continue;
            if (len < len_min || len > len_max_v) continue;
            if (std::abs(tan.Dot(tan0)) < tan_dot_min) continue;
            double d = mid.Distance(pt0);
            if (d >= best_d) continue;
            best_d = d;
            best   = e;
        }
        if (best.IsNull()) return false;
        out_edge = best;
        return true;
    };

    // Try each edge in order. Track failures so we can re-try them
    // later: per-edge mode is order-dependent (each fillet mutates
    // `running`, changing the topology that downstream edges see),
    // and an edge that ChFi3d rejects on the unmodified body can
    // sometimes succeed after a neighbouring fillet has tidied up
    // the local stripe geometry. We saw this on Page_015 Fillet002
    // edge[0] (163mm closed BSpline at z=+0.02) failing both modes
    // while edge[5] (the same geometry mirrored to z=-0.02)
    // succeeded with Rational a few iterations later.
    auto try_both_modes = [&](const TopoDS_Edge& e,
                              TopoDS_Shape&      out_result,
                              const char*&       out_mode) -> bool
    {
        // Rational first to match the batch-mode preference --
        // gives one BSpline blend per chained tangent group; switch
        // to QuasiAngular only when Rational refuses (closed BSpline
        // edges, high-curvature pathologies).
        if (try_one(running, e, ChFi3d_Rational, out_result)) {
            out_mode = "Rational";
            return true;
        }
        if (try_one(running, e, ChFi3d_QuasiAngular, out_result)) {
            out_mode = "QuasiAngular";
            return true;
        }
        return false;
    };

    int          ok_count = 0;
    std::vector<size_t> failed;
    for (size_t i = 0; i < leaf_edges.size(); ++i)
    {
        std::string geom = edge_geom_str(leaf_edges[i]);

        TopoDS_Shape result;
        const char*  mode = nullptr;
        if (try_both_modes(leaf_edges[i], result, mode))
        {
            running = result;
            ++ok_count;
            TA_LOG( "[FILLET]   edge[%zu] ok (%s) %s\n",
                         i, mode, geom.c_str());
        }
        else
        {
            failed.push_back(i);
            TA_LOG(
                "[FILLET]   edge[%zu] failed first pass %s\n",
                i, geom.c_str());
        }
    }

    // Retry pass: most "failed first pass" edges are actually fine
    // -- BRepFilletAPI::Add auto-chains tangent neighbours into the
    // same contour, so one successful Add typically consumes several
    // leaf edges. Subsequent Add calls with those swallowed edges
    // fail not because ChFi3d can't fillet them but because they no
    // longer exist on the running body. Re-resolve each failed leaf
    // against the now-modified body by (mid, tangent, length); if
    // the original edge survived we retry on the live one (which
    // can finally succeed now that surrounding topology has settled),
    // and if it didn't we record it as chain-consumed and move on.
    int retry_round         = 0;
    int chain_consumed      = 0;
    int chain_consumed_log  = 0;
    while (!failed.empty())
    {
        ++retry_round;
        std::vector<size_t> still_failed;
        int round_ok = 0;
        for (size_t i : failed)
        {
            gp_Pnt pt0; gp_Dir tan0; double len0;
            if (!edge_geom(leaf_edges[i], pt0, tan0, len0)) {
                still_failed.push_back(i);
                continue;
            }
            TopoDS_Edge live;
            if (!find_surviving_edge(running, pt0, tan0, len0, live)) {
                ++chain_consumed;
                if (chain_consumed_log < 8) {
                    TA_LOG(
                        "[FILLET]   edge[%zu] consumed by prior chain "
                        "(no live match on running body)\n", i);
                    ++chain_consumed_log;
                }
                continue;
            }

            std::string geom = edge_geom_str(live);
            TopoDS_Shape result;
            const char*  mode = nullptr;
            if (try_both_modes(live, result, mode))
            {
                running = result;
                ++ok_count;
                ++round_ok;
                TA_LOG(
                    "[FILLET]   edge[%zu] ok (%s, retry round %d, rematched) %s\n",
                    i, mode, retry_round, geom.c_str());
            }
            else
            {
                still_failed.push_back(i);
            }
        }
        if (round_ok == 0)
        {
            // No fillet actually ran this round. The next round
            // would see the same body and decide the same way --
            // log truly-stuck edges and stop. Chain-consumed ones
            // are not failures and were already removed.
            for (size_t i : still_failed)
            {
                std::string geom = edge_geom_str(leaf_edges[i]);
                TA_LOG(
                    "[FILLET]   edge[%zu] FAILED both modes after %d retries %s\n",
                    i, retry_round, geom.c_str());
            }
            break;
        }
        failed = std::move(still_failed);
    }

    // Report what ChFi3d actually produced on the body, not just how
    // many Add() calls returned true. Each successful Add can chain
    // several tangent edges into one contour, so "Add ok" undercounts
    // coverage; the new-surface delta is the honest measure.
    const int new_surface_count = count_faces(running) - base_face_count;
    TA_LOG(
        "[FILLET] op_id=%u per-edge: %d Add() ok + %d chain-consumed, "
        "%d new fillet surfaces / %zu edges (%d retry rounds)\n",
        op_id, ok_count, chain_consumed, new_surface_count,
        leaf_edges.size(), retry_round);

    if (ok_count == 0) {
        return shape;
    }

    auto dst = std::make_shared<brepkit::TopoShape>(running);
    commit_to_vt(tn, vt, shape, dst, {}, "fillet");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Chamfer(const std::shared_ptr<TopoShape>& shape, double dist,
                                             const std::vector<std::shared_ptr<TopoShape>>& edges, uint32_t op_id,
                                             const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                             const std::shared_ptr<brepdb::VersionTree>& vt)
{
    // BRepFilletAPI_MakeChamfer shares ChFi3d_Builder with MakeFillet,
    // and the same stripe / vertex-corner crashes that bit Fillet hit
    // chamfer too. Page_045_Exercise2D-37_byHannu (Chamfer with 10
    // edges, d=0.003) is the canonical repro -- ChFi3d_IsInFront
    // dereferences bad state inside PerformTwoCornerbyInter when
    // several chamfered edges meet at one vertex. Mirror Fillet's
    // scaffolding: convert SEH to Standard_Failure, dedup leaf edges,
    // try batch then per-edge fallback. See Fillet for the rationale
    // on each step; this function is intentionally parallel to it.
    OSD::SetSignal(Standard_True);

    // Build-stamp probe: if the user sees this line in stderr, the
    // editor.exe really did pick up the SEH-wrapped Chamfer. If the
    // line is missing, the binary is stale (relink editor.vcxproj).
    TA_LOG(
        "[CHAMFER] op_id=%u entry, build=" __DATE__ " " __TIME__
        ", dist=%.6f, edges=%zu (BREPKIT_CHAMFER_FORCE_SINGLE=%s)\n",
        op_id, dist, edges.size(),
        std::getenv("BREPKIT_CHAMFER_FORCE_SINGLE") ? std::getenv("BREPKIT_CHAMFER_FORCE_SINGLE") : "<unset>");

    const char* force_single_env = std::getenv("BREPKIT_CHAMFER_FORCE_SINGLE");
    bool        force_single     = force_single_env && force_single_env[0] != '0';

    std::vector<TopoDS_Edge> leaf_edges;
    if (edges.empty())
    {
        for (TopExp_Explorer ex(shape->GetShape(), TopAbs_EDGE); ex.More(); ex.Next()) {
            leaf_edges.push_back(TopoDS::Edge(ex.Current()));
        }
    }
    else
    {
        leaf_edges = CollectLeafEdges(edges);
    }

    {
        TopTools_IndexedMapOfShape seen;
        std::vector<TopoDS_Edge>   uniq;
        uniq.reserve(leaf_edges.size());
        for (const auto& e : leaf_edges) {
            if (seen.Add(e)) uniq.push_back(e);
        }
        if (uniq.size() != leaf_edges.size()) {
            TA_LOG(
                "[CHAMFER] op_id=%u dedup'd leaf_edges %zu -> %zu\n",
                op_id, leaf_edges.size(), uniq.size());
        }
        leaf_edges = std::move(uniq);
    }

    if (leaf_edges.empty()) {
        TA_LOG(
            "[CHAMFER] op_id=%u SKIPPED -- no leaf edges to chamfer\n",
            op_id);
        return shape;
    }

    auto try_batch = [&](std::unique_ptr<BRepFilletAPI_MakeChamfer>& out,
                         TopoDS_Shape&                                out_shape) -> bool
    {
        out.reset(new BRepFilletAPI_MakeChamfer(shape->GetShape()));
        for (const auto& e : leaf_edges) {
            out->Add(dist, e);
        }
        TA_LOG(
            "[CHAMFER] op_id=%u about to seh_safe_build batch\n", op_id);
        std::fflush(stderr);
        int r = -1;
        try {
            // seh_safe_build handles the Win32 SEH path (ChFi3d
            // access violations). C++ Standard_Failure throws
            // (numeric pathologies, contour collisions) still
            // unwind out and land in the catch (...) below.
            r = seh_safe_build(out.get());
        } catch (...) {
            r = -1;
        }
        TA_LOG(
            "[CHAMFER] op_id=%u batch seh_safe_build returned r=%d (%s)\n",
            op_id, r, seh_safe_build_label(r));
        if (r == 1) {
            out_shape = out->Shape();
            return true;
        }
        return false;
    };

    std::unique_ptr<BRepFilletAPI_MakeChamfer> batch_holder;
    TopoDS_Shape result;
    bool         batch_ok = false;
    if (!force_single) {
        batch_ok = try_batch(batch_holder, result);
    }

    if (batch_ok) {
        brepgraph::TopoNaming::PidMap pid_map;
        if (tn) {
            pid_map = tn->Update(*batch_holder, result, shape->GetShape(), op_id);
        }
        auto dst = std::make_shared<brepkit::TopoShape>(result);
        commit_to_vt(tn, vt, shape, dst, pid_map, "chamfer");
        return dst;
    }

    // ---- Per-edge fallback ----
    //
    // Same trade-off as Fillet: chaining N MakeChamfer instances
    // through TopoNaming correctly is non-trivial, so the fallback
    // bypasses naming. Acceptable here because the user is verifying
    // geometry, not ref stability across saves.
    TA_LOG(
        "[CHAMFER] op_id=%u batch failed, retrying %zu edges singly\n",
        op_id, leaf_edges.size());

    auto edge_geom_str = [](const TopoDS_Edge& e) -> std::string {
        char buf[256];
        if (BRep_Tool::Degenerated(e)) return "DEGENERATE";
        try {
            BRepAdaptor_Curve curve(e);
            double f = curve.FirstParameter();
            double l = curve.LastParameter();
            gp_Pnt p_mid = curve.Value(0.5 * (f + l));
            GProp_GProps props;
            BRepGProp::LinearProperties(e, props);
            std::snprintf(buf, sizeof(buf),
                "mid=(%.4f,%.4f,%.4f) len=%.4f",
                p_mid.X(), p_mid.Y(), p_mid.Z(), props.Mass());
            return buf;
        } catch (...) { return "UNREADABLE"; }
    };

    auto edge_geom = [](const TopoDS_Edge& e,
                        gp_Pnt& mid, gp_Dir& tan, double& len) -> bool {
        if (BRep_Tool::Degenerated(e)) return false;
        try {
            BRepAdaptor_Curve curve(e);
            double f = curve.FirstParameter();
            double l = curve.LastParameter();
            double m = 0.5 * (f + l);
            gp_Pnt p;  gp_Vec v;
            curve.D1(m, p, v);
            if (v.Magnitude() < 1e-12) return false;
            mid = p; tan = gp_Dir(v);
            GProp_GProps props;
            BRepGProp::LinearProperties(e, props);
            len = props.Mass();
            return true;
        } catch (...) { return false; }
    };

    // Match against the running body's edges. The thresholds account
    // for the geometric distortion that prior per-edge chamfers apply
    // to neighbour edges sharing a vertex:
    //   - length: a neighbour chamfered at ONE end shrinks by `dist`;
    //     chamfered at BOTH ends shrinks by 2*dist. Allow up to 2*dist
    //     of shrinkage (with a small numerical slack) and forbid
    //     growth -- a real surviving edge can only get shorter.
    //   - midpoint: when only ONE end shrinks (asymmetric), the
    //     midpoint shifts by dist/2 along the edge; the old 0.1*dist
    //     budget was 5x too tight, killing every asymmetrically-eaten
    //     neighbour. Use `dist` (covers up to 2*dist shrinkage on one
    //     side).
    //   - tangent: 0.996 (~5 deg) unchanged; a real edge can't change
    //     direction.
    // The old `len_rel_tol = 0.05` (5%) threshold turned every 60-70 mm
    // edge with neighbour chamfers on both ends into "chain-consumed"
    // (Page_045 lost 23/40 vertical posts because 2*3 mm / 60 mm = 10%).
    auto find_surviving_edge = [&](const TopoDS_Shape& body,
                                   const gp_Pnt&       pt0,
                                   const gp_Dir&       tan0,
                                   double              len0,
                                   TopoDS_Edge&        out_edge) -> bool {
        double pt_tol = std::max(dist, 1e-6);
        const double tan_dot_min = 0.996;
        const double len_slack   = std::max(1e-3 * dist, 1e-9);
        const double len_min     = len0 - 2.0 * dist - len_slack;
        const double len_max     = len0 + len_slack;
        double      best_d = pt_tol;
        TopoDS_Edge best;
        for (TopExp_Explorer ex(body, TopAbs_EDGE); ex.More(); ex.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(ex.Current());
            gp_Pnt mid; gp_Dir tan; double len;
            if (!edge_geom(e, mid, tan, len)) continue;
            if (len < len_min || len > len_max) continue;
            if (std::abs(tan.Dot(tan0)) < tan_dot_min) continue;
            double d = mid.Distance(pt0);
            if (d >= best_d) continue;
            best_d = d;
            best   = e;
        }
        if (best.IsNull()) return false;
        out_edge = best;
        return true;
    };

    TopoDS_Shape running = shape->GetShape();

    auto count_faces = [](const TopoDS_Shape& s) -> int {
        int n = 0;
        for (TopExp_Explorer ex(s, TopAbs_FACE); ex.More(); ex.Next()) ++n;
        return n;
    };
    const int base_face_count = count_faces(running);

    auto try_one = [&](TopoDS_Shape&      working,
                       const TopoDS_Edge& e,
                       TopoDS_Shape&      out_result) -> bool {
        // MakeChamfer ctor + Add stay in the C++ try (can throw
        // Standard_Failure on bad input geometry); the actual
        // Build is routed through seh_safe_build so a ChFi3d
        // access violation on this edge becomes a false return
        // and the caller moves on to the next edge.
        try {
            BRepFilletAPI_MakeChamfer c(working);
            c.Add(dist, e);
            int r = -1;
            try {
                r = seh_safe_build(&c);
            } catch (...) {
                r = -1;
            }
            if (r == 1) {
                out_result = c.Shape();
                return true;
            }
        } catch (...) {
            // fall through to false
        }
        return false;
    };

    int                 ok_count = 0;
    std::vector<size_t> failed;
    for (size_t i = 0; i < leaf_edges.size(); ++i) {
        std::string  geom = edge_geom_str(leaf_edges[i]);
        TopoDS_Shape res;
        if (try_one(running, leaf_edges[i], res)) {
            running = res;
            ++ok_count;
            TA_LOG( "[CHAMFER]   edge[%zu] ok %s\n",
                         i, geom.c_str());
        } else {
            failed.push_back(i);
            TA_LOG(
                "[CHAMFER]   edge[%zu] failed first pass %s\n",
                i, geom.c_str());
        }
    }

    int retry_round        = 0;
    int chain_consumed     = 0;
    int chain_consumed_log = 0;
    while (!failed.empty()) {
        ++retry_round;
        std::vector<size_t> still_failed;
        int round_ok = 0;
        for (size_t i : failed) {
            gp_Pnt pt0; gp_Dir tan0; double len0;
            if (!edge_geom(leaf_edges[i], pt0, tan0, len0)) {
                still_failed.push_back(i);
                continue;
            }
            TopoDS_Edge live;
            if (!find_surviving_edge(running, pt0, tan0, len0, live)) {
                ++chain_consumed;
                if (chain_consumed_log < 8) {
                    TA_LOG(
                        "[CHAMFER]   edge[%zu] consumed by prior chain "
                        "(no live match on running body)\n", i);
                    ++chain_consumed_log;
                }
                continue;
            }
            std::string  geom = edge_geom_str(live);
            TopoDS_Shape res;
            if (try_one(running, live, res)) {
                running = res;
                ++ok_count;
                ++round_ok;
                TA_LOG(
                    "[CHAMFER]   edge[%zu] ok (retry round %d, rematched) %s\n",
                    i, retry_round, geom.c_str());
            } else {
                still_failed.push_back(i);
            }
        }
        if (round_ok == 0) {
            for (size_t i : still_failed) {
                std::string geom = edge_geom_str(leaf_edges[i]);
                TA_LOG(
                    "[CHAMFER]   edge[%zu] FAILED after %d retries %s\n",
                    i, retry_round, geom.c_str());
            }
            break;
        }
        failed = std::move(still_failed);
    }

    const int new_surface_count = count_faces(running) - base_face_count;
    TA_LOG(
        "[CHAMFER] op_id=%u per-edge: %d Add() ok + %d chain-consumed, "
        "%d new chamfer surfaces / %zu edges (%d retry rounds)\n",
        op_id, ok_count, chain_consumed, new_surface_count,
        leaf_edges.size(), retry_round);

    if (ok_count == 0) {
        return shape;
    }

    auto dst = std::make_shared<brepkit::TopoShape>(running);
    commit_to_vt(tn, vt, shape, dst, {}, "chamfer");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Prism(const std::shared_ptr<TopoShape>& face, double x, double y, double z,
                                           uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                           const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepPrimAPI_MakePrism prism(face->GetShape(), gp_Vec(x, y, z));

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(prism, prism.Shape(), face->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(prism.Shape());
    commit_to_vt(tn, vt, face, dst, pid_map, "prism");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Split(const std::shared_ptr<TopoShape>& base, const std::shared_ptr<TopoShape>& tool,
                                           uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                           const std::shared_ptr<brepdb::VersionTree>& vt)
{
    TopTools_ListOfShape bases, tools;
    bases.Append(base->GetShape());
    tools.Append(tool->GetShape());

    BRepAlgoAPI_Splitter algo;
    algo.SetArguments(bases);
    algo.SetTools(tools);
    algo.Build();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cerr);
    }

    // Serialize tool BEFORE tn->Update() unbinds its shapes
    brepdb::BRepWorld tool_world;
    if (tn && vt) { tool_world = serialize_world(tn, tool->GetShape()); }

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = ShapeBuilder::MakeCompound({ base, tool });
        opencascade::handle<BRepTools_History> o_hist = algo.History();
        pid_map = tn->Update(o_hist, algo.Shape(), old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(algo.Shape());
    merge_to_vt(tn, vt, base, tool, dst, std::move(tool_world), pid_map, "split");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Cut(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
                                         uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                         const std::shared_ptr<brepdb::VersionTree>& vt)
{
    // 1e-6 m fuzzy, mirrored from Fuse below. Without it OCCT silently
    // returns the base shape unchanged whenever the tool touches an
    // existing face plane-on (e.g. Page_031's sphere equator sitting
    // on the Pad's bottom face) -- the boolean "looks like it didn't
    // happen" because the unsubtracted base flows out as the result.
    BRepAlgoAPI_Cut algo;
    TopTools_ListOfShape args;  args.Append(s1->GetShape());
    TopTools_ListOfShape tools; tools.Append(s2->GetShape());
    algo.SetArguments(args);
    algo.SetTools(tools);
    algo.SetFuzzyValue(ScaledBopFuzzy(s1->GetShape(), s2->GetShape()));
    const auto _bop_t0 = std::chrono::steady_clock::now();
    algo.Build();
    const auto _bop_t1 = std::chrono::steady_clock::now();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cerr);
    }

    // Refine: unify coplanar BOP fragments + unwrap a single-SOLID
    // COMPOUND wrapper. Mirrors FreeCAD PartDesign Pocket default
    // Refine=true. See RefineBopResult above for rationale.
    opencascade::handle<BRepTools_History> hist;
    if (tn) hist = algo.History();
    const auto _ref_t0 = std::chrono::steady_clock::now();
    TopoDS_Shape refined = RefineBopResult(algo.Shape(), hist);
    if (bop_prof_on()) {
        using _ms = std::chrono::duration<double, std::milli>;
        const auto _ref_t1 = std::chrono::steady_clock::now();
        bop_prof_log("cut", _ms(_bop_t1 - _bop_t0).count(),
                     _ms(_ref_t1 - _ref_t0).count(),
                     BopProfFaceCount(algo.Shape()), BopProfFaceCount(refined));
    }

    brepdb::BRepWorld tool_world;
    if (tn && vt) { tool_world = serialize_world(tn, s2->GetShape()); }

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = ShapeBuilder::MakeCompound({ s1, s2 });
        pid_map = tn->Update(hist, refined, old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(refined);
    merge_to_vt(tn, vt, s1, s2, dst, std::move(tool_world), pid_map, "cut");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Fuse(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
                                          uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                          const std::shared_ptr<brepdb::VersionTree>& vt)
{
    // 1e-6 m fuzzy absorbs precision noise so face-coincident operands
    // (e.g. an extruded pad with an r=R cylindrical hole fused against
    // an annulus whose inner radius is also R) intersect correctly
    // rather than silently returning an empty COMPOUND.
    auto run_fuse = [&](double fuzzy) {
        auto a = std::make_unique<BRepAlgoAPI_Fuse>();
        TopTools_ListOfShape args;  args.Append(s1->GetShape());
        TopTools_ListOfShape tools; tools.Append(s2->GetShape());
        a->SetArguments(args);
        a->SetTools(tools);
        a->SetFuzzyValue(fuzzy);
        a->Build();
        return a;
    };

    const double base_fuzzy =
        ScaledBopFuzzy(s1->GetShape(), s2->GetShape());
    const auto _bop_t0 = std::chrono::steady_clock::now();
    auto algo = run_fuse(base_fuzzy);
    if (!algo->IsDone()) {
        algo->DumpErrors(std::cerr);
    }

    // When the BOP returns a COMPOUND wrapping multiple SOLIDs but
    // the operands' bboxes overlap (so a single merged SOLID is the
    // expected answer), OCCT's surface-intersection step dropped
    // below the default fuzzy and left them as disjoint pieces. Seen
    // on Page_033 Revolution+Pad: Revolution has an annular hollow
    // at x in [-29, -25] that the Pad cylinder fills, and OCCT left
    // Revolution's inner cylindrical face as an internal boundary
    // instead of removing it. Retry with progressively larger fuzzy
    // until the solid count drops; keep whichever algo run produced
    // the best (smallest) compound so its History stays valid for
    // TopoNaming::Update below.
    const bool is_retry_candidate =
        algo->IsDone() && !algo->Shape().IsNull() &&
        algo->Shape().ShapeType() == TopAbs_COMPOUND &&
        CountSolids(algo->Shape()) > 1 &&
        BboxesOverlap(s1->GetShape(), s2->GetShape());

    if (is_retry_candidate && bop_retry_mode() != 0)
    {
        const bool fast = (bop_retry_mode() == 2);
        auto solid_volume = [](const TopoDS_Shape& s) -> double {
            if (s.IsNull()) return 0.0;
            GProp_GProps gp;
            BRepGProp::VolumeProperties(s, gp);
            return gp.Mass();
        };
        const int start_solids = CountSolids(algo->Shape());
        int       best_solids  = start_solids;
        double    base_volume  = solid_volume(algo->Shape());
        int       retries_ran = 0, retries_accepted = 0;
        const auto _t0 = std::chrono::steady_clock::now();
        for (double fuzzy : {base_fuzzy * 10.0, base_fuzzy * 100.0,
                             base_fuzzy * 1000.0}) {
            auto retry = run_fuse(fuzzy);
            ++retries_ran;
            if (!retry->IsDone() || retry->Shape().IsNull()) continue;
            int retry_solids = CountSolids(retry->Shape());
            // Accept a fragment merge (fewer solids) ONLY when it also
            // preserves the body's volume. A larger fuzzy that DROPS the
            // volume is not gluing coincident faces -- it is melting features
            // thinner than the fuzzy and collapsing the body to a degenerate
            // remnant. That remnant has a low solid count, so the bare
            // "fewest solids wins" rule used to PREFER it over the correct
            // result: a pattern fuse onto a body carrying 0.1mm pins escalated
            // to a 1mm fuzzy, wiped the whole part to a sliver (1 solid), and
            // that sliver won -- R2900_30 rebuilt to "nothing displayed".
            // A real merge keeps the union volume; require that.
            double retry_volume = solid_volume(retry->Shape());
            if (retry_solids < best_solids &&
                retry_volume >= 0.9 * base_volume) {
                algo = std::move(retry);
                best_solids = retry_solids;
                ++retries_accepted;
                if (best_solids <= 1) break;
            }
            else if (fast) {
                // FAST mode: this fuzzy step didn't yield an accepted merge.
                // A genuinely-disjoint multi-solid never merges at any fuzzy,
                // so escalating further just burns full booleans (the measured
                // waste: ran=3 accepted=0). Stop here. Validated result-
                // preserving against the committed geo goldens.
                break;
            }
        }
        // Instrumentation: how often this fires and what it costs. A line
        // with accepted=0 is a fully-wasted retry burst (legit multi-solid);
        // accepted>0 means a fuzzy escalation actually merged a spurious gap.
        const double _ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - _t0).count();
        std::fprintf(stderr,
            "[bop_retry] op_id=%u solids %d->%d  ran=%d accepted=%d  %.1fms\n",
            op_id, start_solids, best_solids, retries_ran, retries_accepted, _ms);
        std::fflush(stderr);
    }
    else if (is_retry_candidate)
    {
        // Retry disabled by env -- record that this fuse WOULD have retried,
        // so the A/B (BREPKIT_BOP_RETRY=0) still shows how many candidates exist.
        std::fprintf(stderr,
            "[bop_retry] op_id=%u solids=%d candidate (RETRY DISABLED)\n",
            op_id, CountSolids(algo->Shape()));
        std::fflush(stderr);
    }

    const auto _bop_t1 = std::chrono::steady_clock::now();

    // Refine: unify coplanar BOP fragments + unwrap a single-SOLID
    // COMPOUND wrapper. Mirrors FreeCAD PartDesign Pad default
    // Refine=true.
    opencascade::handle<BRepTools_History> hist;
    if (tn) hist = algo->History();
    const auto _ref_t0 = std::chrono::steady_clock::now();
    TopoDS_Shape refined = RefineBopResult(algo->Shape(), hist);
    if (bop_prof_on()) {
        using _ms = std::chrono::duration<double, std::milli>;
        const auto _ref_t1 = std::chrono::steady_clock::now();
        bop_prof_log("fuse", _ms(_bop_t1 - _bop_t0).count(),
                     _ms(_ref_t1 - _ref_t0).count(),
                     BopProfFaceCount(algo->Shape()), BopProfFaceCount(refined));
    }

    brepdb::BRepWorld tool_world;
    if (tn && vt) { tool_world = serialize_world(tn, s2->GetShape()); }

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = ShapeBuilder::MakeCompound({ s1, s2 });
        pid_map = tn->Update(hist, refined, old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(refined);
    merge_to_vt(tn, vt, s1, s2, dst, std::move(tool_world), pid_map, "fuse");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Common(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
                                            uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                            const std::shared_ptr<brepdb::VersionTree>& vt)
{
    // Same fuzzy-tolerance reasoning as Cut / Fuse above.
    BRepAlgoAPI_Common algo;
    TopTools_ListOfShape args;  args.Append(s1->GetShape());
    TopTools_ListOfShape tools; tools.Append(s2->GetShape());
    algo.SetArguments(args);
    algo.SetTools(tools);
    algo.SetFuzzyValue(ScaledBopFuzzy(s1->GetShape(), s2->GetShape()));
    const auto _bop_t0 = std::chrono::steady_clock::now();
    algo.Build();
    const auto _bop_t1 = std::chrono::steady_clock::now();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cerr);
    }

    // Refine: same rationale as Cut / Fuse.
    opencascade::handle<BRepTools_History> hist;
    if (tn) hist = algo.History();
    const auto _ref_t0 = std::chrono::steady_clock::now();
    TopoDS_Shape refined = RefineBopResult(algo.Shape(), hist);
    if (bop_prof_on()) {
        using _ms = std::chrono::duration<double, std::milli>;
        const auto _ref_t1 = std::chrono::steady_clock::now();
        bop_prof_log("common", _ms(_bop_t1 - _bop_t0).count(),
                     _ms(_ref_t1 - _ref_t0).count(),
                     BopProfFaceCount(algo.Shape()), BopProfFaceCount(refined));
    }

    brepdb::BRepWorld tool_world;
    if (tn && vt) { tool_world = serialize_world(tn, s2->GetShape()); }

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = ShapeBuilder::MakeCompound({ s1, s2 });
        pid_map = tn->Update(hist, refined, old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(refined);
    merge_to_vt(tn, vt, s1, s2, dst, std::move(tool_world), pid_map, "common");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Section(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
                                             uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                             const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepAlgoAPI_Section algo;
    algo.Init1(s1->GetShape());
    algo.Init2(s2->GetShape());
    algo.Approximation(Standard_False);
    algo.Build();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cerr);
    }

    brepdb::BRepWorld tool_world;
    if (tn && vt) { tool_world = serialize_world(tn, s2->GetShape()); }

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = ShapeBuilder::MakeCompound({ s1, s2 });
        opencascade::handle<BRepTools_History> o_hist = algo.History();
        pid_map = tn->Update(o_hist, algo.Shape(), old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(algo.Shape());
    merge_to_vt(tn, vt, s1, s2, dst, std::move(tool_world), pid_map, "section");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Sew(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
    uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
    const std::shared_ptr<brepdb::VersionTree>& vt)
{
    Standard_Real tolerance = 1e-6;
    BRepBuilderAPI_Sewing algo(tolerance);

    algo.Add(s1->GetShape());
    algo.Add(s2->GetShape());

    algo.Perform();

    //if (tn)
    //{
    //    auto old_shp = ShapeBuilder::MakeCompound({ s1, s2 });
    //    opencascade::handle<BRepTools_History> o_hist = algo.History();
    //    auto upd_hist_graph = [&](const std::shared_ptr<brepgraph::HistGraph>& hg)
    //    {
    //        if (!hg) {
    //            return;
    //        }
    //        auto type = trans_type(hg->GetType());
    //        ShapeHistory hist(o_hist, type, algo.Shape(), s1->GetShape());
    //        hg->Update(hist, op_id);
    //    };
    //    upd_hist_graph(tn->GetEdgeGraph());
    //    upd_hist_graph(tn->GetFaceGraph());
    //    upd_hist_graph(tn->GetSolidGraph());
    //}

    auto shp = algo.SewedShape();
    auto type = shp.ShapeType();

    return std::make_shared<brepkit::TopoShape>(algo.SewedShape());
}

std::shared_ptr<TopoShape> TopoAlgo::UnifySameDomain(const std::shared_ptr<TopoShape>& shape,
                                                     uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                                     const std::shared_ptr<brepdb::VersionTree>& vt)
{
    ShapeUpgrade_UnifySameDomain algo;
    algo.Initialize(shape->GetShape(), Standard_True, Standard_True, Standard_False);
    algo.Build();

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn)
    {
        opencascade::handle<BRepTools_History> o_hist = algo.History();
        pid_map = tn->Update(o_hist, algo.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(algo.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "unify_same_domain");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::SplitBodyAtPoints(
    const std::shared_ptr<TopoShape>&             shape,
    const std::vector<sm::vec3>&                  points,
    uint32_t                                      op_id,
    const std::shared_ptr<brepgraph::TopoNaming>& tn,
    const std::shared_ptr<brepdb::VersionTree>&   vt)
{
    if (!shape || shape->GetShape().IsNull()) return shape;
    if (points.empty())                       return shape;

    // Filter hint points to those that actually lie on an edge of
    // the body and are NOT already at a vertex. Use a sub-mm
    // tolerance (BOP fuzzy is typically below this; FreeCAD-saved
    // BReps land within microns of cax-computed midpoints when
    // edges line up). Off-body points and already-vertex points
    // are silently dropped -- adding them as tools to
    // BOPAlgo_Splitter is either a no-op or an error.
    const double on_edge_tol  = 1e-4;     // 0.1 mm in metres
    const double at_vertex_tol = 1e-5;    // 10 microns

    TopTools_IndexedMapOfShape vert_map;
    TopExp::MapShapes(shape->GetShape(), TopAbs_VERTEX, vert_map);

    TopTools_IndexedMapOfShape edge_map;
    TopExp::MapShapes(shape->GetShape(), TopAbs_EDGE, edge_map);

    std::vector<TopoDS_Vertex> tools;
    tools.reserve(points.size());
    for (const auto& p : points)
    {
        gp_Pnt pt(p.x, p.y, p.z);

        // Skip if coincident with an existing vertex.
        bool at_existing_vertex = false;
        for (int i = 1; i <= vert_map.Extent(); ++i)
        {
            const TopoDS_Vertex& v =
                TopoDS::Vertex(vert_map.FindKey(i));
            gp_Pnt q = BRep_Tool::Pnt(v);
            if (pt.Distance(q) < at_vertex_tol) {
                at_existing_vertex = true;
                break;
            }
        }
        if (at_existing_vertex) continue;

        // Find the nearest edge; if it's within on_edge_tol, accept
        // the point as a valid split site.
        TopoDS_Vertex tool_v = BRepBuilderAPI_MakeVertex(pt);
        bool on_some_edge = false;
        for (int i = 1; i <= edge_map.Extent(); ++i)
        {
            const TopoDS_Edge& e = TopoDS::Edge(edge_map.FindKey(i));
            BRepExtrema_DistShapeShape ext(tool_v, e);
            if (ext.IsDone() && ext.NbSolution() > 0 &&
                ext.Value() < on_edge_tol)
            {
                on_some_edge = true;
                break;
            }
        }
        if (!on_some_edge) continue;

        tools.push_back(tool_v);
    }

    if (tools.empty()) {
        TA_LOG(
            "[SPLIT_BODY] op_id=%u no usable hints (all off-body "
            "or at existing vertices); body unchanged\n", op_id);
        return shape;
    }

    BOPAlgo_Splitter splitter;
    TopTools_ListOfShape args;
    args.Append(shape->GetShape());
    splitter.SetArguments(args);

    TopTools_ListOfShape tools_list;
    for (const auto& tv : tools) {
        tools_list.Append(tv);
    }
    splitter.SetTools(tools_list);

    try {
        splitter.Perform();
    } catch (...) {
        TA_LOG(
            "[SPLIT_BODY] op_id=%u splitter threw; body unchanged\n",
            op_id);
        return shape;
    }
    if (splitter.HasErrors()) {
        TA_LOG(
            "[SPLIT_BODY] op_id=%u splitter reported errors; body "
            "unchanged\n", op_id);
        return shape;
    }

    TopoDS_Shape result = splitter.Shape();
    TA_LOG(
        "[SPLIT_BODY] op_id=%u applied %zu split point(s)\n",
        op_id, tools.size());

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn) {
        opencascade::handle<BRepTools_History> hist = splitter.History();
        pid_map = tn->Update(hist, result, shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(result);
    commit_to_vt(tn, vt, shape, dst, pid_map, "split_body_at_points");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Translate(const std::shared_ptr<TopoShape>& shape, double x, double y, double z,
                                               uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                               const std::shared_ptr<brepdb::VersionTree>& vt)
{
    gp_Trsf trsf;
    trsf.SetTranslation(gp_Vec(x, y, z));
    auto trans = BRepBuilderAPI_Transform(shape->GetShape(), trsf, Standard_True);

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(trans, trans.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(trans.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "translate");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Rotate(const std::shared_ptr<TopoShape>& shape,
                                            const sm::vec3& pos, const sm::vec3& dir, double angle,
                                            uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                            const std::shared_ptr<brepdb::VersionTree>& vt)
{
    gp_Ax1 axis(trans_pnt(pos), trans_dir(dir));

    gp_Trsf trsf;
    trsf.SetRotation(axis, angle);
    auto trans = BRepBuilderAPI_Transform(shape->GetShape(), trsf, Standard_True);

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(trans, trans.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(trans.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "rotate");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Scale(const std::shared_ptr<TopoShape>& shape,
                                           const sm::vec3& center, double factor,
                                           uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                           const std::shared_ptr<brepdb::VersionTree>& vt)
{
    gp_Trsf trsf;
    trsf.SetScale(trans_pnt(center), factor);
    auto trans = BRepBuilderAPI_Transform(shape->GetShape(), trsf, Standard_True);

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(trans, trans.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(trans.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "scale");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Transform(const std::shared_ptr<TopoShape>& shape,
                                                const double* mat4x4,
                                                uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                                const std::shared_ptr<brepdb::VersionTree>& vt)
{
    gp_Trsf trsf;
    // mat4x4 is row-major 4x4, OCCT SetValues takes row,col (1-based)
    trsf.SetValues(
        mat4x4[0],  mat4x4[1],  mat4x4[2],  mat4x4[3],
        mat4x4[4],  mat4x4[5],  mat4x4[6],  mat4x4[7],
        mat4x4[8],  mat4x4[9],  mat4x4[10], mat4x4[11]);

    auto trans = BRepBuilderAPI_Transform(shape->GetShape(), trsf, Standard_True);

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(trans, trans.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(trans.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "transform");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Mirror(const std::shared_ptr<TopoShape>& shape, const sm::vec3& pos, const sm::vec3& dir,
                                            uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                            const std::shared_ptr<brepdb::VersionTree>& vt)
{
    // PartDesign::Mirrored is a plane reflection: (x,y,z) -> reflect
    // across the plane through `pos` with normal `dir`. OCCT's
    // gp_Trsf::SetMirror has two overloads:
    //   SetMirror(gp_Ax1) : symmetry about a LINE (== 180-degree
    //     rotation about that line); flips the two axes perpendicular
    //     to the line, which is the wrong operation here.
    //   SetMirror(gp_Ax2) : reflection across the plane perpendicular
    //     to Ax2's main direction at Ax2's location.
    // We need the second form -- using gp_Ax1 swapped not just X but
    // also Y/Z, so mirrored arms landed at the wrong height when the
    // input shape was offset in Z.
    gp_Ax2 plane(trans_pnt(pos), trans_dir(dir));
    gp_Trsf trsf;
    trsf.SetMirror(plane);

    auto trans = BRepBuilderAPI_Transform(shape->GetShape(), trsf, Standard_True);

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(trans, trans.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(trans.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "mirror");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Draft(const std::shared_ptr<TopoShape>& shape,
                                           const sm::vec3& dir, float angle, float len_max,
                                           uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                           const std::shared_ptr<brepdb::VersionTree>& vt)
{
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape->GetShape(), TopAbs_FACE, faces);

    const auto& face = TopoDS::Face(faces.FindKey(1));

    BRepOffsetAPI_MakeDraft draft(face, trans_vec(dir), angle);
    draft.Perform(len_max);

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(draft, draft.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(draft.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "draft");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::ThickSolid(const std::shared_ptr<TopoShape>& shape, const std::vector<std::shared_ptr<TopoShape>>& faces, float offset,
                                                uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                                const std::shared_ptr<brepdb::VersionTree>& vt)
{
    TopTools_ListOfShape faces_to_rm;
    for (auto& face : faces) {
        faces_to_rm.Append(face->GetShape());
    }

    if (faces_to_rm.IsEmpty())
    {
        // MakeThickSolidByJoin with no closing faces silently
        // degenerates into a plain offset of the entire solid, which
        // looks like an intact shape rather than a shelled one --
        // almost always a resolver miss upstream, not user intent.
        TA_LOG(
                     "[TopoAlgo::ThickSolid] no closing faces "
                     "(upstream face resolution likely missed); "
                     "offset=%g faces_in=%zu\n",
                     (double)offset, faces.size());
        return nullptr;
    }

    // Tol scales with |offset|. The original hard-coded 1e-3 worked
    // for mm-scale callers but breaks for metre-scale inputs (a 1 mm
    // tolerance vs a ~1.4 mm wall makes MakeThickSolidByJoin silently
    // fail and the subsequent Shape() throws). The Confusion floor
    // keeps us safely above OCCT's numerical noise floor.
    const double abs_offset = std::fabs((double)offset);
    const double tol = std::max(Precision::Confusion(),
                                abs_offset * 1.0e-3);

    // MakeThickSolidByJoin is documented to take a Solid. The cax
    // pipeline wraps BOP results in a single-element Compound, and
    // OCCT does technically accept Compound input -- it just walks
    // into a quiet degenerate path where the closing face stays in
    // the outer shell and only a tiny offset patch lands inside
    // (Page_037 produced 16 outer + 3 inner faces instead of the
    // expected ~30 in 1 shell). Strip the Compound when it carries
    // exactly one Solid. Closing faces stay valid: MapShapes
    // descended into the Compound to find them, so their TShape* is
    // shared with the unwrapped Solid.
    TopoDS_Shape solid_in = shape->GetShape();
    if (solid_in.ShapeType() == TopAbs_COMPOUND)
    {
        TopoDS_Shape only;
        int n = 0;
        for (TopExp_Explorer ex(solid_in, TopAbs_SOLID);
             ex.More(); ex.Next())
        {
            only = ex.Current();
            ++n;
            if (n > 1) break;
        }
        if (n == 1) solid_in = only;
    }

    // Pre-pass: UnifySameDomain merges coplanar / cosurface neighbor
    // faces that cax's BOP can leave split where FreeCAD's BOP keeps
    // them whole. Without this MakeThickSolidByJoin's join algorithm
    // bails into the same degenerate output (Page_037: cax 16 faces
    // vs FC 15 -- ByJoin produced 19 faces in 2 shells; after unify
    // it produces ~27 faces in 1 shell). Closing faces have to be
    // remapped through unify's History since UnifySD rewrites TShape
    // pointers and the algorithm matches faces by TShape*, not by
    // geometry.
    opencascade::handle<BRepTools_History> chain_hist;
    {
        ShapeUpgrade_UnifySameDomain unify(solid_in,
                                           /*UnifyEdges*/ Standard_True,
                                           /*UnifyFaces*/ Standard_True,
                                           /*ConcatBSplines*/ Standard_False);
        unify.Build();
        TopoDS_Shape unified = unify.Shape();
        chain_hist = unify.History();

        TopTools_ListOfShape remapped;
        for (TopTools_ListIteratorOfListOfShape it(faces_to_rm);
             it.More(); it.Next())
        {
            const TopoDS_Shape& old_face = it.Value();
            const TopTools_ListOfShape& mods = chain_hist->Modified(old_face);
            if (!mods.IsEmpty()) {
                for (TopTools_ListIteratorOfListOfShape mi(mods);
                     mi.More(); mi.Next())
                {
                    remapped.Append(mi.Value());
                }
            } else if (!chain_hist->IsRemoved(old_face)) {
                remapped.Append(old_face);
            }
        }

        solid_in = unified;
        faces_to_rm.Clear();
        faces_to_rm.Append(remapped);
    }

    if (faces_to_rm.IsEmpty())
    {
        // Every closing face disappeared during unification. The
        // user-intended face is gone, so refuse to fall through into
        // the empty-list degenerate path.
        TA_LOG(
                     "[TopoAlgo::ThickSolid] all closing faces "
                     "vanished during UnifySameDomain pre-pass\n");
        return nullptr;
    }

    // Stash the unified shape; it is what we will pass to
    // BRepTools_History below as the input to the join step.
    const TopoDS_Shape unified_solid = solid_in;

    // Force Intersection=true. OCCT 8.0 dev's default of false
    // skips planar-vs-curved cross-face offset computation; on
    // geometries like Page_037 that drops the algorithm straight
    // into the degenerate output even when closing faces are
    // valid. The remaining args mirror FreeCAD's defaults so we
    // match its expected behavior aside from this one flip.
    BRepOffsetAPI_MakeThickSolid by_join;
    by_join.MakeThickSolidByJoin(solid_in, faces_to_rm, offset, tol,
                                  BRepOffset_Skin,
                                  Standard_True,   // Intersection
                                  Standard_False,  // SelfInter
                                  GeomAbs_Arc);

    auto count_faces = [](BRepOffsetAPI_MakeThickSolid& mk) -> int {
        TopTools_IndexedMapOfShape fm;
        TopExp::MapShapes(mk.Shape(), TopAbs_FACE, fm);
        return fm.Extent();
    };
    TopTools_IndexedMapOfShape in_fm;
    TopExp::MapShapes(unified_solid, TopAbs_FACE, in_fm);
    const int in_faces = in_fm.Extent();
    const int join_faces = by_join.IsDone() ? count_faces(by_join) : -1;

    // ByJoin is degenerate when the result has roughly the same
    // face count as the input -- the signature of "outer shell
    // preserved + tiny offset cap". A proper shell has at least
    // ~1.5x the input face count (closing face removed, every
    // other face offset, plus wall faces). When ByJoin hits that
    // pattern, try BySimple, which uses a simpler internal
    // algorithm that handles cases ByJoin chokes on. BySimple
    // ignores the closing-face list and produces a closed hollow
    // (no opening) -- weaker semantics than ByJoin's intended
    // output but strictly better than a "no offset happened"
    // result that pretends to be a shell.
    const bool degenerate = (join_faces < 0) ||
                            (join_faces > 0 && join_faces < in_faces * 3 / 2);
    BRepOffsetAPI_MakeThickSolid by_simple;
    if (degenerate)
    {
        TA_LOG(
                     "[TopoAlgo::ThickSolid] ByJoin degenerate "
                     "(out=%d, in=%d); retry BySimple\n",
                     join_faces, in_faces);
        by_simple.MakeThickSolidBySimple(solid_in, offset);
    }
    const bool use_simple = degenerate && by_simple.IsDone();
    BRepOffsetAPI_MakeThickSolid& thick_solid =
        use_simple ? by_simple : by_join;

    if (!thick_solid.IsDone())
    {
        TA_LOG(
                     "[TopoAlgo::ThickSolid] not done: offset=%g tol=%g "
                     "faces=%zu\n", (double)offset, tol, faces.size());
        return nullptr;
    }

    // BRepOffsetAPI_MakeThickSolid (OCCT 8.0rc) does not expose
    // History(), so we have to build one via the template ctor
    // that walks every sub-shape of the args and queries
    // Modified/Generated/IsDeleted on the algo. On Page_037 the
    // ByJoin path returns IsDone()==true but leaves dangling
    // BRepOffset_MakeOffset::myAsDes entries, and one of those
    // sub-shape queries AVs inside TKBRep. Wrap the ctor in the
    // same SEH harness Chamfer/Fillet use.
    //
    // When the AV fires, the algorithm's internal state is wedged:
    // an earlier attempt that just nullified the history and kept
    // thick_solid.Shape() shipped a degenerate result downstream
    // (empty / partially-built compound), which broke the document
    // a few frames later when the editor tried to dispatch ves hooks
    // on a half-formed scene node. So if SEH catches the ctor, treat
    // the whole op as failed and return nullptr -- the caller (the
    // "shell" calc op in calc_ops.cpp) already handles a null shape
    // the same way it handles any other op failure.
    //
    // A BySimple recovery attempt was tried and reverted: each SEH
    // catch leaks OCCT internal allocations (C++ unwind is bypassed,
    // see the comment near seh_call_void), and stacking two of them
    // -- one for ByJoin's history, one for BySimple's -- corrupted
    // enough state that ves runtime constants got trampled, crashing
    // the script VM with a null ObjString. One SEH catch per op is
    // the budget; spend it on the failure path and exit, don't try
    // to recover.
    opencascade::handle<BRepTools_History> join_hist;
    {
        TopTools_ListOfShape join_args;
        join_args.Append(unified_solid);
        struct Ctx {
            TopTools_ListOfShape* args;
            BRepOffsetAPI_MakeThickSolid* algo;
            opencascade::handle<BRepTools_History>* out;
            bool ok;
        };
        Ctx ctx{&join_args, &thick_solid, &join_hist, false};
        auto runner = +[](void* p) -> void {
            auto* c = static_cast<Ctx*>(p);
            try {
                *c->out = new BRepTools_History(*c->args, *c->algo);
                c->ok = true;
            } catch (...) {
                c->ok = false;
            }
        };
#ifdef _MSC_VER
        int seh = seh_call_void(runner, &ctx);
#else
        runner(&ctx);
        int seh = 0;
#endif
        if (seh < 0 || !ctx.ok) {
            TA_LOG(
                         "[TopoAlgo::ThickSolid] BRepTools_History "
                         "construction faulted (seh=%d ok=%d); "
                         "treating op as failed\n",
                         seh, (int)ctx.ok);
            return nullptr;
        }
    }

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn) {
        // Compose the unify history (orig -> unified) with the join
        // step's builder history (unified -> final) so TopoNaming
        // records lineage in one update rather than treating the
        // shelled faces as orphans. The merged chain_hist effectively
        // maps orig faces directly to final faces.
        chain_hist->Merge(join_hist);

        brepkit::TopoShape new_ts(thick_solid.Shape());
        pid_map = tn->Update(chain_hist, new_ts, *shape, op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(thick_solid.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "thick_solid");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::ThruSections(const std::vector<std::shared_ptr<TopoShape>>& wires,
                                                  bool is_solid,
                                                  uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                                  const std::shared_ptr<brepdb::VersionTree>& vt)
{
    // OCCT's BRepOffsetAPI_ThruSections never reports failure by return
    // value: AddWire / Build can leave the algorithm "not done", and the
    // first Shape() access then raises StdFail_NotDone from deep inside
    // TKTopAlgo. That throw used to escape this function and unwind all
    // the way out through the loft op lambda -> Evaluator -> Replayer ->
    // the ves VM with nothing catching it, crashing editor.exe (golden
    // Page_094: incompatible / degenerate section wires). Mirror the
    // ThickSolid contract above -- swallow the OCCT failure here and
    // return nullptr; the "loft" calc op already maps a null shape onto
    // an ordinary op failure (calc_ops.cpp: `if (!shp) return {};`).
    try {
        BRepOffsetAPI_ThruSections thru_sections(is_solid ? Standard_True : Standard_False);
        for (auto& wire : wires) {
            thru_sections.AddWire(wire->ToWire());
        }
        thru_sections.Build();
        if (!thru_sections.IsDone() || thru_sections.Shape().IsNull()) {
            TA_LOG(
                         "[TopoAlgo::ThruSections] loft did not build "
                         "(%zu section wires, is_solid=%d); treating op "
                         "as failed\n",
                         wires.size(), (int)is_solid);
            return nullptr;
        }
        const TopoDS_Shape result = thru_sections.Shape();

        brepgraph::TopoNaming::PidMap pid_map;
        if (tn)
        {
            auto old_shp = ShapeBuilder::MakeCompound(wires);
            pid_map = tn->Update(thru_sections, result, old_shp->GetShape(), op_id);
        }
        auto dst = std::make_shared<brepkit::TopoShape>(result);
        auto dummy_src = std::make_shared<brepkit::TopoShape>();
        commit_to_vt(tn, vt, dummy_src, dst, pid_map, "thru_sections");
        return dst;
    }
    catch (const Standard_Failure& e) {
        TA_LOG(
                     "[TopoAlgo::ThruSections] OCCT failure: %s; treating "
                     "op as failed\n",
                     e.GetMessageString());
        return nullptr;
    }
}

std::shared_ptr<TopoShape> TopoAlgo::OffsetShape(const std::shared_ptr<TopoShape>& shape, float offset, bool is_solid,
                                                 uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                                 const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepOffset_MakeSimpleOffset builder;
    builder.Initialize(shape->GetShape(), offset);
    builder.SetBuildSolidFlag(is_solid);
    builder.Perform();

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(builder, shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(builder.GetResultShape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "offset_shape");
    return dst;
}

}
