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
#include <cstring>
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
#include <TopoDS_Iterator.hxx>
#include <BRep_Builder.hxx>
#include <TopExp_Explorer.hxx>
#include <BOPAlgo_BuilderSolid.hxx>
#include <BOPAlgo_GlueEnum.hxx>
#include <BOPAlgo_MakerVolume.hxx>
#include <BOPAlgo_Splitter.hxx>
#include <Geom_Plane.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
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
#include <BRepAdaptor_Surface.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepTools.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <BRepTools_History.hxx>
#include <Standard_Failure.hxx>
#include <gp_Ax2.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <GeomLProp_SLProps.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <BRep_Tool.hxx>
#include <Precision.hxx>
#include <Message_ProgressIndicator.hxx>

#ifdef _MSC_VER
#  include <excpt.h>
#  include <float.h>
#endif

// OSD::SetSignal(Standard_True) unmasks hardware floating-point traps
// (zero-divide / invalid / overflow) PROCESS-WIDE so that ChFi3d's
// internal FP faults surface as catchable exceptions instead of
// silently corrupting the fillet. But the unmask outlives the op: the
// next sketch solve anywhere in the process (planegcs evaluates
// gradients that legitimately divide by zero and expects a quiet NaN)
// then traps, and the trap surfaces as a bogus
// "device or resource busy" system_error from deep inside the
// exception-dispatch machinery. Scope the unmask to the dressup op:
// save the exception-mask bits on entry, restore them on any exit.
#ifdef _MSC_VER
struct FpTrapScope
{
    unsigned int saved = 0;
    FpTrapScope()  { _controlfp_s(&saved, 0, 0); }
    ~FpTrapScope() { unsigned int tmp; _controlfp_s(&tmp, saved, _MCW_EM); }
};
#else
struct FpTrapScope {};
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
// Wall-clock cancellation for BOP algorithms. OCCT polls UserBreak()
// between pave-filler stages / intersection pairs; once the deadline
// passes the algorithm stops with BOPAlgo_AlertUserBreak (surfaces as
// HasErrors()), and the caller falls back exactly as for any other
// failed BOP. Motivation: BOPAlgo_MakerVolume on a face soup whose
// boundaries come from trimmed B-splines can degenerate into an
// unbounded intersection walk -- 02-ear 组合1 ran 13+ minutes at
// ~110MB/s straight to 90+GB and froze the machine three times on
// 2026-06-12 before this guard existed.
class DeadlineIndicator : public Message_ProgressIndicator
{
public:
    explicit DeadlineIndicator(double seconds)
        : m_deadline(std::chrono::steady_clock::now()
                     + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                           std::chrono::duration<double>(seconds))) {}
    bool Expired() const
    { return std::chrono::steady_clock::now() > m_deadline; }
protected:
    Standard_Boolean UserBreak() override { return Expired(); }
    void Show(const Message_ProgressScope&, const Standard_Boolean) override {}
private:
    std::chrono::steady_clock::time_point m_deadline;
};

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

// Run the Fuse / Cut / Common BOPs with OCCT's internal parallelism
// (intersection + DS-filler stages fan out via OSD_Parallel). The win
// concentrates exactly where this codebase hurts: one big body against a
// many-solid tool compound -- R2900_100's pattern-ring fuse is 27 s
// single-threaded and dominated by pairwise face intersections that
// parallelize cleanly. The result topology is identical (RunParallel
// only schedules independent sub-tasks; verified against the geo
// goldens). TopoAlgo_Ext::FuseInstancesAndUnify already runs parallel;
// this brings the primary booleans in line. BREPKIT_BOP_PARALLEL=0
// reverts to serial for A/B.
static bool bop_parallel()
{
    static const bool on = [] {
        const char* e = std::getenv("BREPKIT_BOP_PARALLEL");
        return !(e && e[0] == '0');
    }();
    return on;
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
    // Guard: a degenerate operand (e.g. a path-less sweep that collapses
    // in-plane) can carry a non-finite or astronomically large bbox
    // coordinate. Unclamped, diag*5e-6 became fuzzy=1e95 (gate=1e96) and
    // BRepAlgoAPI_Fuse raised BOPAlgo_AlertBuilderFailed, aborting the
    // whole replay. The model lives at mm-to-metre scale; any diagonal
    // past ~1 km is broken geometry, not a real tolerance -- fall back to
    // the safe floor so one bad operand can't poison the boolean.
    if (!std::isfinite(diag) || diag > 1.0e3) return 1e-6;
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
    // the env var below. FpTrapScope re-masks the FP traps when this
    // function exits -- leaving them unmasked poisons later sketch
    // solves (see the struct comment).
    FpTrapScope fp_trap_scope;
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
        char buf[320];
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
            // Edge + worst vertex tolerance: ChFi3d refuses a blend whose
            // radius is comparable to the local tolerance (a fuse-seam
            // edge bumped to ~1e-4 m kills a 2e-4 m fillet), and that
            // failure mode is invisible without these numbers.
            double e_tol = BRep_Tool::Tolerance(e);
            double v_tol = 0.0;
            for (TopExp_Explorer vx(e, TopAbs_VERTEX); vx.More(); vx.Next()) {
                v_tol = std::max(
                    v_tol, BRep_Tool::Tolerance(TopoDS::Vertex(vx.Current())));
            }
            std::snprintf(buf, sizeof(buf),
                "mid=(%.4f,%.4f,%.4f) start=(%.4f,%.4f,%.4f) "
                "end=(%.4f,%.4f,%.4f) len=%.4f etol=%.3g vtol=%.3g",
                p_mid.X(), p_mid.Y(), p_mid.Z(),
                p_start.X(), p_start.Y(), p_start.Z(),
                p_end.X(), p_end.Y(), p_end.Z(),
                props.Mass(), e_tol, v_tol);
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
        // ---- Scaled-space rescue ----
        //
        // Every attempt so far ran at the document's native scale.
        // ChFi3d works against ABSOLUTE Precision::Confusion (1e-7),
        // so at metre scale a 0.2 mm fillet chasing 0.5 mm corner
        // edges sits a bare 3 decades above the precision floor and
        // the corner solves collapse -- while the IDENTICAL body at
        // mm scale fillets cleanly in one batch call (R2900_100
        // Fillet4: 16 edges, r=2e-4 m, every path above fails;
        // unit_scale=1 import, same op, batch Rational succeeds in
        // 150 ms). Re-run the batch fillet in a scaled-up copy of
        // the body and scale the result back. TopoNaming bookkeeping
        // is skipped, exactly like the refined-input and per-edge
        // paths above.
        const double target_extent = 300.0;   // land in mm-like numbers
        double k = 1.0;
        if (input_extent > 0.0) {
            k = std::pow(10.0, std::round(std::log10(target_extent /
                                                     input_extent)));
        }
        if (k >= 10.0)
        {
            try
            {
                gp_Trsf t_up;
                t_up.SetScale(gp_Pnt(0.0, 0.0, 0.0), k);
                BRepBuilderAPI_Transform up(shape->GetShape(), t_up,
                                            /*Copy=*/Standard_True);

                std::vector<TopoDS_Edge> scaled_edges;
                scaled_edges.reserve(leaf_edges.size());
                for (const auto& e : leaf_edges) {
                    const TopoDS_Shape& se = up.ModifiedShape(e);
                    if (!se.IsNull() &&
                        se.ShapeType() == TopAbs_EDGE) {
                        scaled_edges.push_back(TopoDS::Edge(se));
                    }
                }

                auto try_scaled = [&](ChFi3d_FilletShape mode,
                                      TopoDS_Shape& out_shape) -> bool {
                    BRepFilletAPI_MakeFillet f(up.Shape());
                    f.SetFilletShape(mode);
                    for (const auto& e : scaled_edges) {
                        f.Add(radius * k, e);
                    }
                    int r = -1;
                    try { r = seh_safe_build(&f); }
                    catch (...) { r = -1; }
                    if (r != 1) return false;
                    TopoDS_Shape fs = UnwrapSingleSolid(f.Shape());
                    // Same self-intersecting-blend guard as the batch
                    // path, in scaled units.
                    Bnd_Box b;
                    BRepBndLib::Add(fs, b);
                    if (b.IsVoid()) return false;
                    double xmin, ymin, zmin, xmax, ymax, zmax;
                    b.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                    double ext = std::max({ xmax - xmin, ymax - ymin,
                                            zmax - zmin });
                    if (ext > (input_extent * k + 2.0 * radius * k) * 10.0) {
                        return false;
                    }
                    out_shape = fs;
                    return true;
                };

                TopoDS_Shape scaled_result;
                if (!scaled_edges.empty() &&
                    (try_scaled(ChFi3d_Rational, scaled_result) ||
                     try_scaled(ChFi3d_QuasiAngular, scaled_result)))
                {
                    gp_Trsf t_down;
                    t_down.SetScale(gp_Pnt(0.0, 0.0, 0.0), 1.0 / k);
                    BRepBuilderAPI_Transform down(scaled_result, t_down,
                                                  /*Copy=*/Standard_True);
                    TopoDS_Shape back = UnwrapSingleSolid(down.Shape());
                    if (!back.IsNull())
                    {
                        TA_LOG(
                            "[FILLET] op_id=%u scaled-space rescue OK "
                            "(k=%g, %zu/%zu edges)\n",
                            op_id, k, scaled_edges.size(),
                            leaf_edges.size());
                        auto dst = std::make_shared<brepkit::TopoShape>(back);
                        commit_to_vt(tn, vt, shape, dst, {}, "fillet");
                        return dst;
                    }
                }
            }
            catch (...) {
                // fall through to the WARNING below
            }
        }

        // Unconditional (not TA_LOG): a fillet that was asked for N edges
        // and changed NOTHING is a conversion defect, not a debug detail.
        // R2900_100's Fillet4 no-opped through every path -- batch,
        // refined-input, per-edge x2 modes -- and the only symptom was a
        // byte-identical body two metrics later.
        std::fprintf(stderr,
            "[FILLET] op_id=%u WARNING: all paths failed, body UNCHANGED "
            "(%zu edges requested, radius=%g, scaled retry k=%g)\n",
            op_id, leaf_edges.size(), radius, k);
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
    FpTrapScope fp_trap_scope;
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
        // Unconditional (not TA_LOG), parity with the FILLET warning: a
        // chamfer that was asked for N edges and changed NOTHING is a
        // conversion defect, not a debug detail. R2900's Chamfer4/6/10/11
        // no-opped through batch + per-edge and the only trace was the
        // Replayer's IsSame catch-all far downstream.
        std::fprintf(stderr,
            "[CHAMFER] op_id=%u WARNING: all paths failed, body UNCHANGED "
            "(%zu edges requested, dist=%g)\n",
            op_id, leaf_edges.size(), dist);
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

std::shared_ptr<TopoShape> TopoAlgo::DPrism(const std::shared_ptr<TopoShape>& face, double x, double y, double z,
                                            double angle,
                                            uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                            const std::shared_ptr<brepdb::VersionTree>& vt)
{
    const double height = std::sqrt(x * x + y * y + z * z);
    if (std::fabs(angle) < 1e-12 || height < 1e-12) {
        return Prism(face, x, y, z, op_id, tn, vt);
    }

    TopoDS_Shape fs = face->GetShape();
    TopoDS_Face  spine;
    if (fs.ShapeType() == TopAbs_FACE) {
        spine = TopoDS::Face(fs);
    } else {
        TopExp_Explorer ex(fs, TopAbs_FACE);
        if (ex.More()) {
            spine = TopoDS::Face(ex.Current());
        }
    }

    bool   planar = false;
    gp_Dir n;
    if (!spine.IsNull()) {
        BRepAdaptor_Surface surf(spine);
        if (surf.GetType() == GeomAbs_Plane) {
            n = surf.Plane().Axis().Direction();
            if (spine.Orientation() == TopAbs_REVERSED) {
                n.Reverse();
            }
            planar = true;
        }
    }
    if (!planar) {
        std::fprintf(stderr,
            "[DPRISM] op_id=%u WARNING: non-planar profile, draft %.4g rad "
            "dropped; falling back to straight prism\n", op_id, angle);
        return Prism(face, x, y, z, op_id, tn, vt);
    }

    // Single-loop profiles only for now: BRepOffsetAPI_MakeOffset on a
    // multi-wire face offsets the holes too, but pairing each offset
    // loop back to its source for the loft is bookkeeping this case
    // does not need yet (R2900_100's drafted plate is one rectangle).
    int nwires = 0;
    for (TopExp_Explorer wx(spine, TopAbs_WIRE); wx.More(); wx.Next()) {
        ++nwires;
    }
    if (nwires != 1) {
        std::fprintf(stderr,
            "[DPRISM] op_id=%u WARNING: profile has %d wires, draft "
            "%.4g rad dropped; falling back to straight prism\n",
            op_id, nwires, angle);
        return Prism(face, x, y, z, op_id, tn, vt);
    }

    // Build the tapered solid as a ruled loft between the profile wire
    // and a copy offset IN PLANE by -h*tan(angle) (shrink: ZW3D's
    // positive draft) and translated to the far end. Exact for line /
    // arc profiles; GeomAbs_Intersection keeps sharp corners sharp
    // (Arc would round the offset rectangle's corners). This stays
    // inside TKOffset, which every cax consumer already links --
    // LocOpe_DPrism would have pulled TKFeat into the whole link tree.
    TopoDS_Shape result;
    try {
        const TopoDS_Wire w0 = BRepTools::OuterWire(spine);

        BRepOffsetAPI_MakeOffset off(spine, GeomAbs_Intersection);
        off.Perform(-height * std::tan(angle));
        TopoDS_Wire w1;
        for (TopExp_Explorer ex(off.Shape(), TopAbs_WIRE); ex.More(); ex.Next()) {
            w1 = TopoDS::Wire(ex.Current());
            break;
        }
        if (!w1.IsNull()) {
            gp_Trsf t;
            t.SetTranslation(gp_Vec(x, y, z));
            w1 = TopoDS::Wire(
                BRepBuilderAPI_Transform(w1, t, /*Copy=*/true).Shape());

            BRepOffsetAPI_ThruSections loft(/*isSolid=*/Standard_True,
                                            /*ruled=*/Standard_True);
            loft.AddWire(w0);
            loft.AddWire(w1);
            loft.Build();
            if (loft.IsDone()) {
                result = loft.Shape();
            }
        }
    } catch (...) {
    }

    if (result.IsNull()) {
        std::fprintf(stderr,
            "[DPRISM] op_id=%u WARNING: offset/loft failed (h=%.4g "
            "angle=%.4g rad); falling back to straight prism\n",
            op_id, height, angle);
        return Prism(face, x, y, z, op_id, tn, vt);
    }

    // No incremental TopoNaming history on the drafted path (LocOpe
    // exposes its generated-shape maps differently from BRepBuilderAPI
    // and chaining them is not worth it yet); commit the geometry and
    // let downstream refs resolve geometrically.
    auto dst = std::make_shared<brepkit::TopoShape>(result);
    commit_to_vt(tn, vt, face, dst, {}, "dprism");
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

namespace
{

// Signed distance of `p` to the tool's ORIENTED surface: dot(p - q, n(q))
// with q the closest point on a tool FACE and n the face normal including
// the face's orientation flag. Returns 0 when the closest support is an
// edge / vertex (the cut seam -- ambiguous) or extrema fails; callers
// treat 0 as "no vote".
double SignedDistToTool(const TopoDS_Shape& tool, const gp_Pnt& p)
{
    TopoDS_Vertex v = BRepBuilderAPI_MakeVertex(p);
    BRepExtrema_DistShapeShape ext(v, tool);
    if (!ext.IsDone() || ext.NbSolution() < 1) {
        return 0.0;
    }
    for (int i = 1; i <= ext.NbSolution(); ++i)
    {
        if (ext.SupportTypeShape2(i) != BRepExtrema_IsInFace) {
            continue;
        }
        const TopoDS_Face f = TopoDS::Face(ext.SupportOnShape2(i));
        double u = 0.0, w = 0.0;
        ext.ParOnFaceS2(i, u, w);
        BRepAdaptor_Surface surf(f);
        gp_Pnt sp;
        gp_Vec d1u, d1v;
        surf.D1(u, w, sp, d1u, d1v);
        gp_Vec n = d1u.Crossed(d1v);
        if (n.Magnitude() < 1e-12) {
            continue;
        }
        if (f.Orientation() == TopAbs_REVERSED) {
            n.Reverse();
        }
        return gp_Vec(sp, p).Dot(n) / n.Magnitude();
    }
    // Extended-surface fallback, PLANAR tools only: the nearest tool
    // point lies on an edge / vertex -- p projects beyond the bounded
    // face, the tangent / no-cut case. ZW3D trims by SPACE PARTITION
    // (02-ear 修剪2: the lens pair is exactly tangent to the L-band's
    // wall plane, zero transversal intersection, yet truth discards the
    // whole face on the flap plane's far side). A plane's extension is
    // exact; B-spline extrapolation is not -- those keep the zero vote.
    for (int i = 1; i <= ext.NbSolution(); ++i)
    {
        const TopoDS_Shape sup = ext.SupportOnShape2(i);
        if (sup.ShapeType() != TopAbs_EDGE &&
            sup.ShapeType() != TopAbs_VERTEX) {
            continue;
        }
        for (TopExp_Explorer fx(tool, TopAbs_FACE); fx.More(); fx.Next())
        {
            bool contains = false;
            for (TopExp_Explorer sx(fx.Current(), sup.ShapeType());
                 sx.More() && !contains; sx.Next())
            {
                if (sx.Current().IsSame(sup)) { contains = true; }
            }
            if (!contains) { continue; }
            const TopoDS_Face f = TopoDS::Face(fx.Current());
            Handle(Geom_Surface) hs = BRep_Tool::Surface(f);
            if (hs.IsNull()) { continue; }
            // No planarity gate: ZW3D's STEP encodes even flat faces as
            // B-splines, so a Geom_Plane downcast never fires. The
            // projection clamps to the surface DOMAIN (no extrapolation)
            // and the normal at that nearest domain point classifies the
            // side just as the bounded-face branch does.
            GeomAPI_ProjectPointOnSurf proj(p, hs);
            if (!proj.IsDone() || proj.NbPoints() < 1) { continue; }
            Standard_Real u = 0.0, w = 0.0;
            proj.LowerDistanceParameters(u, w);
            gp_Pnt sp;
            gp_Vec d1u, d1v;
            hs->D1(u, w, sp, d1u, d1v);
            gp_Vec n = d1u.Crossed(d1v);
            if (n.Magnitude() < 1e-12) { continue; }
            if (f.Orientation() == TopAbs_REVERSED) {
                n.Reverse();
            }
            return gp_Vec(sp, p).Dot(n) / n.Magnitude();
        }
    }
    return 0.0;
}

// Side vote of a whole fragment: area-weighted signed distance over its
// face centroids. Weighting makes samples near the cut seam (tiny |d|)
// irrelevant and L-shaped faces (centroid off the trim region but on the
// correct side) harmless.
double FragmentSideVote(const TopoDS_Shape& tool, const TopoDS_Shape& frag)
{
    double acc = 0.0;
    for (TopExp_Explorer fx(frag, TopAbs_FACE); fx.More(); fx.Next())
    {
        GProp_GProps g;
        BRepGProp::SurfaceProperties(fx.Current(), g);
        const gp_Pnt c = g.CentreOfMass();
        acc += g.Mass() * SignedDistToTool(tool, c);
    }
    return acc;
}

} // namespace

namespace
{

// One directional trim pass: split `arg` by `cutter`'s faces and keep
// the pieces whose side sign (vs the cutter's oriented surface) matches
// `ref`. Granularity matters: the splitter rebuilds a SHELL argument as
// ONE shell whose faces are now split along the cut curve -- top-level
// iteration sees a single "fragment" and the trim silently no-ops
// (02-ear 修剪1: the z=-0.6 plane cut the skin, but the skin stayed one
// shell). So: SOLIDs classify whole, anything sheet-like decomposes to
// FACES that classify individually (kept ones are sewn back so the
// surviving skin stays ONE body); free wires / edges ride along uncut.
struct TrimSideResult
{
    bool ok      = false;   // splitter ran and something was kept
    int  frags   = 0;
    int  kept    = 0;
    int  dropped = 0;
    TopoDS_Shape shape;     // kept content (single child unwrapped)
    opencascade::handle<BRepTools_History> hist;
};

// keep_zero_votes: what to do with a fragment the classifier can't side
// (no face support -- e.g. its nearest cutter point is an edge). For the
// BASE side losing geometry silently is the worse error -> keep. For the
// TOOL side ZW3D's default is consumption; keeping unclassifiable tool
// junk inflates the model with material ZW3D removed (02-ear 修剪3: an
// uncut 342 mm^2 horizontal sheet piece rode in on a zero vote) -> drop.
// zero_adj: optional adjacency referee for zero-vote fragments. A
// mutual-trim CAP (the tool sliver that seals the kept base's cut
// opening) lies ON the partition boundary -- distance-sign voting is
// structurally zero there. ZW3D keeps exactly the tool pieces bounding
// the kept region: with zero_adj set, a zero-vote fragment is kept iff
// it RESTS on those edges (02-ear 修剪3: the 46.4+63.4mm^2 z=8.2 caps
// that later seal the dome for the final solid).
TrimSideResult TrimOneSide(const TopoDS_Shape& arg, const TopoDS_Shape& cutter,
                           double ref, uint32_t op_id, const char* side,
                           bool keep_zero_votes,
                           const TopoDS_Shape* zero_adj = nullptr)
{
    TrimSideResult r;

    TopTools_ListOfShape args, cuts;
    args.Append(arg);
    cuts.Append(cutter);
    BRepAlgoAPI_Splitter algo;
    algo.SetArguments(args);
    algo.SetTools(cuts);
    // Same rationale as Cut/Fuse: plane-on-plane tangencies otherwise
    // make the splitter miss the intersection entirely.
    algo.SetFuzzyValue(1e-6);
    algo.Build();
    if (!algo.IsDone())
    {
        std::fprintf(stderr,
            "[TRIM] op_id=%u WARNING: splitter failed (%s side)\n",
            op_id, side);
        std::fflush(stderr);
        return r;
    }
    r.hist = algo.History();

    const TopoDS_Shape result = algo.Shape();
    std::vector<TopoDS_Shape> top;
    if (result.ShapeType() == TopAbs_COMPOUND)
    {
        for (TopoDS_Iterator it(result); it.More(); it.Next()) {
            top.push_back(it.Value());
        }
    }
    else
    {
        top.push_back(result);
    }
    std::vector<TopoDS_Shape> frags;       // solids (whole) + faces
    std::vector<TopoDS_Shape> riders;      // non-face leftovers, kept as-is
    for (const TopoDS_Shape& t : top)
    {
        if (t.ShapeType() == TopAbs_SOLID)
        {
            frags.push_back(t);
            continue;
        }
        bool any_face = false;
        for (TopExp_Explorer fx(t, TopAbs_FACE); fx.More(); fx.Next())
        {
            frags.push_back(fx.Current());
            any_face = true;
        }
        if (!any_face) {
            riders.push_back(t);
        }
    }
    // Forensics when the splitter produced no cut: print both bboxes and
    // the min distance so "cutter never reaches arg" vs "tangency the
    // fuzzy didn't bridge" vs "wrong base wired" are distinguishable
    // from the log alone.
    if (frags.size() <= 1)
    {
        Bnd_Box ab, cb;
        BRepBndLib::Add(arg, ab);
        BRepBndLib::Add(cutter, cb);
        double ax0=0, ay0=0, az0=0, ax1=0, ay1=0, az1=0;
        double cx0=0, cy0=0, cz0=0, cx1=0, cy1=0, cz1=0;
        if (!ab.IsVoid()) ab.Get(ax0, ay0, az0, ax1, ay1, az1);
        if (!cb.IsVoid()) cb.Get(cx0, cy0, cz0, cx1, cy1, cz1);
        BRepExtrema_DistShapeShape dist(arg, cutter);
        const double d = (dist.IsDone() && dist.NbSolution() > 0)
                             ? dist.Value() : -1.0;
        std::fprintf(stderr,
            "[trim] op_id=%u NOSPLIT(%s) arg_bbox=(%.4g,%.4g,%.4g)(%.4g,"
            "%.4g,%.4g) cutter_bbox=(%.4g,%.4g,%.4g)(%.4g,%.4g,%.4g) "
            "min_dist=%.4g\n",
            op_id, side, ax0, ay0, az0, ax1, ay1, az1,
            cx0, cy0, cz0, cx1, cy1, cz1, d);
        std::fflush(stderr);
    }

    BRep_Builder builder;
    TopoDS_Compound kept;
    builder.MakeCompound(kept);
    std::vector<TopoDS_Shape> kept_faces;
    for (const TopoDS_Shape& f : frags)
    {
        const double vote = FragmentSideVote(cutter, f);
        bool keep;
        if (vote != 0.0)
        {
            keep = (vote > 0.0) == (ref > 0.0);
        }
        else if (zero_adj != nullptr && !zero_adj->IsNull())
        {
            BRepExtrema_DistShapeShape da(f, *zero_adj);
            keep = da.IsDone() && da.NbSolution() > 0 &&
                   da.Value() <= 5e-4;
        }
        else
        {
            keep = keep_zero_votes;
        }
        {
            GProp_GProps fg;
            BRepGProp::SurfaceProperties(f, fg);
            Bnd_Box fb;
            BRepBndLib::Add(f, fb);
            double fx0=0, fy0=0, fz0=0, fx1=0, fy1=0, fz1=0;
            if (!fb.IsVoid()) fb.Get(fx0, fy0, fz0, fx1, fy1, fz1);
            std::fprintf(stderr,
                "[trim]   frag side=%s area=%.4g vote=%.3g keep=%d "
                "bbox=(%.4g,%.4g,%.4g)(%.4g,%.4g,%.4g)\n",
                side, fg.Mass() * 1e6, vote, keep ? 1 : 0,
                fx0, fy0, fz0, fx1, fy1, fz1);
        }
        if (!keep) { ++r.dropped; continue; }
        ++r.kept;
        if (f.ShapeType() == TopAbs_FACE) {
            kept_faces.push_back(f);
        } else {
            builder.Add(kept, f);
        }
    }
    r.frags = (int)frags.size();
    if (r.kept == 0)
    {
        std::fprintf(stderr, "[trim] op_id=%u side=%s frags=%d kept=0\n",
                     op_id, side, r.frags);
        std::fflush(stderr);
        return r;
    }
    if (!kept_faces.empty())
    {
        if (kept_faces.size() == 1)
        {
            builder.Add(kept, kept_faces[0]);
        }
        else
        {
            BRepBuilderAPI_Sewing sew(1e-6);
            for (const TopoDS_Shape& f : kept_faces) {
                sew.Add(f);
            }
            sew.Perform();
            const TopoDS_Shape sewn = sew.SewedShape();
            if (!sewn.IsNull()) {
                builder.Add(kept, sewn);
            } else {
                for (const TopoDS_Shape& f : kept_faces) {
                    builder.Add(kept, f);
                }
            }
        }
    }
    for (const TopoDS_Shape& rd : riders) {
        builder.Add(kept, rd);
    }
    r.shape = kept;
    {
        TopoDS_Iterator solo(kept);
        if (solo.More())
        {
            const TopoDS_Shape first = solo.Value();
            solo.Next();
            if (!solo.More()) {
                r.shape = first;   // single child -> unwrap
            }
        }
    }
    r.ok = true;
    std::fprintf(stderr, "[trim] op_id=%u side=%s frags=%d kept=%d dropped=%d\n",
                 op_id, side, r.frags, r.kept, r.dropped);
    return r;
}

} // namespace

std::shared_ptr<TopoShape> TopoAlgo::TrimByTool(const std::shared_ptr<TopoShape>& base, const std::shared_ptr<TopoShape>& tool,
                                                const sm::vec3& keep_pt, const sm::vec3& keep_dir, bool mutual,
                                                uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                                const std::shared_ptr<brepdb::VersionTree>& vt)
{
    // Keep-side witness: keep_pt sits ON the tool, so step a little way
    // along keep_dir before classifying. Scale the step to the model and
    // grow it if the sample still reads as "on the surface".
    Bnd_Box bb;
    BRepBndLib::Add(base->GetShape(), bb);
    double diag = 1.0;
    if (!bb.IsVoid())
    {
        double x0, y0, z0, x1, y1, z1;
        bb.Get(x0, y0, z0, x1, y1, z1);
        diag = std::sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0) + (z1-z0)*(z1-z0));
    }
    gp_Vec dir(keep_dir.x, keep_dir.y, keep_dir.z);
    if (dir.Magnitude() < 1e-12)
    {
        std::fprintf(stderr,
            "[TRIM] op_id=%u WARNING: null keep_dir, body UNCHANGED\n", op_id);
        std::fflush(stderr);
        return base;
    }
    dir.Normalize();
    auto witness = [&](const TopoDS_Shape& surface) -> double {
        double ref = 0.0;
        double eps = std::max(1e-7, 1e-4 * diag);
        for (int attempt = 0; attempt < 5 && std::fabs(ref) < 1e-12; ++attempt)
        {
            const gp_Pnt x_ref(keep_pt.x + eps * dir.X(),
                               keep_pt.y + eps * dir.Y(),
                               keep_pt.z + eps * dir.Z());
            ref = SignedDistToTool(surface, x_ref);
            eps *= 10.0;
        }
        return ref;
    };
    const double ref = witness(tool->GetShape());
    if (std::fabs(ref) < 1e-12)
    {
        std::fprintf(stderr,
            "[TRIM] op_id=%u WARNING: keep-side witness unresolvable, "
            "body UNCHANGED\n", op_id);
        std::fflush(stderr);
        return base;
    }

    TrimSideResult bs = TrimOneSide(base->GetShape(), tool->GetShape(),
                                    ref, op_id, "base",
                                    /*keep_zero_votes=*/true);
    if (!bs.ok)
    {
        std::fprintf(stderr,
            "[TRIM] op_id=%u WARNING: base side kept nothing, body "
            "UNCHANGED\n", op_id);
        std::fflush(stderr);
        return base;
    }

    // Mutual (ZW3D fld8 互剪): the tool is trimmed by the BASE too, and
    // its witnessed-side remnant survives as a separate body (02-ear
    // 修剪3: 拉伸8's sheet leaves a ~110 mm^2 sliver -- the _state area
    // only balances with it kept). Same witness ray, classified against
    // the BASE surface this time. Unresolvable witness or empty keep
    // degrades to the plain consume-the-tool trim.
    TopoDS_Shape out_shape = bs.shape;
    if (mutual)
    {
        // The surviving tool remnant lies on the side of the base the
        // witness ray LEAVES, i.e. the reverse of the witnessed base
        // side (02-ear 修剪3: same-side kept ~470 mm^2 of 拉伸8 where
        // ZW3D keeps a 110 mm^2 sliver tucked under the skin).
        const double ref2 = -witness(base->GetShape());
        if (std::fabs(ref2) >= 1e-12)
        {
            // Free boundary of the KEPT base side: the cap slivers that
            // must survive rest exactly on these edges.
            TopoDS_Compound kept_free;
            {
                BRep_Builder kb;
                kb.MakeCompound(kept_free);
                TopTools_IndexedDataMapOfShapeListOfShape ke2f;
                TopExp::MapShapesAndAncestors(bs.shape, TopAbs_EDGE,
                                              TopAbs_FACE, ke2f);
                for (int i = 1; i <= ke2f.Extent(); ++i)
                {
                    const TopoDS_Edge& e = TopoDS::Edge(ke2f.FindKey(i));
                    if (BRep_Tool::Degenerated(e)) { continue; }
                    if (ke2f.FindFromIndex(i).Extent() < 2) {
                        kb.Add(kept_free, e);
                    }
                }
            }
            const TopoDS_Shape kf = kept_free;
            TrimSideResult ts = TrimOneSide(tool->GetShape(),
                                            base->GetShape(),
                                            ref2, op_id, "tool",
                                            /*keep_zero_votes=*/false,
                                            &kf);
            if (ts.ok)
            {
                BRep_Builder builder;
                TopoDS_Compound both;
                builder.MakeCompound(both);
                builder.Add(both, bs.shape);
                builder.Add(both, ts.shape);
                out_shape = both;
            }
        }
        else
        {
            std::fprintf(stderr,
                "[trim] op_id=%u mutual witness unresolvable vs base; "
                "tool remnant dropped\n", op_id);
            std::fflush(stderr);
        }
    }

    // Serialize tool BEFORE tn->Update() unbinds its shapes (Split's
    // pattern); naming maps to the kept subset of the base side only.
    brepdb::BRepWorld tool_world;
    if (tn && vt) { tool_world = serialize_world(tn, tool->GetShape()); }
    brepgraph::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = ShapeBuilder::MakeCompound({ base, tool });
        pid_map = tn->Update(bs.hist, out_shape, old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(out_shape);
    merge_to_vt(tn, vt, base, tool, dst, std::move(tool_world), pid_map, "trim");
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
    // Same big-pair OBB filtering + parallelism as Fuse: small ops stay
    // serial so the golden corpus keeps its byte-stable ordering, big
    // pairs fan out via OSD_Parallel (BREPKIT_BOP_PARALLEL=0 reverts to
    // serial for A/B). See the rationale in Fuse.
    if (BopProfFaceCount(s1->GetShape()) +
        BopProfFaceCount(s2->GetShape()) > 600) {
        algo.SetUseOBB(Standard_True);
        algo.SetRunParallel(bop_parallel());
    }
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
    // Big-pair booleans get OCCT's OBB filtering -- tight oriented boxes
    // prune the cross-seam face-pair candidates that conservative AABBs
    // keep (every face near a mirror plane has an AABB crossing it), and
    // the pruning is DETERMINISTIC -- plus OCCT-internal parallelism
    // (intersection + DS-filler stages fan out via OSD_Parallel;
    // BREPKIT_BOP_PARALLEL=0 reverts to serial for A/B). Small ops stay
    // serial: keeping the golden corpus' face ordering byte-stable is
    // worth more there than the microseconds parallelism would buy.
    const int faces_s1 = BopProfFaceCount(s1->GetShape());
    const int faces_s2 = BopProfFaceCount(s2->GetShape());
    const bool big_pair = (faces_s1 + faces_s2) > 600;

    // 1e-6 m fuzzy absorbs precision noise so face-coincident operands
    // (e.g. an extruded pad with an r=R cylindrical hole fused against
    // an annulus whose inner radius is also R) intersect correctly
    // rather than silently returning an empty COMPOUND.
    auto run_fuse = [&](double fuzzy, bool glue = false) {
        auto a = std::make_unique<BRepAlgoAPI_Fuse>();
        TopTools_ListOfShape args;  args.Append(s1->GetShape());
        TopTools_ListOfShape tools; tools.Append(s2->GetShape());
        a->SetArguments(args);
        a->SetTools(tools);
        a->SetFuzzyValue(fuzzy);
        if (glue) {
            a->SetGlue(BOPAlgo_GlueShift);
        }
        if (big_pair) {
            a->SetUseOBB(Standard_True);
            a->SetRunParallel(bop_parallel());
        }
        a->Build();
        return a;
    };

    const double base_fuzzy =
        ScaledBopFuzzy(s1->GetShape(), s2->GetShape());
    const auto _bop_t0 = std::chrono::steady_clock::now();

    // ---- Mirror-seam GLUE fast path ----
    //
    // Fusing a body with its mirrored half is the document's single
    // most expensive boolean (R2900: two ~2400-face halves, >12 min of
    // CPU in one BRepAlgoAPI_Fuse) because the general BOP intersects
    // every face pair across the seam. But a mirror seam is EXACTLY
    // the case BOPAlgo_GlueShift was built for: the operands touch
    // only on coincident faces in one plane, no real intersections
    // exist. Detection is geometric and conservative: both operands
    // big, and their AABBs overlap only in a slab no thicker than a
    // few fuzzy widths (true plane contact; a pattern instance pushed
    // 0.02 mm INTO the body fails this gate by an order of magnitude).
    // The glue result is accepted only when it actually merged
    // (fewer solids than the operand sum) and its volume equals the
    // operand sum to 1e-8 relative -- non-overlapping operands fuse to
    // exactly vol1+vol2, anything glue got wrong shows up there.
    // Any rejection falls through to the standard full fuse.
    std::unique_ptr<BRepAlgoAPI_Fuse> algo;
    if (big_pair)
    {
        Bnd_Box b1, b2;
        BRepBndLib::Add(s1->GetShape(), b1);
        BRepBndLib::Add(s2->GetShape(), b2);
        if (!b1.IsVoid() && !b2.IsVoid())
        {
            double x1a, y1a, z1a, x1b, y1b, z1b;
            double x2a, y2a, z2a, x2b, y2b, z2b;
            b1.Get(x1a, y1a, z1a, x1b, y1b, z1b);
            b2.Get(x2a, y2a, z2a, x2b, y2b, z2b);
            const double ox = std::min(x1b, x2b) - std::max(x1a, x2a);
            const double oy = std::min(y1b, y2b) - std::max(y1a, y2a);
            const double oz = std::min(z1b, z2b) - std::max(z1a, z2a);
            const double slab = std::min({ ox, oy, oz });

            // Gate decision + operand stats, unconditional for big
            // pairs: when one of these grinds for minutes the log
            // otherwise says nothing about WHY the fast path didn't
            // take it.
            std::fprintf(stderr,
                "[fuse] op_id=%u big_pair faces=%d+%d solids=%d+%d "
                "overlap=(%.4g,%.4g,%.4g) slab=%.4g gate=%.4g\n",
                op_id, faces_s1, faces_s2,
                CountSolids(s1->GetShape()), CountSolids(s2->GetShape()),
                ox, oy, oz, slab, 8.0 * base_fuzzy);
            std::fflush(stderr);

            // CAX_BOP_DUMP=1: write the operands of every big-pair
            // fuse to CWD so a pathological pair can be re-run in
            // isolation (zw_verify --fuse-probe a.brep b.brep)
            // instead of re-replaying an 8-minute document prefix
            // per experiment.
            static const bool bop_dump = [] {
                const char* e = std::getenv("CAX_BOP_DUMP");
                return e && e[0] && e[0] != '0';
            }();
            if (bop_dump) {
                char fn[64];
                std::snprintf(fn, sizeof fn, "bop_%u_a.brep", op_id);
                BRepTools::Write(s1->GetShape(), fn);
                std::snprintf(fn, sizeof fn, "bop_%u_b.brep", op_id);
                BRepTools::Write(s2->GetShape(), fn);
            }

            // CAX_FUSE_GLUE=force: bypass the slab gate (offline
            // --fuse-probe experiments on dumped operand pairs).
            static const bool force_glue = [] {
                const char* e = std::getenv("CAX_FUSE_GLUE");
                return e && std::strcmp(e, "force") == 0;
            }();
            if (slab <= 8.0 * base_fuzzy || force_glue)
            {
                auto glued = run_fuse(base_fuzzy, /*glue=*/true);
                if (glued->IsDone() && !glued->Shape().IsNull())
                {
                    const int ns1 = CountSolids(s1->GetShape());
                    const int ns2 = CountSolids(s2->GetShape());
                    const int nsr = CountSolids(glued->Shape());
                    double v1 = 0.0, v2 = 0.0, vr = 0.0;
                    try {
                        GProp_GProps g1, g2, gr;
                        BRepGProp::VolumeProperties(s1->GetShape(), g1);
                        BRepGProp::VolumeProperties(s2->GetShape(), g2);
                        BRepGProp::VolumeProperties(glued->Shape(), gr);
                        v1 = g1.Mass(); v2 = g2.Mass(); vr = gr.Mass();
                    } catch (...) { vr = 0.0; }
                    const double vsum = v1 + v2;
                    const double vrel = (vsum > 0.0)
                        ? std::fabs(vr - vsum) / vsum : 1.0;
                    if (nsr > 0 && nsr < ns1 + ns2 && vrel <= 1e-8)
                    {
                        std::fprintf(stderr,
                            "[fuse] op_id=%u GLUE seam fast path ok "
                            "(faces %d+%d, solids %d+%d->%d, "
                            "vol_rel=%.2e)\n",
                            op_id, faces_s1, faces_s2, ns1, ns2, nsr,
                            vrel);
                        std::fflush(stderr);
                        algo = std::move(glued);
                    }
                    else
                    {
                        std::fprintf(stderr,
                            "[fuse] op_id=%u GLUE attempt rejected "
                            "(solids %d+%d->%d, vol_rel=%.2e); full "
                            "fuse\n",
                            op_id, ns1, ns2, nsr, vrel);
                        std::fflush(stderr);
                    }
                }
            }
        }
    }

    if (!algo) {
        algo = run_fuse(base_fuzzy);
    }
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

    // Big-pair zero-yield guard. Each retry re-pays the FULL boolean,
    // and on a 1000+ face body that is ~1s per attempt -- yet across
    // every profiled run of the big-import corpus AND the golden suite
    // the escalation has never once been accepted on a big pair
    // (112/112 retry bursts ended accepted=0; r2900_100 alone wasted
    // ~3s/run on ops 77/78/418/419). The gap the retry exists for
    // (Page_033's sub-fuzzy seam) is a small-model artifact that
    // ScaledBopFuzzy now bridges at base fuzzy. Cap the rescue to
    // pairs small enough that a wasted burst is noise; the skip is
    // logged below so a future counter-example is visible in forensics.
    // BREPKIT_BOP_RETRY_MAXFACES tunes the cap (0 = uncapped).
    static const int retry_max_faces = [] {
        const char* e = std::getenv("BREPKIT_BOP_RETRY_MAXFACES");
        if (e && e[0]) return std::atoi(e);
        return 600;
    }();
    const bool retry_big_pair = is_retry_candidate && retry_max_faces > 0 &&
        (BopProfFaceCount(s1->GetShape()) +
         BopProfFaceCount(s2->GetShape()) > retry_max_faces);
    if (retry_big_pair)
    {
        std::fprintf(stderr,
            "[bop_retry] op_id=%u solids=%d candidate SKIPPED (big pair, "
            "faces>%d)\n",
            op_id, CountSolids(algo->Shape()), retry_max_faces);
        std::fflush(stderr);
    }

    if (is_retry_candidate && !retry_big_pair && bop_retry_mode() != 0)
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
    else if (is_retry_candidate && !retry_big_pair)
    {
        // Retry disabled by env -- record that this fuse WOULD have retried,
        // so the A/B (BREPKIT_BOP_RETRY=0) still shows how many candidates exist.
        std::fprintf(stderr,
            "[bop_retry] op_id=%u solids=%d candidate (RETRY DISABLED)\n",
            op_id, CountSolids(algo->Shape()));
        std::fflush(stderr);
    }

    // A fuse whose base operand HAS solids can never legitimately produce
    // an EMPTY result, yet BRepAlgoAPI_Fuse does exactly that (IsDone, no
    // errors, bare empty COMPOUND) on some pathological tools -- R2900's
    // Extrude44 fused an 8-sliver prism (fragmented saw-tooth profile)
    // onto the running body and the WHOLE body came back empty, wiping
    // every feature built so far. Retry with escalated fuzzy looking for
    // ANY solid-bearing result before falling into the last-resort guard
    // below.
    if (algo->IsDone() && !algo->Shape().IsNull() &&
        CountSolids(algo->Shape()) == 0 &&
        CountSolids(s1->GetShape()) > 0)
    {
        // Escalation is capped for big pairs: each retry re-runs the
        // full boolean, and on a many-hundred-face body one pass is
        // already minutes -- quadrupling that for a recovery that
        // rarely succeeds past 10x is how a slow load becomes a
        // "hung" one. Small bodies keep the full 10/100/1000 ladder.
        std::vector<double> ladder = { base_fuzzy * 10.0 };
        if (!big_pair) {
            ladder.push_back(base_fuzzy * 100.0);
            ladder.push_back(base_fuzzy * 1000.0);
        }
        int ran = 0, ok = 0;
        for (double fuzzy : ladder) {
            auto retry = run_fuse(fuzzy);
            ++ran;
            if (retry->IsDone() && !retry->Shape().IsNull() &&
                CountSolids(retry->Shape()) > 0) {
                algo = std::move(retry);
                ++ok;
                break;
            }
        }
        std::fprintf(stderr,
            "[bop_retry] op_id=%u EMPTY fuse result  ran=%d recovered=%d\n",
            op_id, ran, ok);
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

    // Last-resort guard: never hand an empty fuse downstream when the
    // base had solids. One failed boolean must not erase the running
    // body -- before this guard, R2900's chain died at Extrude44 and the
    // document emitted only a handful of post-collapse fragments
    // ("nothing displayed" in the editor). Keep the operands side by
    // side instead: locally-unmerged geometry beats a vanished part.
    // The operands' TShapes (and their TopoNaming entries) are reused
    // verbatim, so downstream geometric edge/face resolution still works;
    // skip tn->Update, whose history maps into the dropped empty result.
    bool fused_fallback = false;
    if ((refined.IsNull() || CountSolids(refined) == 0) &&
        CountSolids(s1->GetShape()) > 0)
    {
        std::fprintf(stderr,
            "[fuse] op_id=%u empty result; keeping unmerged operands\n",
            op_id);
        std::fflush(stderr);
        refined = ShapeBuilder::MakeCompound({ s1, s2 })->GetShape();
        fused_fallback = true;
    }

    brepdb::BRepWorld tool_world;
    if (tn && vt) { tool_world = serialize_world(tn, s2->GetShape()); }

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn && !fused_fallback)
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
    // Same big-pair OBB + parallelism gate as Cut / Fuse.
    if (BopProfFaceCount(s1->GetShape()) +
        BopProfFaceCount(s2->GetShape()) > 600) {
        algo.SetUseOBB(Standard_True);
        algo.SetRunParallel(bop_parallel());
    }
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

std::shared_ptr<TopoShape> TopoAlgo::SewJoin(const std::shared_ptr<TopoShape>& base, const std::shared_ptr<TopoShape>& tool,
                                             double tolerance,
                                             uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                             const std::shared_ptr<brepdb::VersionTree>& vt)
{
    auto has_solid = [](const TopoDS_Shape& s) {
        return TopExp_Explorer(s, TopAbs_SOLID).More();
    };
    // A combine-add where BOTH operands are solids is a plain boolean.
    // Solid + SHEET stays on the joinery path below: ZW3D's combine of
    // the closed ear box with the dome skin encloses the dome region
    // against the box wall -- a fuse can't create that volume, and the
    // solidxsheet BOP on these B-spline skins exploded to the memory
    // cap (bad alloc under the 8G Job Object fence).
    if (has_solid(base->GetShape()) && has_solid(tool->GetShape()))
    {
        std::fprintf(stderr, "[sew] op_id=%u solid operands -> fuse\n",
                     op_id);
        std::fflush(stderr);
        return Fuse(base, tool, op_id, tn, vt);
    }

    std::vector<TopoDS_Shape> faces;
    std::vector<TopoDS_Shape> riders;   // wires / edges ride along uncut
    auto feed = [&](const TopoDS_Shape& s)
    {
        for (TopExp_Explorer fx(s, TopAbs_FACE); fx.More(); fx.Next())
        {
            faces.push_back(fx.Current());
        }
        // Top-level non-face content (free wires from BakedShapes etc.).
        if (s.ShapeType() == TopAbs_COMPOUND)
        {
            for (TopoDS_Iterator it(s); it.More(); it.Next())
            {
                const TopAbs_ShapeEnum t = it.Value().ShapeType();
                if (t == TopAbs_WIRE || t == TopAbs_EDGE || t == TopAbs_VERTEX) {
                    riders.push_back(it.Value());
                }
            }
        }
    };
    feed(base->GetShape());
    feed(tool->GetShape());
    const int n_in_faces = (int)faces.size();
    if (n_in_faces == 0)
    {
        std::fprintf(stderr,
            "[sew] op_id=%u WARNING: no faces to sew, base UNCHANGED\n",
            op_id);
        std::fflush(stderr);
        return base;
    }
    // Free edges of a shape: non-degenerate edges with < 2 face parents.
    auto free_edge_count = [](const TopoDS_Shape& s) -> int {
        TopTools_IndexedDataMapOfShapeListOfShape e2f;
        TopExp::MapShapesAndAncestors(s, TopAbs_EDGE, TopAbs_FACE, e2f);
        int n = 0;
        for (int i = 1; i <= e2f.Extent(); ++i)
        {
            const TopoDS_Edge& e = TopoDS::Edge(e2f.FindKey(i));
            if (BRep_Tool::Degenerated(e)) { continue; }
            if (e2f.FindFromIndex(i).Extent() < 2) { ++n; }
        }
        return n;
    };
    // The replayed pieces meet ZW3D's at slightly different cut lines
    // (a trim's witnessed boundary is kernel-dependent), so the joint
    // gap can exceed the authored sew tolerance. Escalate x10 while the
    // seam stays open, capped at 0.5 mm -- far below any feature size,
    // far above any legitimate seam.
    double       tol  = (tolerance > 0.0 ? tolerance : 1e-6);
    TopoDS_Shape sewn;
    for (int attempt = 0; attempt < 4; ++attempt)
    {
        BRepBuilderAPI_Sewing sew(tol);
        for (const TopoDS_Shape& f : faces) {
            sew.Add(f);
        }
        sew.Perform();
        sewn = sew.SewedShape();
        const int open = sewn.IsNull() ? -1 : free_edge_count(sewn);
        std::fprintf(stderr,
            "[sew] op_id=%u attempt=%d tol=%.4g free_edges=%d\n",
            op_id, attempt, tol, open);
        std::fflush(stderr);
        if (open == 0) {
            break;
        }
        if (tol >= 5e-4) {
            break;
        }
        tol = std::min(tol * 10.0, 5e-4);
    }
    if (sewn.IsNull())
    {
        std::fprintf(stderr,
            "[sew] op_id=%u WARNING: sewing produced nothing, base "
            "UNCHANGED\n", op_id);
        std::fflush(stderr);
        return base;
    }

    // Closed shell -> solid. "Closed" = every non-degenerate edge is
    // shared by >= 2 faces (the Closed() flag is not reliably set by the
    // sewer). An inside-out solid (negative volume) is reversed -- this
    // is what lets the source's explicit face-flip feature stay a no-op.
    auto shell_closed = [](const TopoDS_Shape& shell) -> bool {
        TopTools_IndexedDataMapOfShapeListOfShape e2f;
        TopExp::MapShapesAndAncestors(shell, TopAbs_EDGE, TopAbs_FACE, e2f);
        for (int i = 1; i <= e2f.Extent(); ++i)
        {
            const TopoDS_Edge& e = TopoDS::Edge(e2f.FindKey(i));
            if (BRep_Tool::Degenerated(e)) { continue; }
            if (e2f.FindFromIndex(i).Extent() < 2) { return false; }
        }
        return e2f.Extent() > 0;
    };
    BRep_Builder bb;
    TopoDS_Compound out_c;
    bb.MakeCompound(out_c);
    int n_solids = 0, n_open = 0, n_loose = 0;
    auto place = [&](const TopoDS_Shape& piece)
    {
        if (piece.ShapeType() == TopAbs_SHELL && shell_closed(piece))
        {
            TopoDS_Solid sol;
            bb.MakeSolid(sol);
            bb.Add(sol, piece);
            GProp_GProps g;
            BRepGProp::VolumeProperties(sol, g);
            if (g.Mass() < 0.0) {
                sol.Reverse();
            }
            bb.Add(out_c, sol);
            ++n_solids;
        }
        else
        {
            bb.Add(out_c, piece);
            (piece.ShapeType() == TopAbs_SHELL ? n_open : n_loose) += 1;
        }
    };
    if (sewn.ShapeType() == TopAbs_COMPOUND)
    {
        for (TopoDS_Iterator it(sewn); it.More(); it.Next()) {
            place(it.Value());
        }
    }
    else
    {
        place(sewn);
    }
    for (const TopoDS_Shape& r : riders) {
        bb.Add(out_c, r);
    }
    // Sewing only welds edge-to-edge. A combine whose pieces meet in a
    // T-joint (02-ear 组合1: the dome rim lies EXACTLY mid-face on the
    // wall band, min_dist 8e-17 with no edge to pair) can never close by
    // sewing alone -- the source kernel imprints and extracts the
    // enclosed region. Bounded path first: mutually imprint base and
    // tool with the SAME splitter the trim arm uses (proven ~100 ms on
    // exactly these surfaces, where MakerVolume's全对求交 ran the
    // machine out of memory), sew NON-MANIFOLD so a rim edge holds all
    // three meeting faces, then peel dangling faces -- a flap has a
    // free outer edge and dies, the volume boundary survives closed.
    // Peeling only selects the solidification candidates: peeled faces
    // stay in the output as the open complex (ZW3D keeps them too).
    if (n_solids == 0 && n_in_faces >= 2)
    {
        // Rim imprint, NOT face-face splitting: the only intersection a
        // T-joint needs is the toolside FREE BOUNDARY scribed onto the
        // base face it rests on. Edge-on-face splitting
        // (BRepFeat_SplitShape) is cheap and bounded -- the face-face
        // SS intersector on this part's dome x wall B-spline pair is
        // exactly what ate the machine via MakerVolume AND via a
        // whole-compound BRepAlgoAPI_Splitter.
        auto rim_imprint = [op_id, this_tol = tol](
            const TopoDS_Shape& dst, const TopoDS_Shape& src) -> TopoDS_Shape
        {
            // Free boundary edges of src.
            TopTools_IndexedDataMapOfShapeListOfShape e2f;
            TopExp::MapShapesAndAncestors(src, TopAbs_EDGE, TopAbs_FACE, e2f);
            std::vector<TopoDS_Edge> rim;
            for (int i = 1; i <= e2f.Extent(); ++i)
            {
                const TopoDS_Edge& e = TopoDS::Edge(e2f.FindKey(i));
                if (BRep_Tool::Degenerated(e)) { continue; }
                if (e2f.FindFromIndex(i).Extent() < 2) {
                    rim.push_back(e);
                }
            }
            if (rim.empty()) {
                return dst;
            }
            // Keep only rim edges that actually REST on a dst face --
            // they become the splitter's TOOLS. Edge tools mean every
            // interference pair is edge x face (cheap, robust); it was
            // the face x face SS intersector that ate the machine.
            TopTools_ListOfShape cutters;
            int n_pairs = 0;
            for (const TopoDS_Edge& e : rim)
            {
                bool rests = false;
                for (TopExp_Explorer fx(dst, TopAbs_FACE);
                     fx.More() && !rests; fx.Next())
                {
                    BRepExtrema_DistShapeShape dist(e, fx.Current());
                    rests = dist.IsDone() && dist.NbSolution() > 0 &&
                            dist.Value() <= std::max(this_tol, 1e-5);
                }
                if (rests)
                {
                    cutters.Append(e);
                    ++n_pairs;
                }
            }
            if (n_pairs == 0) {
                return dst;
            }
            try
            {
                TopTools_ListOfShape args;
                args.Append(dst);
                BRepAlgoAPI_Splitter sp;
                sp.SetArguments(args);
                sp.SetTools(cutters);
                sp.SetFuzzyValue(std::max(this_tol, 1e-6));
                sp.Build();
                if (sp.IsDone())
                {
                    std::fprintf(stderr,
                        "[sew] op_id=%u rim_imprint edges=%d ok\n",
                        op_id, n_pairs);
                    std::fflush(stderr);
                    return sp.Shape();
                }
            }
            catch (Standard_Failure& sf)
            {
                const char* m = sf.GetMessageString();
                std::fprintf(stderr,
                    "[sew] op_id=%u rim_imprint THREW %s\n",
                    op_id, m ? m : "Standard_Failure");
                std::fflush(stderr);
            }
            return dst;
        };
        // Tool rim onto base faces, then base rim onto (possibly split)
        // tool faces -- both directions of a general T-joint.
        const TopoDS_Shape ib  = rim_imprint(base->GetShape(),
                                             tool->GetShape());
        const TopoDS_Shape it_ = rim_imprint(tool->GetShape(), ib);

        BRepBuilderAPI_Sewing nsew(tol);
        nsew.SetNonManifoldMode(Standard_True);
        int n_imp = 0;
        for (const TopoDS_Shape* src : { &ib, &it_ })
        {
            for (TopExp_Explorer fx(*src, TopAbs_FACE); fx.More(); fx.Next())
            {
                nsew.Add(fx.Current());
                ++n_imp;
            }
        }
        nsew.Perform();
        const TopoDS_Shape nm = nsew.SewedShape();
        if (!nm.IsNull() && n_imp >= 4)
        {
            // Peel: drop faces with any free (single-parent) edge until
            // stable. Shared TShapes from the non-manifold sew make the
            // parent counts honest across pieces.
            std::vector<TopoDS_Shape> all;
            for (TopExp_Explorer fx(nm, TopAbs_FACE); fx.More(); fx.Next()) {
                all.push_back(fx.Current());
            }
            std::vector<bool> alive(all.size(), true);
            bool pruned = true;
            while (pruned)
            {
                pruned = false;
                BRep_Builder cb;
                TopoDS_Compound cc;
                cb.MakeCompound(cc);
                for (size_t i = 0; i < all.size(); ++i) {
                    if (alive[i]) { cb.Add(cc, all[i]); }
                }
                TopTools_IndexedDataMapOfShapeListOfShape e2f;
                TopExp::MapShapesAndAncestors(cc, TopAbs_EDGE, TopAbs_FACE,
                                              e2f);
                for (size_t i = 0; i < all.size(); ++i)
                {
                    if (!alive[i]) { continue; }
                    bool dangling = false;
                    for (TopExp_Explorer ex(all[i], TopAbs_EDGE); ex.More();
                         ex.Next())
                    {
                        const TopoDS_Edge& e = TopoDS::Edge(ex.Current());
                        if (BRep_Tool::Degenerated(e)) { continue; }
                        const int idx = e2f.FindIndex(e);
                        if (idx == 0 ||
                            e2f.FindFromIndex(idx).Extent() < 2)
                        {
                            dangling = true;
                            break;
                        }
                    }
                    if (dangling)
                    {
                        alive[i] = false;
                        pruned   = true;
                    }
                }
            }
            int n_alive = 0;
            for (bool a : alive) { n_alive += a ? 1 : 0; }
            std::fprintf(stderr,
                "[sew] op_id=%u imprint_peel faces=%d alive=%d\n",
                op_id, (int)all.size(), n_alive);
            std::fflush(stderr);
            if (n_alive >= 4)
            {
                // Survivors form the volume boundary: re-sew manifold,
                // solidify closed shells. Peeled faces rejoin the output
                // as the open complex.
                BRepBuilderAPI_Sewing msew(tol);
                for (size_t i = 0; i < all.size(); ++i) {
                    if (alive[i]) { msew.Add(all[i]); }
                }
                msew.Perform();
                const TopoDS_Shape msewn = msew.SewedShape();
                int n_new_solids = 0;
                BRep_Builder ob;
                TopoDS_Compound oc;
                ob.MakeCompound(oc);
                auto try_solid = [&](const TopoDS_Shape& piece)
                {
                    if (piece.ShapeType() == TopAbs_SHELL &&
                        shell_closed(piece))
                    {
                        TopoDS_Solid sol;
                        ob.MakeSolid(sol);
                        ob.Add(sol, piece);
                        GProp_GProps g;
                        BRepGProp::VolumeProperties(sol, g);
                        if (g.Mass() < 0.0) {
                            sol.Reverse();
                        }
                        ob.Add(oc, sol);
                        ++n_new_solids;
                    }
                    else
                    {
                        ob.Add(oc, piece);
                    }
                };
                if (!msewn.IsNull())
                {
                    if (msewn.ShapeType() == TopAbs_COMPOUND)
                    {
                        for (TopoDS_Iterator i2(msewn); i2.More(); i2.Next()) {
                            try_solid(i2.Value());
                        }
                    }
                    else
                    {
                        try_solid(msewn);
                    }
                }
                std::fprintf(stderr,
                    "[sew] op_id=%u imprint_peel solids=%d\n",
                    op_id, n_new_solids);
                std::fflush(stderr);
                if (n_new_solids > 0)
                {
                    // Peeled flaps: sew what's left into open shell(s)
                    // so they stay one body each, then ride along.
                    int n_flap = 0;
                    BRepBuilderAPI_Sewing fsew(tol);
                    for (size_t i = 0; i < all.size(); ++i) {
                        if (!alive[i]) { fsew.Add(all[i]); ++n_flap; }
                    }
                    if (n_flap > 0)
                    {
                        fsew.Perform();
                        const TopoDS_Shape fs = fsew.SewedShape();
                        if (!fs.IsNull()) {
                            ob.Add(oc, fs);
                        }
                    }
                    for (const TopoDS_Shape& r : riders) {
                        ob.Add(oc, r);
                    }
                    out_c    = oc;
                    n_solids = n_new_solids;
                    n_open   = (n_flap > 0 ? 1 : 0);
                    n_loose  = (int)riders.size();
                }
            }
        }
    }
    // Solid-union-resting-sheet (ZW3D combine): the open shell's rim
    // rests ON the solid's wall (02-ear 组合1: dome rim at 8e-17 on the
    // ear box). Union boundary = solid faces minus the wall piece the
    // sheet covers, plus the sheet. Rim imprint (edge-on-face split,
    // cheap) carves that piece out; covered-vs-outside classification:
    // the nearest sheet point from a covered piece is INTERIOR, from an
    // outside piece it is on the sheet's own rim. No surface-surface
    // intersection anywhere -- the dome x wall SS pair is exactly what
    // ate the machine via MakerVolume.
    bool union_attempted = false;
    if (n_solids == 1 && n_open == 1)
    {
        union_attempted = true;
        TopoDS_Shape B, S;
        for (TopoDS_Iterator it(out_c); it.More(); it.Next())
        {
            if (it.Value().ShapeType() == TopAbs_SOLID) { B = it.Value(); }
            else if (it.Value().ShapeType() == TopAbs_SHELL) {
                S = it.Value();
            }
        }
        // Rim edges of S, for the imprint and the boundary test.
        TopTools_IndexedDataMapOfShapeListOfShape se2f;
        TopExp::MapShapesAndAncestors(S, TopAbs_EDGE, TopAbs_FACE, se2f);
        BRep_Builder rb;
        TopoDS_Compound rim_c;
        rb.MakeCompound(rim_c);
        TopTools_ListOfShape rim_edges;
        for (int i = 1; i <= se2f.Extent(); ++i)
        {
            const TopoDS_Edge& e = TopoDS::Edge(se2f.FindKey(i));
            if (BRep_Tool::Degenerated(e)) { continue; }
            if (se2f.FindFromIndex(i).Extent() < 2)
            {
                rb.Add(rim_c, e);
                rim_edges.Append(e);
            }
        }
        if (!B.IsNull() && !S.IsNull() && !rim_edges.IsEmpty())
        {
            try
            {
                TopTools_ListOfShape args;
                args.Append(B);
                BRepAlgoAPI_Splitter sp;
                sp.SetArguments(args);
                sp.SetTools(rim_edges);
                sp.SetFuzzyValue(std::max(tol, 1e-6));
                sp.Build();
                if (sp.IsDone())
                {
                // ROUTE 2 (preferred): non-manifold sew unifies the
                // imprinted edges across box pieces and sheet, then
                // BOPAlgo_BuilderSolid extracts every enclosed region by
                // orientation analysis -- no seal heuristics, no surface
                // intersections (the imprint already happened). Manifold
                // sewing loses this assembly to the pairing lottery: a
                // hole-cap edge carries THREE faces and the sewer welds
                // the two box-born ones, orphaning the sheet's rim.
                bool route2_done = false;
                {
                    BRepBuilderAPI_Sewing nms(std::max(tol, 1e-6));
                    nms.SetNonManifoldMode(Standard_True);
                    for (TopExp_Explorer fx(sp.Shape(), TopAbs_FACE);
                         fx.More(); fx.Next()) {
                        nms.Add(fx.Current());
                    }
                    for (TopExp_Explorer fx(S, TopAbs_FACE); fx.More();
                         fx.Next()) {
                        nms.Add(fx.Current());
                    }
                    nms.Perform();
                    const TopoDS_Shape nmshape = nms.SewedShape();
                    if (!nmshape.IsNull())
                    {
                        TopTools_ListOfShape lf;
                        int nf = 0;
                        for (TopExp_Explorer fx(nmshape, TopAbs_FACE);
                             fx.More(); fx.Next())
                        {
                            lf.Append(fx.Current()
                                          .Oriented(TopAbs_FORWARD));
                            lf.Append(fx.Current()
                                          .Oriented(TopAbs_REVERSED));
                            ++nf;
                        }
                        BOPAlgo_BuilderSolid bsld;
                        bsld.SetShapes(lf);
                        bsld.Perform();
                        if (!bsld.HasErrors())
                        {
                            BRep_Builder ub;
                            TopoDS_Compound uc;
                            ub.MakeCompound(uc);
                            int    n_areas = 0;
                            double tot     = 0.0;
                            for (TopTools_ListIteratorOfListOfShape it(
                                     bsld.Areas());
                                 it.More(); it.Next())
                            {
                                GProp_GProps g;
                                BRepGProp::VolumeProperties(it.Value(), g);
                                const double v = std::fabs(g.Mass());
                                if (v < 1e-15) { continue; }
                                TopoDS_Shape sol = it.Value();
                                if (g.Mass() < 0.0) {
                                    sol.Reverse();
                                }
                                ub.Add(uc, sol);
                                ++n_areas;
                                tot += v;
                            }
                            std::fprintf(stderr,
                                "[sew] op_id=%u nm+builder areas=%d "
                                "vol=%.6g (faces=%d)\n",
                                op_id, n_areas, tot * 1e9, nf);
                            std::fflush(stderr);
                            if (n_areas > 0)
                            {
                                // Multiple areas arise when T-joint
                                // imprinting encloses more than one
                                // region (e.g. interior + exterior cap).
                                // Fusing them is expensive and can OOM;
                                // for a sew-close op the correct body is
                                // the dominant (largest-volume) region.
                                TopoDS_Shape best;
                                double best_vol = -1.0;
                                for (TopoDS_Iterator it2(uc); it2.More();
                                     it2.Next())
                                {
                                    GProp_GProps gp2;
                                    BRepGProp::VolumeProperties(
                                        it2.Value(), gp2);
                                    const double v2 =
                                        std::fabs(gp2.Mass());
                                    if (v2 > best_vol) {
                                        best_vol = v2;
                                        best     = it2.Value();
                                    }
                                }
                                BRep_Builder ob;
                                TopoDS_Compound oc;
                                ob.MakeCompound(oc);
                                int n_out = 0;
                                if (!best.IsNull()) {
                                    ob.Add(oc, best);
                                    ++n_out;
                                }
                                for (const TopoDS_Shape& r : riders) {
                                    ob.Add(oc, r);
                                }
                                if (n_out > 0)
                                {
                                    out_c    = oc;
                                    n_solids = n_out;
                                    n_open   = 0;
                                    n_loose  = (int)riders.size();
                                    route2_done = true;
                                }
                            }
                        }
                        else
                        {
                            std::fprintf(stderr,
                                "[sew] op_id=%u nm+builder FAILED\n",
                                op_id);
                            std::fflush(stderr);
                        }
                    }
                }
                if (!route2_done)
                {
                    // SEAL pieces: fragments of B's faces that the rim
                    // bounds -- the wall piece UNDER the sheet (interior
                    // nearest-point) and the piece capping the sheet's
                    // planar OPENING (boundary mostly ON the rim; its
                    // interior test fails because the nearest sheet
                    // point from inside a hole is the hole's edge).
                    // B itself stays INTACT (it is already the closed
                    // half of the union); the seal pieces are COPIED
                    // into the sheet's shell so both regions close
                    // independently, then fused.
                    std::vector<TopoDS_Shape> seals;
                    for (TopExp_Explorer fx(sp.Shape(), TopAbs_FACE);
                         fx.More(); fx.Next())
                    {
                        GProp_GProps fg;
                        BRepGProp::SurfaceProperties(fx.Current(), fg);
                        const gp_Pnt c = fg.CentreOfMass();
                        TopoDS_Vertex cv = BRepBuilderAPI_MakeVertex(c);
                        BRepExtrema_DistShapeShape ds(cv, S);
                        bool seal = false;
                        double rim_d = 1e9, on_frac = 0.0;
                        if (ds.IsDone() && ds.NbSolution() > 0)
                        {
                            const gp_Pnt q = ds.PointOnShape2(1);
                            TopoDS_Vertex qv = BRepBuilderAPI_MakeVertex(q);
                            BRepExtrema_DistShapeShape dr(qv, rim_c);
                            rim_d = (dr.IsDone() && dr.NbSolution() > 0)
                                        ? dr.Value() : 1e9;
                            seal = rim_d > 2e-4;
                        }
                        if (!seal)
                        {
                            // Boundary-on-rim fraction.
                            double on_len = 0.0, tot_len = 0.0;
                            for (TopExp_Explorer ex(fx.Current(),
                                                    TopAbs_EDGE);
                                 ex.More(); ex.Next())
                            {
                                GProp_GProps lg;
                                BRepGProp::LinearProperties(ex.Current(),
                                                            lg);
                                tot_len += lg.Mass();
                                BRepExtrema_DistShapeShape de(
                                    ex.Current(), rim_c);
                                if (de.IsDone() && de.NbSolution() > 0 &&
                                    de.Value() <= 5e-4) {
                                    on_len += lg.Mass();
                                }
                            }
                            on_frac = tot_len > 0.0 ? on_len / tot_len
                                                    : 0.0;
                            seal = on_frac >= 0.7;
                        }
                        std::fprintf(stderr,
                            "[sew]   union_rest piece area=%.4g "
                            "rim_d=%.4g on_frac=%.2f seal=%d\n",
                            fg.Mass() * 1e6, rim_d, on_frac,
                            seal ? 1 : 0);
                        if (seal) { seals.push_back(fx.Current()); }
                    }
                    std::vector<TopoDS_Shape> uni;
                    for (TopExp_Explorer fx(S, TopAbs_FACE); fx.More();
                         fx.Next()) {
                        uni.push_back(fx.Current());
                    }
                    for (const TopoDS_Shape& f : seals) {
                        uni.push_back(f);
                    }
                    const int n_covered = (int)seals.size();
                    std::fprintf(stderr,
                        "[sew] op_id=%u union_rest seals=%d sheet_faces=%d\n",
                        op_id, n_covered, (int)uni.size());
                    std::fflush(stderr);
                    if (n_covered > 0)
                    {
                        BRepBuilderAPI_Sewing usew(std::max(tol, 1e-6));
                        for (const TopoDS_Shape& f : uni) {
                            usew.Add(f);
                        }
                        usew.Perform();
                        TopoDS_Shape us = usew.SewedShape();
                        // Planar-hole capping: a trim that cut both
                        // lineages at the same surface leaves a hole
                        // whose boundary loop is PLANAR (02-ear: the
                        // 拉伸8 z=8.2 cut -- truth seals it with the
                        // mutual-trim cap slivers, which distance-sign
                        // voting can never classify because they lie ON
                        // the partition). Synthesize the caps from the
                        // open loops instead: chain free edges by
                        // endpoint, MakeFace(wire) accepts only planar
                        // loops, re-sew. Non-planar openings fail
                        // MakeFace and degrade as before.
                        if (!us.IsNull() && us.ShapeType() == TopAbs_SHELL
                            && !shell_closed(us))
                        {
                            TopTools_IndexedDataMapOfShapeListOfShape h2f;
                            TopExp::MapShapesAndAncestors(
                                us, TopAbs_EDGE, TopAbs_FACE, h2f);
                            std::vector<TopoDS_Edge> open_e;
                            for (int i = 1; i <= h2f.Extent(); ++i)
                            {
                                const TopoDS_Edge& e =
                                    TopoDS::Edge(h2f.FindKey(i));
                                if (BRep_Tool::Degenerated(e)) {
                                    continue;
                                }
                                if (h2f.FindFromIndex(i).Extent() < 2) {
                                    open_e.push_back(e);
                                }
                            }
                            int n_caps = 0;
                            if (!open_e.empty() && open_e.size() <= 16)
                            {
                                try
                                {
                                    BRepBuilderAPI_MakeWire mw;
                                    for (const TopoDS_Edge& e : open_e) {
                                        mw.Add(e);
                                    }
                                    if (mw.IsDone())
                                    {
                                        BRepBuilderAPI_MakeFace mf(
                                            mw.Wire(), Standard_True);
                                        if (mf.IsDone())
                                        {
                                            uni.push_back(mf.Face());
                                            ++n_caps;
                                        }
                                    }
                                }
                                catch (Standard_Failure&) {}
                                if (n_caps > 0)
                                {
                                    BRepBuilderAPI_Sewing rsew(
                                        std::max(tol, 1e-6));
                                    for (const TopoDS_Shape& f : uni) {
                                        rsew.Add(f);
                                    }
                                    rsew.Perform();
                                    const TopoDS_Shape rs =
                                        rsew.SewedShape();
                                    if (!rs.IsNull() &&
                                        rs.ShapeType() == TopAbs_SHELL) {
                                        us = rs;
                                    }
                                }
                            }
                            std::fprintf(stderr,
                                "[sew] op_id=%u union_rest hole_caps=%d "
                                "(open_edges=%d)\n",
                                op_id, n_caps, (int)open_e.size());
                            std::fflush(stderr);
                        }
                        if (!us.IsNull() &&
                            us.ShapeType() == TopAbs_SHELL &&
                            shell_closed(us))
                        {
                            TopoDS_Solid sol;
                            BRep_Builder sb;
                            sb.MakeSolid(sol);
                            sb.Add(sol, us);
                            GProp_GProps g;
                            BRepGProp::VolumeProperties(sol, g);
                            if (g.Mass() < 0.0) {
                                sol.Reverse();
                                BRepGProp::VolumeProperties(sol, g);
                            }
                            std::fprintf(stderr,
                                "[sew] op_id=%u union_rest sheet region "
                                "closed vol=%.6g\n", op_id,
                                std::fabs(g.Mass()) * 1e9);
                            std::fflush(stderr);
                            // Two closed regions sharing their seal
                            // faces: fold into the single union solid
                            // (plain solid-solid fuse on glued inputs).
                            auto bsol = std::make_shared<brepkit::TopoShape>(B);
                            auto dsol = std::make_shared<brepkit::TopoShape>(
                                TopoDS_Shape(sol));
                            auto fused = Fuse(bsol, dsol, op_id, nullptr);
                            TopoDS_Shape result =
                                (fused && !fused->GetShape().IsNull())
                                    ? fused->GetShape()
                                    : TopoDS_Shape();
                            int n_out = 0;
                            BRep_Builder ob;
                            TopoDS_Compound oc;
                            ob.MakeCompound(oc);
                            if (!result.IsNull())
                            {
                                for (TopExp_Explorer sx(result,
                                                        TopAbs_SOLID);
                                     sx.More(); sx.Next())
                                {
                                    ob.Add(oc, sx.Current());
                                    ++n_out;
                                }
                            }
                            if (n_out == 0)
                            {
                                ob.Add(oc, B);
                                ob.Add(oc, sol);
                                n_out = 2;
                            }
                            std::fprintf(stderr,
                                "[sew] op_id=%u union_rest UNION "
                                "solids=%d\n", op_id, n_out);
                            std::fflush(stderr);
                            for (const TopoDS_Shape& r : riders) {
                                ob.Add(oc, r);
                            }
                            out_c    = oc;
                            n_solids = n_out;
                            n_open   = 0;
                            n_loose  = (int)riders.size();
                        }
                        else
                        {
                            std::fprintf(stderr,
                                "[sew] op_id=%u union_rest shell did not "
                                "close; kept sewn pieces\n", op_id);
                            // Census the unclosed edges: which boundary
                            // still has nothing to pair with.
                            if (!us.IsNull())
                            {
                                TopTools_IndexedDataMapOfShapeListOfShape
                                    ue2f;
                                TopExp::MapShapesAndAncestors(
                                    us, TopAbs_EDGE, TopAbs_FACE, ue2f);
                                for (int i = 1; i <= ue2f.Extent(); ++i)
                                {
                                    const TopoDS_Edge& e =
                                        TopoDS::Edge(ue2f.FindKey(i));
                                    if (BRep_Tool::Degenerated(e)) {
                                        continue;
                                    }
                                    if (ue2f.FindFromIndex(i).Extent()
                                        >= 2) {
                                        continue;
                                    }
                                    GProp_GProps lg;
                                    BRepGProp::LinearProperties(e, lg);
                                    Bnd_Box eb;
                                    BRepBndLib::Add(e, eb);
                                    double x0=0,y0=0,z0=0,x1=0,y1=0,z1=0;
                                    if (!eb.IsVoid()) {
                                        eb.Get(x0,y0,z0,x1,y1,z1);
                                    }
                                    std::fprintf(stderr,
                                        "[sew]   union_rest open_edge "
                                        "len=%.4g bbox=(%.4g,%.4g,%.4g)"
                                        "(%.4g,%.4g,%.4g)\n",
                                        lg.Mass(), x0,y0,z0,x1,y1,z1);
                                }
                            }
                            std::fflush(stderr);
                        }
                    }
                }
                }   // !route2_done (legacy seal path)
            }
            catch (Standard_Failure& sf)
            {
                const char* m = sf.GetMessageString();
                std::fprintf(stderr,
                    "[sew] op_id=%u union_rest THREW %s\n",
                    op_id, m ? m : "Standard_Failure");
                std::fflush(stderr);
            }
        }
    }
    // Last resort, budgeted: MakerVolume's blind face-soup intersection.
    // Its rap sheet on this part: unbounded OOM (machine froze), 60 s
    // budget burnouts, and an access violation on the post-standalone
    // composite args -- it has never once succeeded where the structured
    // paths failed. Run it only when the union path did not apply at
    // all (pure-sheet states), never after union_rest already degraded
    // gracefully.
    if (!union_attempted && (n_open > 0 || n_solids == 0))
    {
        BOPAlgo_MakerVolume mv;
        TopTools_ListOfShape args;
        // COMPOSITE args, not the face soup: the sewn pieces (a closed
        // box solid, a dome shell) go in whole, so the interference
        // stage runs once per piece-pair instead of once per face-pair
        // -- the decomposed form ran 13x13 B-spline intersections and
        // blew the 60 s budget on the very op it exists for.
        int n_args = 0;
        for (TopoDS_Iterator it(out_c); it.More(); it.Next())
        {
            const TopAbs_ShapeEnum t = it.Value().ShapeType();
            if (t == TopAbs_SOLID || t == TopAbs_SHELL ||
                t == TopAbs_FACE)
            {
                args.Append(it.Value());
                ++n_args;
            }
        }
        if (n_args == 0)
        {
            for (const TopoDS_Shape& f : faces) {
                args.Append(f);
            }
        }
        mv.SetArguments(args);
        mv.SetIntersect(Standard_True);
        mv.SetFuzzyValue(1e-6);
        // Budgeted: the T-joint volume extraction is worth seconds,
        // never the machine (see DeadlineIndicator). On timeout the
        // algo reports HasErrors() and we keep the sewn open shells --
        // the same degradation as any other maker_volume failure.
        // BREPKIT_MV_BUDGET_S overrides (seconds, default 30).
        static const double mv_budget = [] {
            if (const char* e = std::getenv("BREPKIT_MV_BUDGET_S")) {
                const double v = std::atof(e);
                if (v > 0.0) { return v; }
            }
            return 30.0;
        }();
        opencascade::handle<DeadlineIndicator> deadline =
            new DeadlineIndicator(mv_budget);
        // The deadline aborts only where OCCT polls UserBreak (between
        // pave-filler stages). The 02-ear 组合1 explosion lives INSIDE
        // one face-face intersection, which never polls -- there the
        // process memory budget (zw_verify installs a Job Object cap,
        // CAX_MEM_BUDGET_MB) fails the allocation instead, and the
        // throw below degrades to the sewn open shells.
        bool mv_threw = false;
        try
        {
            mv.Perform(deadline->Start());
        }
        catch (Standard_Failure& f)
        {
            mv_threw = true;
            const char* m = f.GetMessageString();
            std::fprintf(stderr,
                "[sew] op_id=%u maker_volume THREW %s (sewn open shells "
                "kept)\n", op_id, m ? m : "Standard_Failure");
        }
        catch (const std::bad_alloc&)
        {
            mv_threw = true;
            std::fprintf(stderr,
                "[sew] op_id=%u maker_volume THREW bad_alloc (memory "
                "budget hit, sewn open shells kept)\n", op_id);
        }
        std::fflush(stderr);
        if (!mv_threw && !mv.HasErrors())
        {
            int    n_mv     = 0;
            double mv_vol   = 0.0;
            BRep_Builder vb;
            TopoDS_Compound vols;
            vb.MakeCompound(vols);
            for (TopExp_Explorer sx(mv.Shape(), TopAbs_SOLID); sx.More();
                 sx.Next())
            {
                GProp_GProps g;
                BRepGProp::VolumeProperties(sx.Current(), g);
                TopoDS_Shape sol = sx.Current();
                if (g.Mass() < 0.0) {
                    sol.Reverse();
                }
                vb.Add(vols, sol);
                mv_vol += std::fabs(g.Mass());
                ++n_mv;
            }
            std::fprintf(stderr,
                "[sew] op_id=%u maker_volume solids=%d vol=%.6g\n",
                op_id, n_mv, mv_vol * 1e9);
            if (n_mv > 0)
            {
                // MakerVolume returns each enclosed region separately --
                // the ear box and the dome region share their imprinted
                // wall as two glued solids where the source has ONE.
                // Fold them with a plain solid-solid fuse (cheap and
                // clean on glued inputs).
                TopoDS_Shape merged;
                if (n_mv > 1)
                {
                    std::shared_ptr<TopoShape> acc;
                    for (TopoDS_Iterator it(vols); it.More(); it.Next())
                    {
                        auto cur = std::make_shared<brepkit::TopoShape>(
                            it.Value());
                        acc = acc ? Fuse(acc, cur, op_id, nullptr) : cur;
                    }
                    if (acc && !acc->GetShape().IsNull())
                    {
                        merged = acc->GetShape();
                        int nm = 0;
                        for (TopExp_Explorer sx(merged, TopAbs_SOLID);
                             sx.More(); sx.Next()) {
                            ++nm;
                        }
                        std::fprintf(stderr,
                            "[sew] op_id=%u maker_volume fold %d -> %d "
                            "solid(s)\n", op_id, n_mv, nm);
                        n_mv = nm;
                    }
                }
                // The enclosed region replaces the sewn complex (its
                // boundary faces ARE the complex); riders still tag
                // along.
                BRep_Builder ob;
                TopoDS_Compound oc;
                ob.MakeCompound(oc);
                if (!merged.IsNull())
                {
                    for (TopExp_Explorer sx(merged, TopAbs_SOLID);
                         sx.More(); sx.Next()) {
                        ob.Add(oc, sx.Current());
                    }
                }
                else
                {
                    for (TopoDS_Iterator it(vols); it.More(); it.Next()) {
                        ob.Add(oc, it.Value());
                    }
                }
                for (const TopoDS_Shape& r : riders) {
                    ob.Add(oc, r);
                }
                out_c    = oc;
                n_solids = n_mv;
                n_open   = 0;
                n_loose  = (int)riders.size();
            }
        }
        else if (!mv_threw)
        {
            std::fprintf(stderr,
                "[sew] op_id=%u maker_volume %s\n", op_id,
                deadline->Expired()
                    ? "TIMED OUT (budget spent, sewn open shells kept; "
                      "BREPKIT_MV_BUDGET_S raises the budget)"
                    : "FAILED (errors)");
        }
    }
    TopoDS_Shape out_shape = out_c;
    {
        TopoDS_Iterator solo(out_c);
        if (solo.More())
        {
            const TopoDS_Shape first = solo.Value();
            solo.Next();
            if (!solo.More()) {
                out_shape = first;
            }
        }
    }
    std::fprintf(stderr,
        "[sew] op_id=%u faces=%d -> solids=%d open_shells=%d loose=%d "
        "tol=%.4g\n",
        op_id, n_in_faces, n_solids, n_open, n_loose, tolerance);
    // Open-seam forensics: where does each surviving open shell gape?
    // Per shell: face count, free-edge count + summed length + bbox of
    // the free edges -- enough to tell a genuinely detached stray from
    // a joint that misses closure by a hair.
    if (n_open > 0)
    {
        int si = 0;
        for (TopoDS_Iterator it(out_c); it.More(); it.Next(), ++si)
        {
            const TopoDS_Shape& piece = it.Value();
            if (piece.ShapeType() != TopAbs_SHELL) { continue; }
            TopTools_IndexedDataMapOfShapeListOfShape e2f;
            TopExp::MapShapesAndAncestors(piece, TopAbs_EDGE, TopAbs_FACE,
                                          e2f);
            int    nfree = 0;
            double flen  = 0.0;
            Bnd_Box fb;
            for (int i = 1; i <= e2f.Extent(); ++i)
            {
                const TopoDS_Edge& e = TopoDS::Edge(e2f.FindKey(i));
                if (BRep_Tool::Degenerated(e)) { continue; }
                if (e2f.FindFromIndex(i).Extent() >= 2) { continue; }
                ++nfree;
                GProp_GProps lg;
                BRepGProp::LinearProperties(e, lg);
                flen += lg.Mass();
                BRepBndLib::Add(e, fb);
            }
            int nf = 0;
            for (TopExp_Explorer fx(piece, TopAbs_FACE); fx.More();
                 fx.Next()) {
                ++nf;
            }
            double x0=0, y0=0, z0=0, x1=0, y1=0, z1=0;
            if (!fb.IsVoid()) fb.Get(x0, y0, z0, x1, y1, z1);
            std::fprintf(stderr,
                "[sew]   shell=%d faces=%d free_edges=%d free_len=%.4g "
                "free_bbox=(%.4g,%.4g,%.4g)(%.4g,%.4g,%.4g)\n",
                si, nf, nfree, flen, x0, y0, z0, x1, y1, z1);
        }
        // T-joint detector: for every open shell, how far is its free
        // boundary from the OTHER pieces' geometry? ~0 with no edge to
        // sew to = the rim lands mid-face (needs imprint, not sewing);
        // large = the pieces genuinely don't line up.
        std::vector<TopoDS_Shape> pieces;
        for (TopoDS_Iterator it(out_c); it.More(); it.Next()) {
            pieces.push_back(it.Value());
        }
        for (size_t a = 0; a < pieces.size(); ++a)
        {
            if (pieces[a].ShapeType() != TopAbs_SHELL) { continue; }
            TopTools_IndexedDataMapOfShapeListOfShape e2f;
            TopExp::MapShapesAndAncestors(pieces[a], TopAbs_EDGE,
                                          TopAbs_FACE, e2f);
            BRep_Builder fb_b;
            TopoDS_Compound free_c;
            fb_b.MakeCompound(free_c);
            int nfree = 0;
            for (int i = 1; i <= e2f.Extent(); ++i)
            {
                const TopoDS_Edge& e = TopoDS::Edge(e2f.FindKey(i));
                if (BRep_Tool::Degenerated(e)) { continue; }
                if (e2f.FindFromIndex(i).Extent() >= 2) { continue; }
                fb_b.Add(free_c, e);
                ++nfree;
            }
            if (nfree == 0) { continue; }
            // Per free EDGE: how far is it from each other piece? A
            // mid-face rest reads ~0 (T-joint, needs imprint); a
            // millimetre reads "pieces genuinely apart there"; tens of
            // millimetres reads "the sealing piece is missing".
            int ei = 0;
            for (TopoDS_Iterator eit(free_c); eit.More(); eit.Next(), ++ei)
            {
                Bnd_Box eb;
                BRepBndLib::Add(eit.Value(), eb);
                double x0=0, y0=0, z0=0, x1=0, y1=0, z1=0;
                if (!eb.IsVoid()) eb.Get(x0, y0, z0, x1, y1, z1);
                GProp_GProps lg;
                BRepGProp::LinearProperties(eit.Value(), lg);
                for (size_t b = 0; b < pieces.size(); ++b)
                {
                    if (a == b) { continue; }
                    BRepExtrema_DistShapeShape dist(eit.Value(), pieces[b]);
                    if (dist.IsDone() && dist.NbSolution() > 0)
                    {
                        std::fprintf(stderr,
                            "[sew]   freeb shell=%d edge=%d len=%.4g -> "
                            "piece=%d min_dist=%.4g "
                            "ebbox=(%.4g,%.4g,%.4g)(%.4g,%.4g,%.4g)\n",
                            (int)a, ei, lg.Mass(), (int)b, dist.Value(),
                            x0, y0, z0, x1, y1, z1);
                    }
                }
            }
        }
    }
    std::fflush(stderr);

    // BRepBuilderAPI_Sewing exposes no BRepTools_History; naming across a
    // sew is dropped (same as the legacy Sew above). The verify path runs
    // without tn anyway; editor naming resumes at the next history op.
    brepdb::BRepWorld tool_world;
    if (tn && vt) { tool_world = serialize_world(tn, tool->GetShape()); }
    brepgraph::TopoNaming::PidMap pid_map;
    auto dst = std::make_shared<brepkit::TopoShape>(out_shape);
    merge_to_vt(tn, vt, base, tool, dst, std::move(tool_world), pid_map, "sew");
    return dst;
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

// Build a drill-hole cutting tool (cylinder + conical drill tip) at `pt`
// on `base`. ZW3D's FtHoleMain exports the placement POINT but not the
// drill axis, so the axis is derived here from the base face nearest pt:
// the inward normal -- the direction that goes INTO solid material. The
// caller Cuts the returned tool from base. radius = dia/2; cylinder length
// = depth (or the bbox diagonal when `through`); the tip is a cone whose
// half-angle = tip_deg/2 (118 deg is the standard twist-drill point), sized
// so its base radius matches the bore and it sits just below the cylinder
// bottom -- this reproduces the truth hole volume exactly (verified on
// DKBA81377750 孔1: cylinder 166.3 + tip 5.8 = 172.1 mm^3 vs truth 172).
// Returns null when no face is found near pt or the normal is undefined,
// so the caller can fall back to a no-op (body without the hole) rather
// than emit garbage.
std::shared_ptr<TopoShape> TopoAlgo::DrillTool(const std::shared_ptr<TopoShape>& base,
                                               double px, double py, double pz,
                                               double dia, double depth, double tip_deg, bool through,
                                               uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                               const std::shared_ptr<brepdb::VersionTree>& vt)
{
    if (!base || dia <= 0.0) {
        return nullptr;
    }
    const TopoDS_Shape& bs = base->GetShape();
    if (bs.IsNull()) {
        return nullptr;
    }
    const gp_Pnt   P(px, py, pz);
    const TopoDS_Vertex V = BRepBuilderAPI_MakeVertex(P);

    // Pick the base face whose TRIMMED area is nearest pt (a hole is placed
    // ON a face). DistShapeShape respects face trimming, so a point above a
    // small pocket face beats the larger surface it lies within.
    TopoDS_Face best;
    double      bestD = 1e300;
    for (TopExp_Explorer fx(bs, TopAbs_FACE); fx.More(); fx.Next()) {
        const TopoDS_Face& f = TopoDS::Face(fx.Current());
        BRepExtrema_DistShapeShape d(V, f);
        if (!d.IsDone() || d.NbSolution() < 1) {
            continue;
        }
        const double dv = d.Value();
        if (dv < bestD) {
            bestD = dv;
            best  = f;
        }
    }
    if (best.IsNull()) {
        return nullptr;
    }

    // Outward normal at the projection of pt onto the chosen face's surface.
    gp_Dir nrm;
    {
        Handle(Geom_Surface) surf = BRep_Tool::Surface(best);
        if (surf.IsNull()) {
            return nullptr;
        }
        GeomAPI_ProjectPointOnSurf proj(P, surf);
        double u = 0.0, v = 0.0;
        if (proj.NbPoints() > 0) {
            proj.LowerDistanceParameters(u, v);
        }
        GeomLProp_SLProps props(surf, u, v, 1, 1e-7);
        if (!props.IsNormalDefined()) {
            return nullptr;
        }
        nrm = props.Normal();
        if (best.Orientation() == TopAbs_REVERSED) {
            nrm.Reverse();
        }
    }

    // Inward axis = the direction that lands INSIDE the solid. Probe a hair
    // along -normal; if that is not IN material, the body sits on the other
    // side, so drill along +normal instead.
    gp_Dir axis = nrm.Reversed();
    {
        const gp_Pnt probe = P.Translated(gp_Vec(axis) * 1.0e-3);
        BRepClass3d_SolidClassifier cls(bs, probe, 1.0e-7);
        if (cls.State() != TopAbs_IN) {
            axis = nrm;
        }
    }

    const double r = 0.5 * dia;
    double       H = depth;
    if (through || H <= 0.0) {
        Bnd_Box bb;
        BRepBndLib::Add(bs, bb);
        double xmin, ymin, zmin, xmax, ymax, zmax;
        bb.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        const double diag = gp_Pnt(xmin, ymin, zmin).Distance(gp_Pnt(xmax, ymax, zmax));
        H = (diag > 0.0 ? diag : 1.0) * 1.1;
    }

    TopoDS_Shape cyl;
    try {
        cyl = BRepPrimAPI_MakeCylinder(gp_Ax2(P, axis), r, H).Shape();
    } catch (Standard_Failure&) {
        return nullptr;
    }
    if (cyl.IsNull()) {
        return nullptr;
    }

    // Conical drill point below the cylinder bottom (skipped for a through
    // hole, whose far end exits the part).
    if (!through && tip_deg > 0.0 && tip_deg < 180.0) {
        const double pi   = 3.14159265358979323846;
        const double half = 0.5 * tip_deg * pi / 180.0;
        const double tan_h = std::tan(half);
        if (tan_h > 1e-6) {
            const double tipH = r / tan_h;
            const gp_Pnt bottom = P.Translated(gp_Vec(axis) * H);
            TopoDS_Shape cone;
            try {
                cone = BRepPrimAPI_MakeCone(gp_Ax2(bottom, axis), r, 0.0, tipH).Shape();
            } catch (Standard_Failure&) {
                cone.Nullify();
            }
            if (!cone.IsNull()) {
                auto cylS  = std::make_shared<brepkit::TopoShape>(cyl);
                auto coneS = std::make_shared<brepkit::TopoShape>(cone);
                auto fused = Fuse(cylS, coneS, op_id, tn, vt);
                if (fused && !fused->GetShape().IsNull()) {
                    return fused;
                }
            }
        }
    }
    return std::make_shared<brepkit::TopoShape>(cyl);
}

}
