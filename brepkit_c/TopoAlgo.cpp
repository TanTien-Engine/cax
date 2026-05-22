#include "TopoAlgo.h"
#include "TopoShape.h"
#include "occt_adapter.h"

#include "ShapeBuilder.h"
#include "brepgraph_c/history/TopoNaming.h"
#include "brepdb_c/WorldSender.h"
#include "brepdb_c/VersionTree.h"

#include <cstdio>
#include <cstdlib>

// OCCT
#include <OSD.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepOffsetAPI_MakeDraft.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_Failure.hxx>
#include <gp_Ax2.hxx>

namespace
{

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
    if (leaf_edges.empty()) {
        // All refs failed to resolve, or no real edges. Skip the
        // op rather than calling fillet.Shape(), which would throw
        // StdFail_NotDone when NbContours()==0.
        return shape;
    }

    // ---- Batch attempt: add all edges in one MakeFillet call ----
    BRepFilletAPI_MakeFillet batch(shape->GetShape());
    bool         batch_ok = false;
    TopoDS_Shape result;
    if (!force_single)
    {
        for (const auto& e : leaf_edges) {
            batch.Add(radius, e);
        }
        try {
            batch.Build();
            if (batch.IsDone()) {
                result   = batch.Shape();
                batch_ok = true;
            }
        } catch (...) {
            // OCCT throws Standard_Failure subclasses on numeric
            // pathologies, and SEH (converted by OSD::SetSignal)
            // when ChFi3d_StripeEdgeInter dereferences something
            // it shouldn't. Either way: bail to the per-edge path.
            batch_ok = false;
        }
    }

    if (batch_ok)
    {
        result = UnwrapSingleSolid(result);
        brepgraph::TopoNaming::PidMap pid_map;
        if (tn) {
            pid_map = tn->Update(batch, result, shape->GetShape(), op_id);
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
    std::fprintf(stderr,
        "[FILLET] op_id=%u batch failed, retrying %zu edges singly\n",
        op_id, leaf_edges.size());

    TopoDS_Shape running = shape->GetShape();
    int          ok_count = 0;
    for (size_t i = 0; i < leaf_edges.size(); ++i)
    {
        try {
            BRepFilletAPI_MakeFillet f(running);
            f.Add(radius, leaf_edges[i]);
            f.Build();
            if (f.IsDone()) {
                running = UnwrapSingleSolid(f.Shape());
                ++ok_count;
                std::fprintf(stderr, "[FILLET]   edge[%zu] ok\n", i);
            } else {
                std::fprintf(stderr, "[FILLET]   edge[%zu] IsDone=false\n", i);
            }
        } catch (...) {
            std::fprintf(stderr, "[FILLET]   edge[%zu] threw\n", i);
        }
    }
    std::fprintf(stderr,
        "[FILLET] op_id=%u per-edge retry: %d/%zu applied\n",
        op_id, ok_count, leaf_edges.size());

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
    BRepFilletAPI_MakeChamfer chamfer(shape->GetShape());

    if (edges.empty())
    {
        for (TopExp_Explorer ex(shape->GetShape(), TopAbs_EDGE); ex.More(); ex.Next())
            chamfer.Add(dist, TopoDS::Edge(ex.Current()));
    }
    else
    {
        for (auto& edge : edges) {
            if (!edge) continue;
            if (edge->GetShape().ShapeType() == TopAbs_EDGE) {
                chamfer.Add(dist, TopoDS::Edge(edge->GetShape()));
            } else {
                for (TopExp_Explorer ex(edge->GetShape(), TopAbs_EDGE); ex.More(); ex.Next())
                    chamfer.Add(dist, TopoDS::Edge(ex.Current()));
            }
        }
    }

    // Symmetric to Fillet: empty result set means there's nothing
    // to chamfer (all refs unresolved); skip cleanly.
    if (chamfer.NbContours() == 0) {
        return shape;
    }

    TopoDS_Shape result;
    try {
        chamfer.Build();
        if (!chamfer.IsDone()) {
            return shape;
        }
        result = chamfer.Shape();
    } catch (const Standard_Failure&) {
        return shape;
    }

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(chamfer, result, shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(result);
    commit_to_vt(tn, vt, shape, dst, pid_map, "chamfer");
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
        algo.DumpErrors(std::cout);
    }
    if (algo.HasWarnings()) {
        algo.DumpErrors(std::cout);
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
    BRepAlgoAPI_Cut algo(s1->GetShape(), s2->GetShape());
    algo.Build();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cout);
    }
    if (algo.HasWarnings()) {
        algo.DumpErrors(std::cout);
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
    BRepAlgoAPI_Fuse algo;
    TopTools_ListOfShape args;  args.Append(s1->GetShape());
    TopTools_ListOfShape tools; tools.Append(s2->GetShape());
    algo.SetArguments(args);
    algo.SetTools(tools);
    algo.SetFuzzyValue(1e-6);
    algo.Build();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cerr);
    }
    if (algo.HasWarnings()) {
        algo.DumpWarnings(std::cerr);
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
    merge_to_vt(tn, vt, s1, s2, dst, std::move(tool_world), pid_map, "fuse");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Common(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
                                            uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                            const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepAlgoAPI_Common algo(s1->GetShape(), s2->GetShape());
    //algo.Build();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cout);
    }
    if (algo.HasWarnings()) {
        algo.DumpErrors(std::cout);
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
        algo.DumpErrors(std::cout);
    }
    if (algo.HasWarnings()) {
        algo.DumpErrors(std::cout);
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

    BRepOffsetAPI_MakeThickSolid thick_solid;
    thick_solid.MakeThickSolidByJoin(shape->GetShape(), faces_to_rm, offset, 1.e-3);

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(thick_solid, thick_solid.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(thick_solid.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "thick_solid");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::ThruSections(const std::vector<std::shared_ptr<TopoShape>>& wires,
                                                  uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn,
                                                  const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepOffsetAPI_ThruSections thru_sections(Standard_False);
    for (auto& wire : wires) {
        thru_sections.AddWire(wire->ToWire());
    }

    brepgraph::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = ShapeBuilder::MakeCompound(wires);
        pid_map = tn->Update(thru_sections, thru_sections.Shape(), old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(thru_sections.Shape());
    auto dummy_src = std::make_shared<brepkit::TopoShape>();
    commit_to_vt(tn, vt, dummy_src, dst, pid_map, "thru_sections");
    return dst;
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
