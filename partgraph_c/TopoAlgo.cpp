#include "TopoAlgo.h"
#include "TopoShape.h"
#include "occt_adapter.h"

#include "BRepHistory.h"
#include "BRepBuilder.h"
#include "breptopo_c/TopoNaming.h"
#include "brepdb_c/GeomSender.h"
#include "brepdb_c/GeomPool.h"
#include "brepdb_c/VersionTree.h"

// OCCT
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

namespace
{

void commit_to_vt(const std::shared_ptr<breptopo::TopoNaming>& tn,
                  const std::shared_ptr<brepdb::VersionTree>& vt,
                  const TopoDS_Shape& shape,
                  const breptopo::TopoNaming::PidMap& pid_map,
                  const std::string& op_name)
{
    if (!tn || !vt) {
        return;
    }
    brepdb::GeomSender sender(tn);
    brepdb::GeometryPool new_pool;
    sender.Serialize(shape, new_pool);
    vt->Commit(new_pool, pid_map, op_name);
}

}

namespace partgraph
{

std::shared_ptr<TopoShape> TopoAlgo::Fillet(const std::shared_ptr<TopoShape>& shape, double radius,
                                            const std::vector<std::shared_ptr<TopoShape>>& edges, uint32_t op_id,
                                            const std::shared_ptr<breptopo::TopoNaming>& tn,
                                            const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepFilletAPI_MakeFillet fillet(shape->GetShape());

    if (edges.empty())
    {
        TopExp_Explorer edge_explorer(shape->GetShape(), TopAbs_EDGE);
        while (edge_explorer.More())
        {
            TopoDS_Edge edge = TopoDS::Edge(edge_explorer.Current());
            fillet.Add(radius, edge);
            edge_explorer.Next();
        }
    }
    else
    {
        for (auto& edge : edges) {
            fillet.Add(radius, edge->ToEdge());
        }
    }

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(fillet, fillet.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<partgraph::TopoShape>(fillet.Shape());
    commit_to_vt(tn, vt, dst->GetShape(), pid_map, "fillet");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Chamfer(const std::shared_ptr<TopoShape>& shape, double dist,
                                             const std::vector<std::shared_ptr<TopoShape>>& edges, uint32_t op_id,
                                             const std::shared_ptr<breptopo::TopoNaming>& tn,
                                             const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepFilletAPI_MakeChamfer chamfer(shape->GetShape());

    if (edges.empty())
    {
        TopExp_Explorer edge_explorer(shape->GetShape(), TopAbs_EDGE);
        while (edge_explorer.More())
        {
            TopoDS_Edge edge = TopoDS::Edge(edge_explorer.Current());
            chamfer.Add(dist, edge);
            edge_explorer.Next();
        }
    }
    else
    {
        for (auto& edge : edges) {
            chamfer.Add(dist, edge->ToEdge());
        }
    }

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(chamfer, chamfer.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<partgraph::TopoShape>(chamfer.Shape());
    commit_to_vt(tn, vt, dst->GetShape(), pid_map, "chamfer");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Prism(const std::shared_ptr<TopoShape>& face, double x, double y, double z,
                                           uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                           const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepPrimAPI_MakePrism prism(face->GetShape(), gp_Vec(x, y, z));

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(prism, prism.Shape(), face->GetShape(), op_id);
    }
    auto dst = std::make_shared<partgraph::TopoShape>(prism.Shape());
    commit_to_vt(tn, vt, dst->GetShape(), pid_map, "prism");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Split(const std::shared_ptr<TopoShape>& base, const std::shared_ptr<TopoShape>& tool,
                                           uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
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

    breptopo::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = BRepBuilder::MakeCompound({ base, tool });
        opencascade::handle<BRepTools_History> o_hist = algo.History();
        pid_map = tn->Update(o_hist, algo.Shape(), old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<partgraph::TopoShape>(algo.Shape());
    commit_to_vt(tn, vt, dst->GetShape(), pid_map, "split");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Cut(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
                                         uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
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

    breptopo::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = BRepBuilder::MakeCompound({ s1, s2 });
        opencascade::handle<BRepTools_History> o_hist = algo.History();
        pid_map = tn->Update(o_hist, algo.Shape(), old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<partgraph::TopoShape>(algo.Shape());
    commit_to_vt(tn, vt, dst->GetShape(), pid_map, "cut");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Fuse(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
                                          uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                          const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepAlgoAPI_Fuse algo(s1->GetShape(), s2->GetShape());
    algo.Build();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cout);
    }
    if (algo.HasWarnings()) {
        algo.DumpErrors(std::cout);
    }

    breptopo::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = BRepBuilder::MakeCompound({ s1, s2 });
        opencascade::handle<BRepTools_History> o_hist = algo.History();
        pid_map = tn->Update(o_hist, algo.Shape(), old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<partgraph::TopoShape>(algo.Shape());
    commit_to_vt(tn, vt, dst->GetShape(), pid_map, "fuse");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Common(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
                                            uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
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

    breptopo::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = BRepBuilder::MakeCompound({ s1, s2 });
        opencascade::handle<BRepTools_History> o_hist = algo.History();
        pid_map = tn->Update(o_hist, algo.Shape(), old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<partgraph::TopoShape>(algo.Shape());
    commit_to_vt(tn, vt, dst->GetShape(), pid_map, "common");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Section(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
                                             uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
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

    breptopo::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = BRepBuilder::MakeCompound({ s1, s2 });
        opencascade::handle<BRepTools_History> o_hist = algo.History();
        pid_map = tn->Update(o_hist, algo.Shape(), old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<partgraph::TopoShape>(algo.Shape());
    commit_to_vt(tn, vt, dst->GetShape(), pid_map, "section");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Sew(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
    uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
    const std::shared_ptr<brepdb::VersionTree>& vt)
{
    Standard_Real tolerance = 1e-6;
    BRepBuilderAPI_Sewing algo(tolerance);

    algo.Add(s1->GetShape());
    algo.Add(s2->GetShape());

    algo.Perform();

    //if (tn)
    //{
    //    auto old_shp = BRepBuilder::MakeCompound({ s1, s2 });
    //    opencascade::handle<BRepTools_History> o_hist = algo.History();
    //    auto upd_hist_graph = [&](const std::shared_ptr<breptopo::HistGraph>& hg)
    //    {
    //        if (!hg) {
    //            return;
    //        }
    //        auto type = trans_type(hg->GetType());
    //        BRepHistory hist(o_hist, type, algo.Shape(), s1->GetShape());
    //        hg->Update(hist, op_id);
    //    };
    //    upd_hist_graph(tn->GetEdgeGraph());
    //    upd_hist_graph(tn->GetFaceGraph());
    //    upd_hist_graph(tn->GetSolidGraph());
    //}

    auto shp = algo.SewedShape();
    auto type = shp.ShapeType();

    return std::make_shared<partgraph::TopoShape>(algo.SewedShape());
}

std::shared_ptr<TopoShape> TopoAlgo::UnifySameDomain(const std::shared_ptr<TopoShape>& shape,
                                                     uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                                     const std::shared_ptr<brepdb::VersionTree>& vt)
{
    ShapeUpgrade_UnifySameDomain algo;
    algo.Initialize(shape->GetShape(), Standard_True, Standard_True, Standard_False);
    algo.Build();

    breptopo::TopoNaming::PidMap pid_map;
    if (tn)
    {
        opencascade::handle<BRepTools_History> o_hist = algo.History();
        pid_map = tn->Update(o_hist, algo.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<partgraph::TopoShape>(algo.Shape());
    commit_to_vt(tn, vt, dst->GetShape(), pid_map, "unify_same_domain");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Translate(const std::shared_ptr<TopoShape>& shape, double x, double y, double z,
                                               uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                               const std::shared_ptr<brepdb::VersionTree>& vt)
{
    gp_Trsf trsf;
    trsf.SetTranslation(gp_Vec(x, y, z));
    auto trans = BRepBuilderAPI_Transform(shape->GetShape(), trsf, Standard_True);

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(trans, trans.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<partgraph::TopoShape>(trans.Shape());
    commit_to_vt(tn, vt, dst->GetShape(), pid_map, "translate");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Mirror(const std::shared_ptr<TopoShape>& shape, const sm::vec3& pos, const sm::vec3& dir,
                                            uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                            const std::shared_ptr<brepdb::VersionTree>& vt)
{
    gp_Ax1 axis;
    axis.SetLocation(trans_pnt(pos));
    axis.SetDirection(trans_vec(dir));

    gp_Trsf trsf;
    trsf.SetMirror(axis);

    auto trans = BRepBuilderAPI_Transform(shape->GetShape(), trsf, Standard_True);

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(trans, trans.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<partgraph::TopoShape>(trans.Shape());
    commit_to_vt(tn, vt, dst->GetShape(), pid_map, "mirror");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Draft(const std::shared_ptr<TopoShape>& shape,
                                           const sm::vec3& dir, float angle, float len_max,
                                           uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                           const std::shared_ptr<brepdb::VersionTree>& vt)
{
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape->GetShape(), TopAbs_FACE, faces);

    const auto& face = TopoDS::Face(faces.FindKey(1));

    BRepOffsetAPI_MakeDraft draft(face, trans_vec(dir), angle);
    draft.Perform(len_max);

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(draft, draft.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<partgraph::TopoShape>(draft.Shape());
    commit_to_vt(tn, vt, dst->GetShape(), pid_map, "draft");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::ThickSolid(const std::shared_ptr<TopoShape>& shape, const std::vector<std::shared_ptr<TopoShape>>& faces, float offset,
                                                uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                                const std::shared_ptr<brepdb::VersionTree>& vt)
{
    TopTools_ListOfShape faces_to_rm;
    for (auto& face : faces) {
        faces_to_rm.Append(face->GetShape());
    }

    BRepOffsetAPI_MakeThickSolid thick_solid;
    thick_solid.MakeThickSolidByJoin(shape->GetShape(), faces_to_rm, offset, 1.e-3);

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(thick_solid, thick_solid.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<partgraph::TopoShape>(thick_solid.Shape());
    commit_to_vt(tn, vt, dst->GetShape(), pid_map, "thick_solid");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::ThruSections(const std::vector<std::shared_ptr<TopoShape>>& wires,
                                                  uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                                  const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepOffsetAPI_ThruSections thru_sections(Standard_False);
    for (auto& wire : wires) {
        thru_sections.AddWire(wire->ToWire());
    }

    breptopo::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = BRepBuilder::MakeCompound(wires);
        pid_map = tn->Update(thru_sections, thru_sections.Shape(), old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<partgraph::TopoShape>(thru_sections.Shape());
    commit_to_vt(tn, vt, dst->GetShape(), pid_map, "thru_sections");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::OffsetShape(const std::shared_ptr<TopoShape>& shape, float offset, bool is_solid,
                                                 uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                                 const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepOffset_MakeSimpleOffset builder;
    builder.Initialize(shape->GetShape(), offset);
    builder.SetBuildSolidFlag(is_solid);
    builder.Perform();

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(builder, shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<partgraph::TopoShape>(builder.GetResultShape());
    commit_to_vt(tn, vt, dst->GetShape(), pid_map, "offset_shape");
    return dst;
}

}
