#include "TopoAlgo.h"
#include "TopoShape.h"
#include "occt_adapter.h"

#include "BRepHistory.h"
#include "BRepBuilder.h"
#include "../breptopo_c/HistMgr.h"

// OCCT
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepOffsetAPI_MakeDraft.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakePrism.hxx>

namespace partgraph
{

std::shared_ptr<TopoShape> TopoAlgo::Fillet(const std::shared_ptr<TopoShape>& shape, double radius, 
                                            const std::vector<std::shared_ptr<TopoShape>>& edges)
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

    //BRepHistory hist(fillet, TopAbs_FACE, fillet.Shape(), shape->GetShape());
    //BRepHistory hist(fillet, TopAbs_EDGE, fillet.Shape(), shape->GetShape());

    return std::make_shared<partgraph::TopoShape>(fillet.Shape());
}

std::shared_ptr<TopoShape> TopoAlgo::Chamfer(const std::shared_ptr<TopoShape>& shape, double dist,
                                             const std::vector<std::shared_ptr<TopoShape>>& edges)
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

    return std::make_shared<partgraph::TopoShape>(chamfer.Shape());
}

std::shared_ptr<TopoShape> TopoAlgo::Prism(const std::shared_ptr<TopoShape>& face, double x, double y, double z)
{
    gp_Vec vec;
    auto prism = BRepPrimAPI_MakePrism(face->GetShape(), gp_Vec(x, y, z));
    return std::make_shared<partgraph::TopoShape>(prism.Shape());
}

std::shared_ptr<TopoShape> TopoAlgo::Cut(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2, 
                                         uint16_t op_id, const std::shared_ptr<breptopo::HistMgr>& hm)
{
    BRepAlgoAPI_Cut algo(s1->GetShape(), s2->GetShape());
    algo.Build();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cout);
    }
    if (algo.HasWarnings()) {
        algo.DumpErrors(std::cout);
    }

    if (hm)
    {
        auto old_shp = BRepBuilder::MakeCompound({ s1, s2 });
        opencascade::handle<BRepTools_History> o_hist = algo.History();
        auto upd_hist_graph = [&](const std::shared_ptr<breptopo::HistGraph>& hg)
        {
            auto type = trans_type(hg->GetType());
            BRepHistory hist(o_hist, type, algo.Shape(), *old_shp);
            hg->Update(hist, op_id);
        };
        upd_hist_graph(hm->GetEdgeGraph());
        upd_hist_graph(hm->GetFaceGraph());
        upd_hist_graph(hm->GetSolidGraph());
    }

    return std::make_shared<partgraph::TopoShape>(algo.Shape());
}

std::shared_ptr<TopoShape> TopoAlgo::Fuse(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2)
{
    BRepAlgoAPI_Fuse algo(s1->GetShape(), s2->GetShape());
    algo.Build();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cout);
    }
    if (algo.HasWarnings()) {
        algo.DumpErrors(std::cout);
    }

    return std::make_shared<partgraph::TopoShape>(algo.Shape());
}

std::shared_ptr<TopoShape> TopoAlgo::Common(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2)
{
    BRepAlgoAPI_Common algo(s1->GetShape(), s2->GetShape());
    algo.Build();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cout);
    }
    if (algo.HasWarnings()) {
        algo.DumpErrors(std::cout);
    }

    return std::make_shared<partgraph::TopoShape>(algo.Shape());
}

std::shared_ptr<TopoShape> TopoAlgo::Section(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2)
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

    return std::make_shared<partgraph::TopoShape>(algo.Shape());
}

std::shared_ptr<TopoShape> TopoAlgo::Translate(const std::shared_ptr<TopoShape>& shape, double x, double y, double z, 
                                               uint16_t op_id, const std::shared_ptr<breptopo::HistMgr>& hm)
{
    gp_Trsf trsf; 
    trsf.SetTranslation(gp_Vec(x, y, z));
    auto trans = BRepBuilderAPI_Transform(shape->GetShape(), trsf, Standard_True);

    if (hm)
    {
        auto upd_hist_graph = [&](const std::shared_ptr<breptopo::HistGraph>& hg)
        {
            auto type = trans_type(hg->GetType());
            BRepHistory hist(trans, type, trans.Shape(), shape->GetShape());
            hg->Update(hist, op_id);
        };
        upd_hist_graph(hm->GetEdgeGraph());
        upd_hist_graph(hm->GetFaceGraph());
        upd_hist_graph(hm->GetSolidGraph());
    }

    return std::make_shared<partgraph::TopoShape>(trans.Shape());
}

std::shared_ptr<TopoShape> TopoAlgo::Mirror(const std::shared_ptr<TopoShape>& shape, const sm::vec3& pos, const sm::vec3& dir)
{
    gp_Ax1 axis;
    axis.SetLocation(trans_pnt(pos));
    axis.SetDirection(trans_vec(dir));

    gp_Trsf trsf;
    trsf.SetMirror(axis);

    auto trans = BRepBuilderAPI_Transform(shape->GetShape(), trsf, Standard_True);
    return std::make_shared<partgraph::TopoShape>(trans.Shape());
}

std::shared_ptr<TopoShape> TopoAlgo::Draft(const std::shared_ptr<TopoShape>& shape, 
                                           const sm::vec3& dir, float angle, float len_max)
{
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape->GetShape(), TopAbs_FACE, faces);

    const auto& face = TopoDS::Face(faces.FindKey(1));

    BRepOffsetAPI_MakeDraft draft(face, trans_vec(dir), angle);
    draft.Perform(len_max);
    return std::make_shared<partgraph::TopoShape>(draft.Shape());
}

std::shared_ptr<TopoShape> TopoAlgo::ThickSolid(const std::shared_ptr<TopoShape>& shape, const std::vector<std::shared_ptr<TopoShape>>& faces, float offset)
{
    TopTools_ListOfShape faces_to_rm;
    for (auto& face : faces) {
        faces_to_rm.Append(face->GetShape());
    }

    BRepOffsetAPI_MakeThickSolid thick_solid;
    thick_solid.MakeThickSolidByJoin(shape->GetShape(), faces_to_rm, offset, 1.e-3);

    return std::make_shared<partgraph::TopoShape>(thick_solid.Shape());
}

std::shared_ptr<TopoShape> TopoAlgo::ThruSections(const std::vector<std::shared_ptr<TopoShape>>& wires)
{
    BRepOffsetAPI_ThruSections thru_sections(Standard_False);
    for (auto& wire : wires) {
        thru_sections.AddWire(wire->ToWire());
    }

    return std::make_shared<partgraph::TopoShape>(thru_sections.Shape());
}

std::shared_ptr<TopoShape> TopoAlgo::OffsetShape(const std::shared_ptr<TopoShape>& shape, float offset, bool is_solid, 
                                                 uint16_t op_id, const std::shared_ptr<breptopo::HistMgr>& hm)
{
    BRepOffset_MakeSimpleOffset builder;
    builder.Initialize(shape->GetShape(), offset);
    builder.SetBuildSolidFlag(is_solid);
    builder.Perform();

    if (hm)
    {
        BRepHistory hist(builder, shape->GetShape());
        //hm->GetEdgeGraph()->Update(hist, op_id);
        hm->GetFaceGraph()->Update(hist, op_id);
        //hm->GetSolidGraph()->Update(hist, op_id);
    }

    return std::make_shared<partgraph::TopoShape>(builder.GetResultShape());
}

} 