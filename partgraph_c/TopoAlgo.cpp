#include "TopoAlgo.h"
#include "TopoDataset.h"

// OCCT
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopExp.hxx>
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

std::shared_ptr<TopoShape> TopoAlgo::Fillet(const std::shared_ptr<TopoShape>& shape, double thickness)
{
    BRepFilletAPI_MakeFillet fillet(shape->GetShape());

    TopExp_Explorer edge_explorer(shape->GetShape(), TopAbs_EDGE);
    while (edge_explorer.More())
    {
        TopoDS_Edge edge = TopoDS::Edge(edge_explorer.Current());
        //Add edge to fillet algorithm
        fillet.Add(thickness / 12., edge);
        edge_explorer.Next();
    }

    //TopTools_IndexedMapOfShape edges;
    //TopExp::MapShapes(shape->GetShape(), TopAbs_EDGE, edges);
    //fillet.Add(thickness / 12., TopoDS::Edge(edges.FindKey(1)));
    //fillet.Add(thickness / 12., TopoDS::Edge(edges.FindKey(3)));
    //fillet.Add(thickness / 12., TopoDS::Edge(edges.FindKey(5)));

    return std::make_shared<partgraph::TopoShape>(fillet.Shape());
}

std::shared_ptr<TopoShape> TopoAlgo::Chamfer(const std::shared_ptr<TopoShape>& shape, double dist)
{
    BRepFilletAPI_MakeChamfer chamfer(shape->GetShape());

    TopExp_Explorer edge_explorer(shape->GetShape(), TopAbs_EDGE);
    while (edge_explorer.More())
    {
        TopoDS_Edge edge = TopoDS::Edge(edge_explorer.Current());
        chamfer.Add(dist, edge);
        edge_explorer.Next();
    }

    return std::make_shared<partgraph::TopoShape>(chamfer.Shape());
}

std::shared_ptr<TopoShape> TopoAlgo::Prism(const std::shared_ptr<TopoFace>& face, double x, double y, double z)
{
    gp_Vec vec;
    auto prism = BRepPrimAPI_MakePrism(face->GetFace(), gp_Vec(x, y, z));
    return std::make_shared<partgraph::TopoShape>(prism.Shape());
}

std::shared_ptr<TopoShape> TopoAlgo::Cut(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2)
{
    BRepAlgoAPI_Cut algo(s1->GetShape(), s2->GetShape());
    algo.Build();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cout);
    }
    if (algo.HasWarnings()) {
        algo.DumpErrors(std::cout);
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

std::shared_ptr<TopoShape> TopoAlgo::Translate(const std::shared_ptr<TopoShape>& shape, double x, double y, double z)
{
    gp_Trsf trsf; 
    trsf.SetTranslation(gp_Vec(x, y, z));
    auto trans = BRepBuilderAPI_Transform(shape->GetShape(), trsf, Standard_True);
    return std::make_shared<partgraph::TopoShape>(trans.Shape());
}

}