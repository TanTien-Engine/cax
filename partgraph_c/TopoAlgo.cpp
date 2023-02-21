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

#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakeSegment.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>

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

std::shared_ptr<TopoShape> TopoAlgo::Prism(const std::shared_ptr<TopoShape>& shape, double x, double y, double z)
{
    //gp_Vec vec;
    //auto prism = BRepPrimAPI_MakePrism(shape->GetShape(), gp_Vec(x, y, z));
    //return std::make_shared<partgraph::TopoShape>(prism.Shape());

    double myWidth = 1.0, myThickness = 0.5, myHeight = 2.0;

  // Profile : Define Support Points
    gp_Pnt aPnt1(-myWidth / 2., 0, 0);
    gp_Pnt aPnt2(-myWidth / 2., -myThickness / 4., 0);
    gp_Pnt aPnt3(0, -myThickness / 2., 0);
    gp_Pnt aPnt4(myWidth / 2., -myThickness / 4., 0);
    gp_Pnt aPnt5(myWidth / 2., 0, 0);

    // Profile : Define the Geometry
    Handle(Geom_TrimmedCurve) anArcOfCircle = GC_MakeArcOfCircle(aPnt2, aPnt3, aPnt4);
    Handle(Geom_TrimmedCurve) aSegment1 = GC_MakeSegment(aPnt1, aPnt2);
    Handle(Geom_TrimmedCurve) aSegment2 = GC_MakeSegment(aPnt4, aPnt5);

    // Profile : Define the Topology
    TopoDS_Edge anEdge1 = BRepBuilderAPI_MakeEdge(aSegment1);
    TopoDS_Edge anEdge2 = BRepBuilderAPI_MakeEdge(anArcOfCircle);
    TopoDS_Edge anEdge3 = BRepBuilderAPI_MakeEdge(aSegment2);
    TopoDS_Wire aWire = BRepBuilderAPI_MakeWire(anEdge1, anEdge2, anEdge3);

    // Complete Profile
    gp_Ax1 xAxis = gp::OX();
    gp_Trsf aTrsf;

    aTrsf.SetMirror(xAxis);
    BRepBuilderAPI_Transform aBRepTrsf(aWire, aTrsf);
    TopoDS_Shape aMirroredShape = aBRepTrsf.Shape();
    TopoDS_Wire aMirroredWire = TopoDS::Wire(aMirroredShape);

    BRepBuilderAPI_MakeWire mkWire;
    mkWire.Add(aWire);
    //mkWire.Add(aMirroredWire);
    TopoDS_Wire myWireProfile = mkWire.Wire();

    // Body : Prism the Profile
    TopoDS_Face myFaceProfile = BRepBuilderAPI_MakeFace(myWireProfile);
    gp_Vec aPrismVec(0, 0, myHeight);
    TopoDS_Shape myBody = BRepPrimAPI_MakePrism(myFaceProfile, aPrismVec);

    return std::make_shared<partgraph::TopoShape>(myBody);
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