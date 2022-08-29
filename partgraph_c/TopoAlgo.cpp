#include "TopoAlgo.h"
#include "TopoShape.h"

// OCCT
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>

namespace partgraph
{

std::shared_ptr<TopoShape> TopoAlgo::Fillet(const std::shared_ptr<TopoShape>& shape, double thickness)
{
    BRepFilletAPI_MakeFillet fillet(shape->GetShape());

    //TopExp_Explorer edge_explorer(src, TopAbs_EDGE);
    //while (edge_explorer.More())
    //{
    //    TopoDS_Edge edge = TopoDS::Edge(edge_explorer.Current());
    //    //Add edge to fillet algorithm
    //    fillet.Add(thickness / 12., edge);
    //    edge_explorer.Next();
    //}

    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(shape->GetShape(), TopAbs_EDGE, edges);
    fillet.Add(thickness / 12., TopoDS::Edge(edges.FindKey(1)));
    fillet.Add(thickness / 12., TopoDS::Edge(edges.FindKey(3)));
    fillet.Add(thickness / 12., TopoDS::Edge(edges.FindKey(5)));

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
    chamfer.Build();

    return std::make_shared<partgraph::TopoShape>(chamfer.Shape());
}

}