#include "brepgraph_c/topology/TopoGraphBuilder.h"
#include "brepgraph_c/computation/NodeShape.h"
#include "brepgraph_c/topology/GraphShape.h"
#include "brepkit_c/TopoShape.h"

#include <graph/Graph.h>
#include <graph/Node.h>
#include <graph/GraphLayout.h>

// OCCT
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopOpeBRepBuild_Tools.hxx>

#include <set>

namespace brepgraph
{

std::shared_ptr<graph::Graph>
TopoGraphBuilder::BuildGraph(const std::vector<std::shared_ptr<brepkit::TopoShape>>& shapes)
{
    auto graph = std::make_shared<graph::Graph>();
    graph->SetDirected(false);
    graph->AddComponent<GraphShape>(shapes);

    for (auto shape : shapes) {
        AddShapeToGraph(shape, graph);
    }

    graph::GraphLayout::StressMinimization(*graph);

	return graph;
}

void TopoGraphBuilder::AddShapeToGraph(const std::shared_ptr<brepkit::TopoShape>& topo_shape,
                                       const std::shared_ptr<graph::Graph>& graph)
{
	auto& shape = topo_shape->GetShape();

    TopTools_IndexedMapOfShape all_faces;
    TopExp::MapShapes(shape, TopAbs_FACE, all_faces);

    std::vector<std::set<int>> adjacency(all_faces.Extent());

    TopTools_IndexedDataMapOfShapeListOfShape edge_face_map;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_face_map);
    for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next())
    {
        const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());
        const TopTools_ListOfShape& faces = edge_face_map.FindFromKey(edge);

        for (TopTools_ListIteratorOfListOfShape i(faces); i.More(); i.Next())
        {
            int f_idx = all_faces.FindIndex(i.Value());
            for (TopTools_ListIteratorOfListOfShape j(faces); j.More(); j.Next())
            {
                int t_idx = all_faces.FindIndex(j.Value());
                if (f_idx == t_idx) {
                    continue;
                }

                adjacency[f_idx - 1].insert(t_idx - 1);
            }
        }
    }

    const size_t offset_idx = graph->GetNodesNum();

    for (size_t i = 0, n = adjacency.size(); i < n; ++i) 
    {
        int key = static_cast<int>(i) + 1;
        auto node = std::make_shared<graph::Node>();
        node->SetValue(key);

        auto face = TopoDS::Face(all_faces.FindKey(key));
        auto shape = std::make_shared<brepkit::TopoShape>(face);
        node->AddComponent<NodeShape>(shape);

        graph->AddNode(node);
    }
    for (size_t i = 0, n = adjacency.size(); i < n; ++i) {
        for (auto itr : adjacency[i]) {
            graph->AddEdge(offset_idx + i, offset_idx + itr);
        }
    }
}

}