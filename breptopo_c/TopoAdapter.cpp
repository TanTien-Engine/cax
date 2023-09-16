#include "TopoAdapter.h"
#include "Graph.h"
#include "GraphTools.h"
#include "../partgraph_c/TopoDataset.h"

// OCCT
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopOpeBRepBuild_Tools.hxx>

#include <set>

namespace breptopo
{

std::shared_ptr<Graph> TopoAdapter::BuildGraph(const std::shared_ptr<partgraph::TopoShape>& topo_shape)
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

    auto graph = std::make_shared<Graph>(topo_shape);

    for (size_t i = 0, n = adjacency.size(); i < n; ++i) 
    {
        auto face = TopoDS::Face(all_faces.FindKey(int(i) + 1));
        graph->AddNode(std::make_shared<partgraph::TopoFace>(face));
    }
    for (size_t i = 0, n = adjacency.size(); i < n; ++i) {
        for (auto itr : adjacency[i]) {
            graph->AddEdge(i, itr);
        }
    }

    GraphTools::Layout(*graph);

	return graph;
}

}