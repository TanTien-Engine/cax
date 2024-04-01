#pragma once

#include <TopoDS_Shape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <NCollection_DataMap.hxx>

#include <memory>
#include <vector>
#include <map>

namespace partgraph { class TopoShape; class TopoFace; class BRepHistory; }
namespace graph { class Graph; class Node; }

namespace breptopo
{

class HistGraph
{
public:
	HistGraph();

	void Update(const std::shared_ptr<partgraph::TopoFace>& from, 
		const std::vector<std::shared_ptr<partgraph::TopoFace>>& to);

	void Update(const partgraph::BRepHistory& hist);

	std::shared_ptr<graph::Node> QueryNode(const std::shared_ptr<partgraph::TopoShape>& shape) const;
	std::shared_ptr<graph::Node> QueryNode(uint32_t uid) const;

	auto GetGraph() { return m_graph; }

private:
	std::shared_ptr<graph::Graph> m_graph;

	int m_time = 0;

	// map shape to gid
	NCollection_DataMap<TopoDS_Shape, size_t, TopTools_ShapeMapHasher> m_curr_shapes;

	std::map<uint32_t, size_t> m_uid2gid;

}; // HistGraph

}