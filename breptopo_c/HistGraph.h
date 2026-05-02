#pragma once

#include <TopoDS_Shape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <NCollection_DataMap.hxx>

#include <memory>
#include <vector>
#include <map>

namespace partgraph { class TopoShape; class BRepHistory; }
namespace graph { class Graph; class Node; }

namespace breptopo
{

class HistGraph
{
public:
	HistGraph();

	void Update(const partgraph::BRepHistory& hist, uint32_t type_id, uint32_t op_id);

	const std::shared_ptr<graph::Node> QueryNode(const std::shared_ptr<partgraph::TopoShape>& shape) const;
	bool QueryNodes(uint32_t uid, std::vector<std::shared_ptr<graph::Node>>& results) const;

	auto GetGraph() { return m_graph; }

private:
	void InitDelNode();

	void CreateGraph(const partgraph::BRepHistory& hist, uint32_t type_id, uint32_t op_id);
	void UpdateGraph(const partgraph::BRepHistory& hist, uint32_t type_id, uint32_t op_id,
		const std::vector<size_t>& old_nodes);

	static uint32_t CalcUID(uint32_t type_id, uint32_t op_id, uint32_t index);

private:
	std::shared_ptr<graph::Graph> m_graph;

	// map shape to gid
	NCollection_DataMap<TopoDS_Shape, size_t, TopTools_ShapeMapHasher> m_curr_shapes;

	std::map<uint32_t, size_t> m_uid2gid;

	size_t m_del_node_idx;
	const graph::Node* m_del_node;

	// map op id to graph node gid
	std::map<uint32_t, std::vector<size_t>> m_op2nodes;

}; // HistGraph

}