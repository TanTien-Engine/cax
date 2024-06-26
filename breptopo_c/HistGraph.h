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
	enum class Type
	{
		Edge,
		Face,
		Solid
	};

public:
	HistGraph(Type type);

	uint16_t NextOpId();
	void Update(const partgraph::BRepHistory& hist, uint16_t op_id);

	std::shared_ptr<graph::Node> QueryNode(const std::shared_ptr<partgraph::TopoShape>& shape) const;
	bool QueryNodes(uint32_t uid, std::vector<std::shared_ptr<graph::Node>>& results) const;

	auto GetGraph() { return m_graph; }

	auto GetType() const { return m_type; }

private:
	void InitDelNode();

	std::shared_ptr<partgraph::TopoShape> TransShape(const TopoDS_Shape& shape) const;

private:
	Type m_type;

	std::shared_ptr<graph::Graph> m_graph;

	uint16_t m_next_op = 0;

	// map shape to gid
	NCollection_DataMap<TopoDS_Shape, size_t, TopTools_ShapeMapHasher> m_curr_shapes;

	std::map<uint32_t, size_t> m_uid2gid;

	size_t m_del_node_idx;
	std::shared_ptr<graph::Node> m_del_node;

}; // HistGraph

}