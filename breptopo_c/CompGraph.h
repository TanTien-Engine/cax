#pragma once

#include <memory>
#include <string>
#include <map>

namespace graph { class Graph; class Node; }

namespace breptopo
{

class CompNode;

class CompGraph
{
public:
	CompGraph();

	int AddNode(const std::shared_ptr<CompNode>& node, const std::string& desc);
	void AddEdge(size_t f_node, size_t t_node);
	void RemoveEdge(size_t f_node, size_t t_node);

	size_t GetNodesNum() const;

	std::shared_ptr<graph::Node> GetNode(size_t idx) const;

	uint16_t CalcOpId(int op_id, int sub_op_id) const;

	void Layout();

	auto GetGraph() const { return m_graph; }

private:
	std::shared_ptr<graph::Graph> m_graph;

	// op_id + sub_op_id -> unique_op_id
	mutable std::map<std::pair<int, int>, uint16_t> m_id_map;

}; // CompGraph

}