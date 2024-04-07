#pragma once

#include <memory>

namespace graph { class Graph; class Node; }

namespace breptopo
{

class CompNode;

class CompGraph
{
public:
	CompGraph();

	int AddNode(const std::shared_ptr<CompNode>& node);
	void AddEdge(size_t f_node, size_t t_node);

	auto GetGraph() const { return m_graph; }

private:
	std::shared_ptr<graph::Graph> m_graph;

}; // CompGraph

}