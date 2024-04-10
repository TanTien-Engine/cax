#pragma once

#include <memory>
#include <string>

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

	auto GetGraph() const { return m_graph; }

private:
	std::shared_ptr<graph::Graph> m_graph;

}; // CompGraph

}