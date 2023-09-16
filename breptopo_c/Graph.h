#pragma once

#include <vector>
#include <memory>

namespace partgraph 
{ 
	class TopoShape; 
	class TopoFace; 
}

namespace breptopo
{

class Node;

class Graph
{
public:
	Graph() {}
	Graph(const std::shared_ptr<partgraph::TopoShape>& shape);

	std::shared_ptr<Node> AddNode(const std::shared_ptr<partgraph::TopoFace>& face);
	void AddEdge(size_t f_node, size_t t_node);

	auto& GetNodes() const { return m_nodes; }

private:
	std::shared_ptr<partgraph::TopoShape> m_shape;

	std::vector<std::shared_ptr<Node>> m_nodes;

}; // Graph

}