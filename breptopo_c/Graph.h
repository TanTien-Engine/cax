#pragma once

#include <graph/Graph.h>

namespace partgraph 
{ 
	class TopoShape; 
	class TopoFace; 
}

namespace breptopo
{

class Node;

class Graph : public graph::Graph
{
public:
	Graph() {}
	Graph(const std::vector<std::shared_ptr<partgraph::TopoShape>>& shapes)
		: m_shapes(shapes) {}

private:
	std::vector<std::shared_ptr<partgraph::TopoShape>> m_shapes;

}; // Graph

}