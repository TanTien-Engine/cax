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
	Graph(const std::shared_ptr<partgraph::TopoShape>& shape) 
		: m_shape(shape) {}

private:
	std::shared_ptr<partgraph::TopoShape> m_shape;

}; // Graph

}