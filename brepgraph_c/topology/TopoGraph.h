#pragma once

#include <memory>
#include <vector>

namespace brepkit { class TopoShape; }
namespace graph { class Graph; }

namespace brepgraph
{

class TopoGraph
{
public:
	TopoGraph(const std::vector<std::shared_ptr<brepkit::TopoShape>>& shapes);

	auto GetGraph() { return m_graph; }

private:
	std::shared_ptr<graph::Graph> m_graph;

}; // TopoGraph

}