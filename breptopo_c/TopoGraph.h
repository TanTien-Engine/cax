#pragma once

#include <memory>
#include <vector>

namespace partgraph { class TopoShape; }
namespace graph { class Graph; }

namespace breptopo
{

class TopoGraph
{
public:
	TopoGraph(const std::vector<std::shared_ptr<partgraph::TopoShape>>& shapes);

	auto GetGraph() { return m_graph; }

private:
	std::shared_ptr<graph::Graph> m_graph;

}; // TopoGraph

}