#pragma once

#include <memory>
#include <vector>

namespace partgraph { class TopoShape; }
namespace graph { class Graph; }

namespace breptopo
{

class TopoGraphBuilder
{
public:
	static std::shared_ptr<graph::Graph>
		BuildGraph(const std::vector<std::shared_ptr<partgraph::TopoShape>>& shapes);

private:
	static void AddShapeToGraph(const std::shared_ptr<partgraph::TopoShape>& shape, 
		const std::shared_ptr<graph::Graph>& graph);

}; // TopoGraphBuilder

}