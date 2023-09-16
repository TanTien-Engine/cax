#pragma once

#include "Graph.h"

#include <memory>

namespace partgraph { class TopoShape; }

namespace breptopo
{

class Graph;

class TopoAdapter
{
public:
	static std::shared_ptr<Graph> BuildGraph(const std::shared_ptr<partgraph::TopoShape>& shape);

}; // TopoAdapter

}