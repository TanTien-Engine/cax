#pragma once

#include <memory>

namespace graph { class Graph; }

namespace brepgraph
{

class CompGraph;

class CompGraphBuilder
{
public:
	static std::shared_ptr<graph::Graph> BuildGraph(const CompGraph& cg, int root_step = -1);

}; // CompGraphBuilder

}
