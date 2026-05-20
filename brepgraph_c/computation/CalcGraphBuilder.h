#pragma once

#include <memory>

namespace graph { class Graph; }

namespace brepgraph
{

class CalcGraph;

class CalcGraphBuilder
{
public:
	static std::shared_ptr<graph::Graph> BuildGraph(const CalcGraph& cg, int root_step = -1);

}; // CalcGraphBuilder

}
