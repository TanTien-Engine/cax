#pragma once

#include <SM_Vector.h>

#include <memory>

namespace breptopo
{

class Graph;
class Node;

class GraphTools
{
public:
	static void Layout(const Graph& graph);

	static std::shared_ptr<Node> QueryNode(const Graph& graph, const sm::vec2& pos);

}; // GraphTools

}