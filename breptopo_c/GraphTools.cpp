#include "GraphTools.h"
#include "Graph.h"
#include "Node.h"

#include <SM_Calc.h>

#include <random>

namespace breptopo
{

void GraphTools::Layout(const Graph& graph)
{
	auto& nodes = graph.GetNodes();
	for (auto& node : nodes)
	{
		float x = static_cast<float>(rand()) / RAND_MAX;
		float y = static_cast<float>(rand()) / RAND_MAX;
		node->SetPos({ x, y });
	}
}

std::shared_ptr<Node> GraphTools::QueryNode(const Graph& graph, const sm::vec2& pos)
{
	auto& nodes = graph.GetNodes();
	for (auto& node : nodes)
	{
		float dist = sm::dis_pos_to_pos(pos, node->GetPos());
		if (dist < FLT_EPSILON)
		{
			return node;
		}
	}

	return nullptr;
}

}