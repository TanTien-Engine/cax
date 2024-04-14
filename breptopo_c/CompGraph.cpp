#include "CompGraph.h"
#include "NodeComp.h"
#include "NodeInfo.h"
#include "CompNode.h"

#include <graph/Graph.h>
#include <graph/Node.h>

namespace breptopo
{

CompGraph::CompGraph()
{
	m_graph = std::make_shared<graph::Graph>();
}

int CompGraph::AddNode(const std::shared_ptr<CompNode>& cnode, const std::string& desc)
{
	int idx = static_cast<int>(m_graph->GetNodes().size());
	auto node = std::make_shared<graph::Node>(idx);

	uint16_t op_id = CalcOpId(idx, 0);
	cnode->SetOpId(op_id);
	node->AddComponent<NodeComp>(cnode);

	node->AddComponent<NodeInfo>(desc);

	m_graph->AddNode(node);

	return op_id;
}

void CompGraph::AddEdge(size_t f_node, size_t t_node)
{
	m_graph->AddEdge(f_node, t_node);
}

uint16_t CompGraph::CalcOpId(int op_id, int sub_op_id) const
{
	auto key = std::make_pair(op_id, sub_op_id);
	auto itr = m_id_map.find(key);
	if (itr == m_id_map.end())
	{
		uint16_t val = static_cast<uint16_t>(m_id_map.size());
		m_id_map.insert({ key, val });
		return val;
	}
	else
	{
		return itr->second;
	}
}

}