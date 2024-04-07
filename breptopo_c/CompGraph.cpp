#include "CompGraph.h"
#include "NodeComp.h"

#include <graph/Graph.h>
#include <graph/Node.h>

namespace breptopo
{

CompGraph::CompGraph()
{
	m_graph = std::make_shared<graph::Graph>();
}

int CompGraph::AddNode(const std::shared_ptr<CompNode>& cnode)
{
	int id = static_cast<int>(m_graph->GetNodes().size());
	auto node = std::make_shared<graph::Node>(id);

	node->AddComponent<NodeComp>(cnode);

	m_graph->AddNode(node);

	return id;
}

void CompGraph::AddEdge(size_t f_node, size_t t_node)
{
	m_graph->AddEdge(f_node, t_node);
}

}