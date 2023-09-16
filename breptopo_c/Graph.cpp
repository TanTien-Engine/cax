#include "Graph.h"
#include "Node.h"
#include "Edge.h"

namespace breptopo
{

Graph::Graph(const std::shared_ptr<partgraph::TopoShape>& shape)
	: m_shape(shape)
{
}

std::shared_ptr<Node> Graph::AddNode(const std::shared_ptr<partgraph::TopoFace>& face)
{
	auto node = std::make_shared<Node>(face);
	m_nodes.push_back(node);
	return node;
}

void Graph::AddEdge(size_t f_node, size_t t_node)
{
	if (f_node < m_nodes.size() && t_node < m_nodes.size()) 
	{
		m_nodes[f_node]->AddConnect(m_nodes[t_node]);
	}
}

}