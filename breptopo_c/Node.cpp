#include "Node.h"

namespace breptopo
{

Node::Node(const std::shared_ptr<partgraph::TopoFace>& face)
	: m_face(face)
{
}

void Node::AddConnect(const std::shared_ptr<Node>& conn)
{
	m_conns.push_back(conn);
}

}