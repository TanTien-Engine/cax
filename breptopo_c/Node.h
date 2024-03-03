#pragma once

#include <graph/Node.h>

namespace partgraph { class TopoFace; }

namespace breptopo
{

class Node : public graph::Node
{
public:
	Node() {}
	Node(const std::shared_ptr<partgraph::TopoFace>& face)
		: m_face(face) {}

	auto GetFace() const { return m_face; }

private:
	std::shared_ptr<partgraph::TopoFace> m_face;

}; // Node

}