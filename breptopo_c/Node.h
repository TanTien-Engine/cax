#pragma once

#include <SM_Vector.h>

#include <memory>
#include <vector>

namespace partgraph { class TopoFace; }

namespace breptopo
{

class Node
{
public:
	Node() {}
	Node(const std::shared_ptr<partgraph::TopoFace>& face);

	void AddConnect(const std::shared_ptr<Node>& conn);
	auto& GetConnects() const { return m_conns; }

	auto& GetPos() const { return m_pos; }
	void SetPos(const sm::vec2& pos) { m_pos = pos; }

	auto GetFace() const { return m_face; }

private:
	std::shared_ptr<partgraph::TopoFace> m_face;

	std::vector<std::shared_ptr<Node>> m_conns;

	sm::vec2 m_pos;

}; // Node

}