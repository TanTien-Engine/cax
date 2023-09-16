#pragma once

namespace breptopo
{

class Edge
{
public:
	Edge(size_t node1, size_t node2)
		: m_node1(node1), m_node2(node2) {}

private:
	size_t m_node1, m_node2;

}; // Edge

}