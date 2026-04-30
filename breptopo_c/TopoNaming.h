#pragma once

#include <memory>

namespace breptopo
{

class HistGraph;

class TopoNaming
{
public:
	TopoNaming();

	auto GetEdgeGraph() const { return m_edge_hg; }
	auto GetFaceGraph() const { return m_face_hg; }
	auto GetSolidGraph() const { return m_solid_hg; }

	uint16_t NextOpId();

private:
	std::shared_ptr<HistGraph> m_edge_hg;
	std::shared_ptr<HistGraph> m_face_hg;
	std::shared_ptr<HistGraph> m_solid_hg;

	uint32_t m_next_op = 0;

}; // TopoNaming

}