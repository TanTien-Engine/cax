#pragma once

#include <memory>

namespace breptopo
{

class HistGraph;

class HistMgr
{
public:
	HistMgr();

	auto GetEdgeGraph() const { return m_edge_hg; }
	auto GetFaceGraph() const { return m_face_hg; }
	auto GetSolidGraph() const { return m_solid_hg; }

private:
	std::shared_ptr<HistGraph> m_edge_hg;
	std::shared_ptr<HistGraph> m_face_hg;
	std::shared_ptr<HistGraph> m_solid_hg;

}; // HistMgr

}