#include "HistGraph.h"
#include "NodeShape.h"
#include "NodeId.h"
#include "../partgraph_c/BRepHistory.h"
#include "../partgraph_c/TopoDataset.h"

#include <graph/Graph.h>
#include <graph/Node.h>
#include <graph/GraphLayout.h>

#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

#include <set>

namespace breptopo
{

HistGraph::HistGraph()
{
	m_graph = std::make_shared<graph::Graph>();
}

void HistGraph::Update(const std::shared_ptr<partgraph::TopoFace>& from, 
	                   const std::vector<std::shared_ptr<partgraph::TopoFace>>& to)
{
}

void HistGraph::Update(const partgraph::BRepHistory& hist)
{
	++m_time;

	std::vector<uint32_t> old_gid, new_gid;

	// from ids
	auto& old_map = hist.GetOldMap();

	//std::vector<uint32_t> old_ids, new_ids;
	//for (int i = 1; i <= m_curr_faces.Extent(); i++)
	//{
	//	old_ids.push_back(m_curr_faces(i).HashCode(0xffff));
	//}
	//for (int i = 1; i <= old_map.Extent(); i++)
	//{
	//	new_ids.push_back(old_map(i).HashCode(0xffff));
	//}

	for (int i = 1; i <= old_map.Extent(); ++i)
	{
		int idx = m_curr_faces.FindIndex(old_map(i));
		assert(idx != 0);
		auto itr = m_idx2gid.find(idx);
		assert(itr != m_idx2gid.end());
		old_gid.push_back(itr->second);
	}

	// rm old
	for (int i = 1; i <= old_map.Extent(); ++i)
	{
		int idx = m_curr_faces.FindIndex(old_map(i));
		auto itr = m_idx2gid.find(idx);
		assert(itr != m_idx2gid.end());
		m_idx2gid.erase(itr);
	}
	for (int i = 1; i <= old_map.Extent(); ++i)
	{
		int idx = m_curr_faces.FindIndex(old_map(i));
		m_curr_faces.RemoveKey(old_map(i));
	}

	//std::vector<uint32_t> end_ids;

	// to ids, add new
	auto& new_map = hist.GetNewMap();
	for (int i = 1; i <= new_map.Extent(); ++i)
	{
		//end_ids.push_back(new_map(i).HashCode(0xffff));

		int idx = m_curr_faces.FindIndex(new_map(i));
		assert(idx == 0);

		idx = m_curr_faces.Add(new_map(i));
		uint32_t uid = (m_time << 16) | (i - 1);

		int gid = m_graph->GetNodes().size();

		m_idx2gid.insert({ idx, gid });
		new_gid.push_back(gid);

		auto node = std::make_shared<graph::Node>(i - 1);

		auto face = TopoDS::Face(new_map(i));
		auto pg_face = std::make_shared<partgraph::TopoFace>(face);
		node->AddComponent<NodeShape>(pg_face);

		node->AddComponent<NodeId>(uid, gid);

		m_graph->AddNode(node);
	}

	auto& map = hist.GetIdxMap();
	for (auto itr : map)
	{
		const int f_gid = old_gid[itr.first];
		// add old, deleted
		if (itr.second.empty())
		{
			int t_gid = m_graph->GetNodes().size();

			auto node = std::make_shared<graph::Node>(-1);
			m_graph->AddNode(node);

			m_graph->AddEdge(f_gid, t_gid);
		}
		else
		{
			for (auto itr_to : itr.second)
			{
				const int t_gid = new_gid[itr_to];
				m_graph->AddEdge(f_gid, t_gid);
			}
		}
	}

	graph::GraphLayout::OptimalHierarchy(*m_graph);
}

}