#include "HistGraph.h"
#include "NodeShape.h"
#include "NodeId.h"
#include "NodeFlags.h"
#include "../partgraph_c/BRepHistory.h"
#include "../partgraph_c/TopoDataset.h"

#include <graph/Graph.h>
#include <graph/Node.h>
#include <graph/GraphLayout.h>

#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

#include <set>
#include <queue>

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

void HistGraph::Update(const partgraph::BRepHistory& hist, int& time)
{
	if (time < 0) {
		time = ++m_time;
	}

	std::vector<size_t> old_gid, new_gid;

	// init old_gid
	auto& old_map = hist.GetOldMap();
	for (int i = 1; i <= old_map.Extent(); ++i)
	{
		size_t gid = m_curr_shapes.Find(old_map(i));
		old_gid.push_back(gid);
	}

	// disconnect old
	auto& map = hist.GetIdxMap();
	for (auto itr : map)
	{
		const size_t gid = old_gid[itr.first];
		m_graph->ClearEdges(gid);
	}

	// init new_gid, add new
	auto& new_map = hist.GetNewMap();
	for (int i = 1; i <= new_map.Extent(); ++i)
	{
		uint32_t uid = (time << 16) | (i - 1);
		auto itr = m_uid2gid.find(uid);
		if (itr == m_uid2gid.end())
		{
			size_t gid = m_graph->GetNodes().size();
			new_gid.push_back(gid);

			//auto node = std::make_shared<graph::Node>(i - 1);
			auto node = std::make_shared<graph::Node>(static_cast<int>(gid));

			auto face = TopoDS::Face(new_map(i));
			auto pg_face = std::make_shared<partgraph::TopoFace>(face);
			node->AddComponent<NodeShape>(pg_face);

			node->AddComponent<NodeId>(uid, gid);

			node->AddComponent<NodeFlags>();

			m_graph->AddNode(node);

			m_uid2gid.insert({ uid, gid });

			m_curr_shapes.Bind(new_map(i), gid);
		}
		else
		{
			size_t gid = itr->second;
			new_gid.push_back(gid);

			auto node = m_graph->GetNodes()[gid];

			auto face = TopoDS::Face(new_map(i));
			auto pg_face = std::make_shared<partgraph::TopoFace>(face);
			node->GetComponent<NodeShape>().SetFace(pg_face);

			m_curr_shapes.Bind(new_map(i), gid);
		}
	}

	// connect new
	for (auto itr : map)
	{
		const size_t f_gid = old_gid[itr.first];
		if (itr.second.empty())
		{
			m_curr_shapes.UnBind(old_map(itr.first + 1));

			bool exists = false;
			auto& edges = m_graph->GetEdges();
			auto range = edges.equal_range(f_gid);
			for (auto itr = range.first; itr != range.second; ++itr)
			{
				auto t_node = m_graph->GetNodes()[itr->second];
				if (t_node->GetId() == -1) 
				{
					exists = true;
					break;
				}
			}

			if (!exists)
			{
				size_t t_gid = m_graph->GetNodes().size();

				auto node = std::make_shared<graph::Node>(-1);
				m_graph->AddNode(node);

				m_graph->AddEdge(f_gid, t_gid);
			}
		}
		else
		{
			for (auto itr_to : itr.second)
			{
				const size_t t_gid = new_gid[itr_to];
				m_graph->AddEdge(f_gid, t_gid);
			}
		}
	}

	// update active
	for (auto itr : map)
	{
		const size_t f_gid = old_gid[itr.first];

		auto node = m_graph->GetNodes()[f_gid];
		auto& cflags = node->GetComponent<NodeFlags>();
		cflags.SetActive(false);

		for (auto itr_to : itr.second)
		{
			const size_t t_gid = new_gid[itr_to];

			auto node = m_graph->GetNodes()[t_gid];
			auto& cflags = node->GetComponent<NodeFlags>();
			cflags.SetActive(true);
		}
	}

	graph::GraphLayout::OptimalHierarchy(*m_graph);
}

std::shared_ptr<graph::Node> HistGraph::
QueryNode(const std::shared_ptr<partgraph::TopoShape>& shape) const
{
	size_t gid = m_curr_shapes.Find(shape->GetShape());
	return m_graph->GetNodes()[gid];
}

bool HistGraph::QueryNodes(uint32_t uid, std::vector<std::shared_ptr<graph::Node>>& results) const
{
	auto itr = m_uid2gid.find(uid);
	if (itr == m_uid2gid.end())
		return false;

	size_t gid = itr->second;

	auto node = m_graph->GetNodes()[gid];
	auto& cflags = node->GetComponent<NodeFlags>();
	if (cflags.IsActive())
	{
		results.push_back(node);
		return true;
	}

	std::queue<size_t> buf;
	buf.push(gid);
	while (!buf.empty())
	{
		auto pid = buf.front(); buf.pop();

		auto& edges = m_graph->GetEdges();
		auto range = edges.equal_range(pid);
		std::vector<size_t> cids;
		std::vector<std::shared_ptr<graph::Node>> cnodes;
		for (auto itr = range.first; itr != range.second; ++itr)
		{
			auto cid = itr->second;
			auto c_node = m_graph->GetNodes()[cid];
			auto& cflags = c_node->GetComponent<NodeFlags>();
			if (!cflags.IsActive())
			{
				cids.push_back(cid);
				//buf.push(cid);
			}
			else
			{
				cnodes.push_back(c_node);
				//return c_node;
			}
		}

		if (cnodes.empty())
		{
			for (auto cid : cids) {
				buf.push(cid);
			}
		}
		else
		{
			for (auto cnode : cnodes) {
				results.push_back(cnode);
			}
			return true;
		}
	}

	return false;
}

}