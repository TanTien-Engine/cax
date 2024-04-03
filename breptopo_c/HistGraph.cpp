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
			auto node = std::make_shared<graph::Node>(gid);

			auto face = TopoDS::Face(new_map(i));
			auto pg_face = std::make_shared<partgraph::TopoFace>(face);
			node->AddComponent<NodeShape>(pg_face);

			node->AddComponent<NodeId>(uid, gid);

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

	auto& map = hist.GetIdxMap();
	for (auto itr : map)
	{
		const size_t f_gid = old_gid[itr.first];
		// add old, deleted
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

	// update inactive
	for (auto itr : map)
	{
		const size_t f_gid = old_gid[itr.first];

		auto node = m_graph->GetNodes()[f_gid];
		auto& cshp = node->GetComponent<NodeShape>();
		cshp.GetFace()->SetInactive(true);

		for (auto itr_to : itr.second)
		{
			const size_t t_gid = new_gid[itr_to];

			auto node = m_graph->GetNodes()[t_gid];
			auto& cshp = node->GetComponent<NodeShape>();
			cshp.GetFace()->SetInactive(false);
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

std::shared_ptr<graph::Node> HistGraph::QueryNode(uint32_t uid) const
{
	auto itr = m_uid2gid.find(uid);
	if (itr == m_uid2gid.end())
		return nullptr;

	size_t gid = itr->second;

	auto node = m_graph->GetNodes()[gid];
	auto& cshp = node->GetComponent<NodeShape>();
	if (!cshp.GetFace()->GetInactive())
		return node;

	std::queue<size_t> buf;
	buf.push(gid);
	while (!buf.empty())
	{
		auto pid = buf.front(); buf.pop();

		auto& edges = m_graph->GetEdges();
		auto range = edges.equal_range(pid);
		for (auto itr = range.first; itr != range.second; ++itr)
		{
			auto cid = itr->second;
			auto c_node = m_graph->GetNodes()[cid];
			auto& cshp = c_node->GetComponent<NodeShape>();
			if (cshp.GetFace()->GetInactive())
			{
				buf.push(cid);
			}
			else
			{
				return c_node;
			}
		}
	}

	return nullptr;
}

}