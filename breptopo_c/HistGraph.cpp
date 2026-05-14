#include "HistGraph.h"
#include "NodeShape.h"
#include "NodeId.h"
#include "NodeFlags.h"
#include "partgraph_c/BRepHistory.h"
#include "partgraph_c/TopoShape.h"

#include <graph/Graph.h>
#include <graph/Node.h>
#include <graph/Edge.h>
#include <graph/GraphLayout.h>

#include <cstring>
#include <utility>

#include <set>
#include <queue>

//#define DEBUG_PRINT

#ifdef DEBUG_PRINT
#include <Windows.h>
#endif // DEBUG_PRINT

#include <assert.h>

namespace breptopo
{

HistGraph::HistGraph()
{
	m_graph = std::make_shared<graph::Graph>();

	InitDelNode();
}

HistGraph::PartialPidMap
HistGraph::Update(const partgraph::BRepHistory& hist, uint32_t type_id, uint32_t op_id)
{
	auto itr = m_op2nodes.find(op_id);
	if (itr == m_op2nodes.end())
		return CreateGraph(hist, type_id, op_id);
	else
		return UpdateGraph(hist, type_id, op_id, itr->second);
}

const std::shared_ptr<graph::Node> 
HistGraph::QueryNode(const std::shared_ptr<partgraph::TopoShape>& shape) const
{
	const size_t* gid = m_curr_shapes.Seek(shape->GetShape());
	if (gid)
		return m_graph->GetNode(*gid);
	else
		return nullptr;
}

bool HistGraph::QueryNodes(uint32_t uid, std::vector<std::shared_ptr<graph::Node>>& results) const
{
	auto itr = m_uid2gid.find(uid);
	if (itr == m_uid2gid.end())
		return false;

	size_t gid = itr->second;

	auto node = m_graph->GetNode(gid);
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

		auto node = m_graph->GetNode(pid);
		std::vector<size_t> cids;
		std::vector<std::shared_ptr<graph::Node>> cnodes;
		for (auto edge : node->GetEdges())
		{
			auto other = edge->GetFromNode() == node.get() ? edge->GetToNode() : edge->GetFromNode();
			auto c_node = other;

			// w_CompGraph_add_*_node
			if (!c_node->HasComponent<NodeFlags>()) {
				continue;
			}

			int cid = -1;
			for (int i = 0; i < m_graph->GetNodesNum(); ++i)
			{
				if (m_graph->GetNode(i).get() == c_node)
				{
					cid = i;
					break;
				}
			}

			auto& cflags = c_node->GetComponent<NodeFlags>();
			if (!cflags.IsActive()) {
				cids.push_back(cid);
			} else {
				cnodes.push_back(m_graph->GetNode(cid));
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
				break;
			}
			return true;
		}
	}

	return false;
}

void HistGraph::InitDelNode()
{
	m_del_node_idx = m_graph->GetNodesNum();

	auto del_node = std::make_shared<graph::Node>();
	del_node->SetValue(-1);

	m_del_node = del_node.get();
	m_graph->AddNode(del_node);
}

HistGraph::PartialPidMap
HistGraph::CreateGraph(const partgraph::BRepHistory& hist, uint32_t type_id, uint32_t op_id)
{
	std::vector<size_t> old_gid, new_gid;

	// init old_gid
	auto& old_map = hist.GetOldMap();
	for (int i = 1; i <= old_map.Extent(); ++i)
	{
#ifdef DEBUG_PRINT
		std::stringstream ss;
		ss << old_map(i).TShape().get();
		std::string s = "++ find " + ss.str() + "\n";
		OutputDebugStringA(s.c_str());
#endif // DEBUG_PRINT

		const size_t* pgid = m_curr_shapes.Seek(old_map(i));
		size_t gid;
		if (pgid) {
			gid = *pgid;
		} else {
			// Shape not yet tracked (e.g. a previous op already consumed it).
			// Create a new root node so the history graph stays connected.
			gid = m_graph->GetNodesNum();

			auto node = std::make_shared<graph::Node>();
			node->SetValue(static_cast<int>(gid));

			auto shape = std::make_shared<partgraph::TopoShape>(old_map(i));
			node->AddComponent<NodeShape>(shape);
			node->AddComponent<NodeId>(0, gid);
			node->AddComponent<NodeFlags>();

			m_graph->AddNode(node);
			m_curr_shapes.Bind(old_map(i), gid);
		}
		old_gid.push_back(gid);
	}

	// init new_gid, add new
	auto& new_map = hist.GetNewMap();

	// unbind old
	for (auto itr : hist.GetIdxMap()) {
		if (itr.second.empty()) {
			m_curr_shapes.UnBind(old_map(itr.first + 1));
		}
	}

	for (int i = 1; i <= new_map.Extent(); ++i)
	{
		const uint32_t uid = CalcUID(type_id, op_id, i - 1);
		auto itr = m_uid2gid.find(uid);
		if (itr == m_uid2gid.end())
		{
			size_t gid = m_graph->GetNodesNum();
			new_gid.push_back(gid);

			auto node = std::make_shared<graph::Node>();
			node->SetValue(static_cast<int>(gid));

			auto shape = std::make_shared<partgraph::TopoShape>(new_map(i));
			node->AddComponent<NodeShape>(shape);

			node->AddComponent<NodeId>(uid, gid);

			node->AddComponent<NodeFlags>();

			m_graph->AddNode(node);

			m_uid2gid.insert({ uid, gid });

			auto itr = m_op2nodes.find(op_id);
			if (itr != m_op2nodes.end())
				itr->second.push_back(gid);
			else
				m_op2nodes.insert({ op_id, {gid} });

#ifdef DEBUG_PRINT
			std::stringstream ss;
			ss << new_map(i).TShape().get();
			std::string s = "++ bind " + ss.str() + "\n";
			OutputDebugStringA(s.c_str());
#endif // DEBUG_PRINT

			m_curr_shapes.Bind(new_map(i), gid);
		}
		else
		{
			size_t gid = itr->second;
			new_gid.push_back(gid);

			auto node = m_graph->GetNode(gid);

			auto shape = std::make_shared<partgraph::TopoShape>(new_map(i));
			node->GetComponent<NodeShape>().SetShape(shape);

#ifdef DEBUG_PRINT
			std::stringstream ss;
			ss << new_map(i).TShape().get();
			std::string s = "++ bind " + ss.str() + "\n";
			OutputDebugStringA(s.c_str());
#endif // DEBUG_PRINT

			m_curr_shapes.Bind(new_map(i), gid);
		}
	}

	// connect new
	for (auto itr : hist.GetIdxMap())
	{
		const size_t f_gid = old_gid[itr.first];
		if (itr.second.empty())
		{
#ifdef DEBUG_PRINT
			auto uid = opencascade::hash(old_map(itr.first + 1).TShape().get());
			std::string s = "++ unbind " + std::to_string(uid) + "\n";
			OutputDebugStringA(s.c_str());
#endif // DEBUG_PRINT

			bool exists = false;
			auto node = m_graph->GetNode(f_gid);
			for (auto edge : node->GetEdges())
			{
				auto other = edge->GetFromNode() == node.get() ? edge->GetToNode() : edge->GetFromNode();
				if (other->GetValue() == -1)
				{
					exists = true;
					break;
				}
			}

			if (!exists)
			{
				m_graph->AddEdge(f_gid, m_del_node_idx);
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
	for (auto itr : hist.GetIdxMap())
	{
		const size_t f_gid = old_gid[itr.first];

		auto node = m_graph->GetNode(f_gid);
		auto& cflags = node->GetComponent<NodeFlags>();
		cflags.SetActive(false);

		for (auto itr_to : itr.second)
		{
			const size_t t_gid = new_gid[itr_to];

			auto node = m_graph->GetNode(t_gid);
			auto& cflags = node->GetComponent<NodeFlags>();
			cflags.SetActive(true);
		}
	}

	graph::GraphLayout::OptimalHierarchy(*m_graph);

	PartialPidMap pid_map;
	for (const auto& kv : hist.GetIdxMap())
	{
		const size_t f_gid = old_gid[kv.first];
		auto old_node = m_graph->GetNode(f_gid);
		const uint32_t old_uid = old_node->GetComponent<NodeId>().GetUID();

		std::vector<uint32_t>& targets = pid_map[old_uid];
		targets.reserve(kv.second.size());
		for (int new_idx : kv.second) {
			targets.push_back(CalcUID(type_id, op_id, new_idx));
		}
	}
	return pid_map;
}

HistGraph::PartialPidMap
HistGraph::UpdateGraph(const partgraph::BRepHistory& hist, uint32_t type_id, uint32_t op_id,
	                   const std::vector<size_t>& old_nodes)
{
	std::vector<size_t> new_nodes;
	auto& new_map = hist.GetNewMap();
	for (int i = 1; i <= new_map.Extent(); ++i)
	{
		const uint32_t uid = CalcUID(type_id, op_id, i - 1);
		auto itr = m_uid2gid.find(uid);
		if (itr != m_uid2gid.end())
			new_nodes.push_back(itr->second);
	}

	if (new_nodes == old_nodes)
	{
		assert(new_nodes.size() == new_map.Extent());
		for (int i = 1; i <= new_map.Extent(); ++i)
		{
			size_t gid = new_nodes[i - 1];
			auto node = m_graph->GetNode(gid);
			auto& cshape = node->GetComponent<NodeShape>();
			if (cshape.GetShape()->GetShape() != new_map(i))
			{
				m_curr_shapes.UnBind(cshape.GetShape()->GetShape());
				m_curr_shapes.Bind(new_map(i), gid);
				cshape.SetShape(std::make_shared<partgraph::TopoShape>(new_map(i)));
			}
		}
	}
	else
	{
		// todo
	}

	return {};
}

void HistGraph::MergeFrom(const HistGraph& other)
{
	// gid remapping: other_gid -> this_gid
	std::map<size_t, size_t> remap;
	remap[other.m_del_node_idx] = m_del_node_idx;

	for (size_t i = 0; i < other.m_graph->GetNodesNum(); ++i)
	{
		if (i == other.m_del_node_idx) continue;
		size_t new_gid = m_graph->GetNodesNum();
		auto node = other.m_graph->GetNode(i);
		// Clear stale edge pointers from the source graph before
		// inserting into this graph; AddEdge will rebuild them.
		node->ClearEdges();
		m_graph->AddNode(node);
		remap[i] = new_gid;
	}

	for (auto& [key, edge] : other.m_graph->GetEdges())
	{
		auto fi = remap.find(key.first);
		auto ti = remap.find(key.second);
		if (fi != remap.end() && ti != remap.end())
			m_graph->AddEdge(fi->second, ti->second);
	}

	NCollection_DataMap<TopoDS_Shape, size_t, TopTools_ShapeMapHasher>::Iterator it(other.m_curr_shapes);
	for (; it.More(); it.Next())
	{
		auto ri = remap.find(it.Value());
		if (ri != remap.end())
			m_curr_shapes.Bind(it.Key(), ri->second);
	}

	for (auto& [uid, gid] : other.m_uid2gid)
	{
		auto ri = remap.find(gid);
		if (ri != remap.end())
			m_uid2gid[uid] = ri->second;
	}

	for (auto& [op_id, gids] : other.m_op2nodes)
	{
		auto& dst = m_op2nodes[op_id];
		for (auto gid : gids)
		{
			auto ri = remap.find(gid);
			if (ri != remap.end())
				dst.push_back(ri->second);
		}
	}
}

uint32_t HistGraph::CalcUID(uint32_t type_id, uint32_t op_id, uint32_t index)
{
	uint32_t uid = ((type_id & 0x07) << 29) |
		((op_id & 0x3FFF) << 15) |
		(index & 0x7FFF);
	return uid;
}

void HistGraph::BindShape(uint32_t uid, const TopoDS_Shape& shape)
{
	auto itr = m_uid2gid.find(uid);
	if (itr == m_uid2gid.end()) return;

	size_t gid = itr->second;
	m_curr_shapes.Bind(shape, gid);

	auto node = m_graph->GetNode(gid);
	if (!node) return;

	auto topo_shape = std::make_shared<partgraph::TopoShape>(shape);
	if (node->HasComponent<NodeShape>())
		node->GetComponent<NodeShape>().SetShape(topo_shape);
	else
		node->AddComponent<NodeShape>(topo_shape);
}

// ---------------------------------------------------------------
//  Binary persistence
//
//  Layout:
//    magic        4 bytes  "HGRF"
//    version      4 bytes  = 1
//    num_nodes    4 bytes
//    for each node:
//      value      4 bytes (int)
//      uid        4 bytes
//      active     1 byte
//    del_node_idx 4 bytes
//    num_edges    4 bytes
//    for each edge:
//      from       4 bytes
//      to         4 bytes
//    num_uid2gid  4 bytes
//    for each:
//      uid        4 bytes
//      gid        4 bytes
//    num_op2nodes 4 bytes
//    for each:
//      op_id      4 bytes
//      num_gids   4 bytes
//      gids       4*n bytes
// ---------------------------------------------------------------

void HistGraph::StoreToByteArray(uint8_t** buf, uint32_t& len) const
{
	// Calculate total size
	uint32_t num_nodes = static_cast<uint32_t>(m_graph->GetNodesNum());
	uint32_t num_edges = static_cast<uint32_t>(m_graph->GetEdges().size());
	uint32_t num_uid2gid = static_cast<uint32_t>(m_uid2gid.size());
	uint32_t num_op2nodes = static_cast<uint32_t>(m_op2nodes.size());

	uint32_t op2nodes_gids_total = 0;
	for (auto& kv : m_op2nodes)
		op2nodes_gids_total += static_cast<uint32_t>(kv.second.size());

	uint32_t total = 4 + 4                                // magic + version
		+ 4 + num_nodes * (4 + 4 + 1)                     // nodes
		+ 4                                                // del_node_idx
		+ 4 + num_edges * 8                                // edges
		+ 4 + num_uid2gid * 8                              // uid2gid
		+ 4 + num_op2nodes * 8 + op2nodes_gids_total * 4;  // op2nodes

	*buf = new uint8_t[total];
	uint8_t* p = *buf;
	len = total;

	auto write32 = [&](uint32_t v) { std::memcpy(p, &v, 4); p += 4; };
	auto write8  = [&](uint8_t v)  { *p = v; p += 1; };

	// magic + version
	std::memcpy(p, "HGRF", 4); p += 4;
	write32(1);

	// nodes
	write32(num_nodes);
	for (uint32_t i = 0; i < num_nodes; ++i)
	{
		auto node = m_graph->GetNode(i);
		write32(static_cast<uint32_t>(node->GetValue()));

		uint32_t uid = 0;
		uint8_t active = 1;
		if (node->HasComponent<NodeId>())
			uid = node->GetComponent<NodeId>().GetUID();
		if (node->HasComponent<NodeFlags>())
			active = node->GetComponent<NodeFlags>().IsActive() ? 1 : 0;

		write32(uid);
		write8(active);
	}

	// del_node_idx
	write32(static_cast<uint32_t>(m_del_node_idx));

	// edges
	write32(num_edges);
	for (auto& [key, edge] : m_graph->GetEdges())
	{
		write32(static_cast<uint32_t>(key.first));
		write32(static_cast<uint32_t>(key.second));
	}

	// uid2gid
	write32(num_uid2gid);
	for (auto& [uid, gid] : m_uid2gid)
	{
		write32(uid);
		write32(static_cast<uint32_t>(gid));
	}

	// op2nodes
	write32(num_op2nodes);
	for (auto& [op_id, gids] : m_op2nodes)
	{
		write32(op_id);
		write32(static_cast<uint32_t>(gids.size()));
		for (auto gid : gids)
			write32(static_cast<uint32_t>(gid));
	}
}

bool HistGraph::LoadFromByteArray(const uint8_t* buf, uint32_t len)
{
	if (len < 8) return false;

	const uint8_t* p = buf;

	auto read32 = [&]() -> uint32_t {
		uint32_t v; std::memcpy(&v, p, 4); p += 4; return v;
	};
	auto read8 = [&]() -> uint8_t {
		return *p++;
	};

	// magic
	if (std::memcmp(p, "HGRF", 4) != 0) return false;
	p += 4;
	uint32_t version = read32();
	if (version != 1) return false;

	// rebuild graph
	m_graph = std::make_shared<graph::Graph>();
	m_curr_shapes.Clear();
	m_uid2gid.clear();
	m_op2nodes.clear();

	// nodes
	uint32_t num_nodes = read32();
	for (uint32_t i = 0; i < num_nodes; ++i)
	{
		int32_t value = static_cast<int32_t>(read32());
		uint32_t uid  = read32();
		uint8_t active = read8();

		auto node = std::make_shared<graph::Node>();
		node->SetValue(value);

		if (value == -1)
		{
			// del node
			m_del_node_idx = i;
			m_del_node = node.get();
		}
		else
		{
			node->AddComponent<NodeId>(uid, i);
			auto& flags = node->AddComponent<NodeFlags>();
			flags.SetActive(active != 0);
		}

		m_graph->AddNode(node);
	}

	// del_node_idx (from file, overrides the detected one above)
	m_del_node_idx = read32();
	m_del_node = m_graph->GetNode(m_del_node_idx).get();

	// edges
	uint32_t num_edges = read32();
	for (uint32_t i = 0; i < num_edges; ++i)
	{
		uint32_t from = read32();
		uint32_t to   = read32();
		m_graph->AddEdge(from, to);
	}

	// uid2gid
	uint32_t num_uid2gid = read32();
	for (uint32_t i = 0; i < num_uid2gid; ++i)
	{
		uint32_t uid = read32();
		uint32_t gid = read32();
		m_uid2gid[uid] = gid;
	}

	// op2nodes
	uint32_t num_op2nodes = read32();
	for (uint32_t i = 0; i < num_op2nodes; ++i)
	{
		uint32_t op_id = read32();
		uint32_t num_gids = read32();
		std::vector<size_t> gids(num_gids);
		for (uint32_t j = 0; j < num_gids; ++j)
			gids[j] = read32();
		m_op2nodes[op_id] = std::move(gids);
	}

	return true;
}

}