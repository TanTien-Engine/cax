#include "HistGraph.h"
#include "NodeShape.h"
#include "NodeId.h"
#include "NodeFlags.h"
#include "partgraph_c/BRepHistory.h"
#include "partgraph_c/TopoShape.h"

#include <graph/Graph.h>
#include <graph/Node.h>

#include <algorithm>
#include <cassert>
#include <cstring>
#include <queue>
#include <unordered_set>

namespace breptopo
{

// ---------------------------------------------------------------
//  Construction
// ---------------------------------------------------------------

HistGraph::HistGraph() = default;

uint32_t HistGraph::CalcUID(uint32_t type_id, uint32_t op_id, uint32_t index)
{
	uint32_t uid = ((type_id & 0x07) << 29) |
		((op_id & 0x3FFF) << 15) |
		(index & 0x7FFF);
	return uid;
}

// ---------------------------------------------------------------
//  Update -- record one OCC modeling step's lineage
//
//  This is the single entry point for mutating the graph. No
//  CreateGraph / UpdateGraph split: re-running the same op with the
//  same op_id simply rebinds the shapes for the same uids and
//  rewrites the forward map for the parents.
// ---------------------------------------------------------------

HistGraph::PartialPidMap
HistGraph::Update(const partgraph::BRepHistory& hist, uint32_t type_id, uint32_t op_id)
{
	PartialPidMap pid_map;
	auto& new_map = hist.GetNewMap();
	auto& old_map = hist.GetOldMap();

	// 1. Assign uids for all outputs of this op and (re)bind shape<->uid.
	std::vector<uint32_t> new_uids;
	new_uids.reserve(new_map.Extent());
	for (int i = 1; i <= new_map.Extent(); ++i)
	{
		uint32_t uid = CalcUID(type_id, op_id, i - 1);
		BindShape(uid, new_map(i));
		new_uids.push_back(uid);
	}

	// 2. For each (old -> new) entry produced by OCC's history, record
	//    the lineage. Unknown old shapes (never bound by a prior op)
	//    are skipped -- their downstream selectors can't reference them.
	for (auto& kv : hist.GetIdxMap())
	{
		int  old_idx  = kv.first;
		auto& new_idxs = kv.second;

		const TopoDS_Shape& old_shape = old_map(old_idx + 1);
		const uint32_t* poid = m_shape2uid.Seek(old_shape);
		if (!poid) continue;
		uint32_t old_uid = *poid;

		std::vector<uint32_t> children;
		children.reserve(new_idxs.size());
		for (int ni : new_idxs)
			children.push_back(new_uids[ni]);

		pid_map[old_uid] = children;
		m_forward[old_uid] = children;
		for (uint32_t c : children)
		{
			auto& bv = m_backward[c];
			if (std::find(bv.begin(), bv.end(), old_uid) == bv.end())
				bv.push_back(old_uid);
		}

		// Old shape is consumed -- remove from live bind maps.
		m_shape2uid.UnBind(old_shape);
		m_uid2shape.erase(old_uid);
	}

	m_op2uids[op_id] = std::move(new_uids);
	return pid_map;
}

// ---------------------------------------------------------------
//  Lookups
// ---------------------------------------------------------------

uint32_t HistGraph::GetUID(const std::shared_ptr<partgraph::TopoShape>& shape) const
{
	if (!shape) return 0xFFFFFFFFu;
	return GetUID(shape->GetShape());
}

uint32_t HistGraph::GetUID(const TopoDS_Shape& shape) const
{
	const uint32_t* puid = m_shape2uid.Seek(shape);
	return puid ? *puid : 0xFFFFFFFFu;
}

bool HistGraph::QueryCurrentShapes(uint32_t uid,
                                   std::vector<std::shared_ptr<partgraph::TopoShape>>& out) const
{
	// Track whether the uid is even known to this graph, so callers can
	// distinguish "never heard of it" from "known but consumed without
	// surviving descendants".
	bool known = (m_uid2shape.find(uid) != m_uid2shape.end()) ||
	             (m_forward.find(uid)   != m_forward.end())   ||
	             (m_backward.find(uid)  != m_backward.end());

	// Fast path: uid is still alive -- return its current shape.
	auto it = m_uid2shape.find(uid);
	if (it != m_uid2shape.end())
	{
		out.push_back(std::make_shared<partgraph::TopoShape>(it->second));
		return true;
	}

	// BFS forward through lineage, collecting all live descendants.
	std::unordered_set<uint32_t> visited;
	std::queue<uint32_t> q;
	q.push(uid);
	visited.insert(uid);

	while (!q.empty())
	{
		uint32_t u = q.front(); q.pop();
		auto fwd = m_forward.find(u);
		if (fwd == m_forward.end()) continue;
		for (uint32_t c : fwd->second)
		{
			if (!visited.insert(c).second) continue;
			auto sit = m_uid2shape.find(c);
			if (sit != m_uid2shape.end())
				out.push_back(std::make_shared<partgraph::TopoShape>(sit->second));
			else
				q.push(c);
		}
	}

	return known;
}

std::vector<std::shared_ptr<partgraph::TopoShape>>
HistGraph::QueryCurrentShapes(uint32_t uid) const
{
	std::vector<std::shared_ptr<partgraph::TopoShape>> out;
	QueryCurrentShapes(uid, out);
	return out;
}

// ---------------------------------------------------------------
//  Backward-compatibility shims: synthesize graph::Node objects on
//  demand so callers using QueryNode / QueryNodes keep working.
// ---------------------------------------------------------------

namespace
{

std::shared_ptr<graph::Node> MakeShimNode(uint32_t uid,
                                          const std::shared_ptr<partgraph::TopoShape>& shape)
{
	auto n = std::make_shared<graph::Node>();
	n->AddComponent<NodeId>(uid, 0);
	if (shape) n->AddComponent<NodeShape>(shape);
	n->AddComponent<NodeFlags>();
	return n;
}

} // namespace

std::shared_ptr<graph::Node>
HistGraph::QueryNode(const std::shared_ptr<partgraph::TopoShape>& shape) const
{
	uint32_t uid = GetUID(shape);
	if (uid == 0xFFFFFFFFu) return nullptr;
	return MakeShimNode(uid, shape);
}

bool HistGraph::QueryNodes(uint32_t uid,
                           std::vector<std::shared_ptr<graph::Node>>& results) const
{
	std::vector<std::shared_ptr<partgraph::TopoShape>> shapes;
	if (!QueryCurrentShapes(uid, shapes))
		return false;
	results.reserve(results.size() + shapes.size());
	for (auto& s : shapes)
		results.push_back(MakeShimNode(uid, s));
	return true;
}

// ---------------------------------------------------------------
//  Historical predicates
// ---------------------------------------------------------------

bool HistGraph::IsKnown(uint32_t uid) const
{
	if (m_uid2shape.find(uid) != m_uid2shape.end()) return true;
	if (m_forward.find(uid)   != m_forward.end())   return true;
	if (m_backward.find(uid)  != m_backward.end())  return true;
	for (auto& kv : m_op2uids)
		if (std::find(kv.second.begin(), kv.second.end(), uid) != kv.second.end())
			return true;
	return false;
}

bool HistGraph::IsActive(uint32_t uid) const
{
	return m_uid2shape.find(uid) != m_uid2shape.end();
}

const std::vector<uint32_t>* HistGraph::Successors(uint32_t uid) const
{
	auto it = m_forward.find(uid);
	return (it != m_forward.end()) ? &it->second : nullptr;
}

const std::vector<uint32_t>* HistGraph::Predecessors(uint32_t uid) const
{
	auto it = m_backward.find(uid);
	return (it != m_backward.end()) ? &it->second : nullptr;
}

// ---------------------------------------------------------------
//  BindShape -- populate transient maps for a known uid
// ---------------------------------------------------------------

void HistGraph::BindShape(uint32_t uid, const TopoDS_Shape& shape)
{
	if (shape.IsNull()) return;

	auto it = m_uid2shape.find(uid);
	if (it != m_uid2shape.end() && !it->second.IsSame(shape))
		m_shape2uid.UnBind(it->second);

	m_uid2shape[uid] = shape;
	m_shape2uid.Bind(shape, uid);
}

// ---------------------------------------------------------------
//  MergeFrom -- union the persistent + transient state of `other`
// ---------------------------------------------------------------

void HistGraph::MergeFrom(const HistGraph& other)
{
	for (auto& kv : other.m_op2uids)
		m_op2uids[kv.first] = kv.second;
	for (auto& kv : other.m_forward)
		m_forward[kv.first] = kv.second;
	for (auto& kv : other.m_backward)
	{
		auto& bv = m_backward[kv.first];
		for (uint32_t p : kv.second)
			if (std::find(bv.begin(), bv.end(), p) == bv.end())
				bv.push_back(p);
	}
	for (auto& kv : other.m_uid2shape)
	{
		if (m_uid2shape.find(kv.first) == m_uid2shape.end())
		{
			m_uid2shape[kv.first] = kv.second;
			m_shape2uid.Bind(kv.second, kv.first);
		}
	}
}

// ---------------------------------------------------------------
//  Clone -- deep copy for parallel fork
// ---------------------------------------------------------------

std::shared_ptr<HistGraph> HistGraph::Clone() const
{
	auto out = std::make_shared<HistGraph>();
	out->m_op2uids  = m_op2uids;
	out->m_forward  = m_forward;
	out->m_backward = m_backward;
	out->m_uid2shape = m_uid2shape;
	// Rebuild shape2uid from uid2shape (NCollection map is not trivially copyable).
	for (auto& kv : out->m_uid2shape)
		out->m_shape2uid.Bind(kv.second, kv.first);
	return out;
}

HistGraph::Snapshot HistGraph::TakeSnapshot() const
{
	Snapshot s;
	s.op2uids = m_op2uids;
	s.forward = m_forward;
	return s;
}

void HistGraph::AbsorbFork(const HistGraph& fork, const Snapshot& base)
{
	// op_id additions: any op_id in fork not present (or with a different
	// uid list) at snapshot time is a new contribution. Overwrite our entry.
	for (auto& kv : fork.m_op2uids)
	{
		auto bit = base.op2uids.find(kv.first);
		if (bit == base.op2uids.end() || bit->second != kv.second)
			m_op2uids[kv.first] = kv.second;
	}

	// forward additions / overrides
	for (auto& kv : fork.m_forward)
	{
		auto bit = base.forward.find(kv.first);
		if (bit == base.forward.end() || bit->second != kv.second)
			m_forward[kv.first] = kv.second;
	}

	// backward: union all parents (parents of disjoint forks don't collide).
	for (auto& kv : fork.m_backward)
	{
		auto& bv = m_backward[kv.first];
		for (uint32_t p : kv.second)
			if (std::find(bv.begin(), bv.end(), p) == bv.end())
				bv.push_back(p);
	}

	// Transient shape bindings: add new uids; for existing uids prefer the
	// fork's binding if it differs (fork ran the op more recently).
	for (auto& kv : fork.m_uid2shape)
	{
		auto it = m_uid2shape.find(kv.first);
		if (it == m_uid2shape.end())
		{
			m_uid2shape[kv.first] = kv.second;
			m_shape2uid.Bind(kv.second, kv.first);
		}
		else if (!it->second.IsSame(kv.second))
		{
			m_shape2uid.UnBind(it->second);
			it->second = kv.second;
			m_shape2uid.Bind(kv.second, kv.first);
		}
	}
}

// ---------------------------------------------------------------
//  Debug graph (visualization only, rebuilt each call)
// ---------------------------------------------------------------

std::shared_ptr<graph::Graph> HistGraph::GetGraph() const
{
	auto g = std::make_shared<graph::Graph>();
	std::unordered_map<uint32_t, size_t> uid2gid;

	// Collect all uids ever seen: union of op2uids + forward keys/values + backward keys/values.
	std::unordered_set<uint32_t> all_uids;
	for (auto& kv : m_op2uids)
		for (uint32_t u : kv.second) all_uids.insert(u);
	for (auto& kv : m_forward)
	{
		all_uids.insert(kv.first);
		for (uint32_t c : kv.second) all_uids.insert(c);
	}
	for (auto& kv : m_backward)
	{
		all_uids.insert(kv.first);
		for (uint32_t p : kv.second) all_uids.insert(p);
	}

	for (uint32_t uid : all_uids)
	{
		auto n = std::make_shared<graph::Node>();
		size_t gid = g->GetNodesNum();
		n->SetValue(static_cast<int>(gid));
		n->AddComponent<NodeId>(uid, gid);
		auto& flags = n->AddComponent<NodeFlags>();
		flags.SetActive(m_uid2shape.find(uid) != m_uid2shape.end());
		auto it = m_uid2shape.find(uid);
		if (it != m_uid2shape.end())
			n->AddComponent<NodeShape>(std::make_shared<partgraph::TopoShape>(it->second));
		g->AddNode(n);
		uid2gid[uid] = gid;
	}

	for (auto& kv : m_forward)
	{
		auto pit = uid2gid.find(kv.first);
		if (pit == uid2gid.end()) continue;
		for (uint32_t c : kv.second)
		{
			auto cit = uid2gid.find(c);
			if (cit == uid2gid.end()) continue;
			g->AddEdge(pit->second, cit->second);
		}
	}

	return g;
}

// ---------------------------------------------------------------
//  Persistence
//
//  Format v1 (magic "HGRF"):
//    magic        4 bytes
//    version      4 bytes  = 1
//    op2uids      uint32 count, then [op_id, uid_count, uid*]
//    forward      uint32 count, then [src_uid, dst_count, dst_uid*]
//
//  m_backward is rebuilt from m_forward on load.
//  Transient maps (m_uid2shape / m_shape2uid) are NOT persisted --
//  they get repopulated from the VersionTree warm-up or by re-eval.
// ---------------------------------------------------------------

namespace
{

inline uint32_t Rd32(const uint8_t*& p) { uint32_t v; std::memcpy(&v, p, 4); p += 4; return v; }
inline void     Wr32(uint8_t*& p, uint32_t v) { std::memcpy(p, &v, 4); p += 4; }

constexpr uint32_t HG_MAGIC   = 0x46524748u; // "HGRF" little-endian
constexpr uint32_t HG_VERSION = 1;

} // namespace

void HistGraph::StoreToByteArray(uint8_t** buf, uint32_t& len) const
{
	uint32_t total = 4 + 4 + 4; // magic + version + op2uids count
	for (auto& kv : m_op2uids)
		total += 4 + 4 + 4 * static_cast<uint32_t>(kv.second.size());
	total += 4; // forward count
	for (auto& kv : m_forward)
		total += 4 + 4 + 4 * static_cast<uint32_t>(kv.second.size());

	*buf = new uint8_t[total];
	len  = total;
	uint8_t* p = *buf;

	Wr32(p, HG_MAGIC);
	Wr32(p, HG_VERSION);

	Wr32(p, static_cast<uint32_t>(m_op2uids.size()));
	for (auto& kv : m_op2uids)
	{
		Wr32(p, kv.first);
		Wr32(p, static_cast<uint32_t>(kv.second.size()));
		for (uint32_t u : kv.second) Wr32(p, u);
	}

	Wr32(p, static_cast<uint32_t>(m_forward.size()));
	for (auto& kv : m_forward)
	{
		Wr32(p, kv.first);
		Wr32(p, static_cast<uint32_t>(kv.second.size()));
		for (uint32_t u : kv.second) Wr32(p, u);
	}

	assert(static_cast<uint32_t>(p - *buf) == total);
}

bool HistGraph::LoadFromByteArray(const uint8_t* buf, uint32_t len)
{
	if (len < 12) return false;
	const uint8_t* p = buf;

	uint32_t magic   = Rd32(p);
	uint32_t version = Rd32(p);
	if (magic != HG_MAGIC || version != HG_VERSION) return false;

	m_op2uids.clear();
	m_forward.clear();
	m_backward.clear();
	m_uid2shape.clear();
	m_shape2uid.Clear();

	uint32_t op_count = Rd32(p);
	for (uint32_t i = 0; i < op_count; ++i)
	{
		uint32_t op_id = Rd32(p);
		uint32_t n     = Rd32(p);
		std::vector<uint32_t> uids(n);
		for (uint32_t j = 0; j < n; ++j) uids[j] = Rd32(p);
		m_op2uids[op_id] = std::move(uids);
	}

	uint32_t fwd_count = Rd32(p);
	for (uint32_t i = 0; i < fwd_count; ++i)
	{
		uint32_t src = Rd32(p);
		uint32_t n   = Rd32(p);
		std::vector<uint32_t> dst(n);
		for (uint32_t j = 0; j < n; ++j) dst[j] = Rd32(p);
		m_forward[src] = std::move(dst);
	}

	// Rebuild backward from forward.
	for (auto& kv : m_forward)
		for (uint32_t c : kv.second)
			m_backward[c].push_back(kv.first);

	return true;
}

} // namespace breptopo
