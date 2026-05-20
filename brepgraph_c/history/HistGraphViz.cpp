// Visualization-only methods split out of HistGraph.cpp so that headless
// consumers (data generation, training, recognition, unit tests) don't pull
// the graph + ogdf libraries through HistGraph.obj.
//
// What lives here: QueryNode / QueryNodes (shim returning graph::Node) and
// GetGraph (rebuilds a graph::Graph for UI inspection). Anything that
// references graph::Graph / graph::Node / graph::GraphLayout belongs here.
//
// The core lineage methods (Update, GetUID, OpOf, ...) stay in HistGraph.cpp
// and have zero graph-lib dependency.

#include "brepgraph_c/history/HistGraph.h"
#include "brepgraph_c/computation/NodeShape.h"
#include "brepgraph_c/history/NodeId.h"
#include "brepgraph_c/history/NodeFlags.h"
#include "brepgraph_c/common/NodeInfo.h"
#include "brepkit_c/TopoShape.h"

#include <TopAbs_ShapeEnum.hxx>

#include <graph/Graph.h>
#include <graph/Node.h>
#include <graph/GraphLayout.h>

#include <algorithm>
#include <sstream>
#include <unordered_map>

namespace brepgraph
{

namespace
{

std::shared_ptr<graph::Node> MakeShimNode(uint32_t uid,
                                          const std::shared_ptr<brepkit::TopoShape>& shape)
{
    auto n = std::make_shared<graph::Node>();
    n->AddComponent<NodeId>(uid, 0);
    if (shape) n->AddComponent<NodeShape>(shape);
    n->AddComponent<NodeFlags>();
    return n;
}

const char* TypeShortName(uint32_t type_id)
{
    switch (type_id) {
    case TopAbs_SOLID:  return "solid";
    case TopAbs_FACE:   return "face";
    case TopAbs_EDGE:   return "edge";
    case TopAbs_VERTEX: return "vertex";
    default:            return "shape";
    }
}

std::string DescribeUid(uint32_t uid, uint32_t owning_op_id, bool active)
{
    uint32_t type_id = HistGraph::TypeOf(uid);
    uint32_t op_id   = HistGraph::OpOf(uid);
    uint32_t index   = HistGraph::IndexOf(uid);
    std::ostringstream os;
    os << TypeShortName(type_id) << "#" << index
       << " @op" << op_id;
    if (owning_op_id != op_id)
        os << " (in op" << owning_op_id << ")";
    if (!active) os << " [dead]";
    return os.str();
}

} // namespace

std::shared_ptr<graph::Node>
HistGraph::QueryNode(const std::shared_ptr<brepkit::TopoShape>& shape) const
{
    uint32_t uid = GetUID(shape);
    if (uid == 0xFFFFFFFFu) return nullptr;
    return MakeShimNode(uid, shape);
}

bool HistGraph::QueryNodes(uint32_t uid,
                           std::vector<std::shared_ptr<graph::Node>>& results) const
{
    std::vector<std::shared_ptr<brepkit::TopoShape>> shapes;
    if (!QueryCurrentShapes(uid, shapes))
        return false;
    results.reserve(results.size() + shapes.size());
    for (auto& s : shapes)
        results.push_back(MakeShimNode(uid, s));
    return true;
}

// ---------------------------------------------------------------
//  Debug graph (visualization only, rebuilt each call)
//
//  Mirrors the spirit of CalcGraphBuilder::BuildGraph and
//  VersionGraph::BuildGraph: walk our persistent state in a
//  deterministic order (op_id ascending; uid ascending within each op),
//  attach NodeId / NodeFlags / NodeShape / NodeInfo components for the
//  UI to read, then run a hierarchical layout. Any UID referenced by
//  m_forward / m_backward but not in any m_op2uids list (e.g. an
//  orphan-input shape recorded as a parent in a previous op) is added
//  as a synthetic node so all lineage edges resolve.
// ---------------------------------------------------------------

std::shared_ptr<graph::Graph> HistGraph::GetGraph() const
{
    auto g = std::make_shared<graph::Graph>();
    g->SetDirected(true);

    std::unordered_map<uint32_t, size_t> uid2gid;

    auto add_node = [&](uint32_t uid, uint32_t owning_op_id) {
        if (uid2gid.count(uid)) return;
        size_t gid = g->GetNodesNum();
        auto n = std::make_shared<graph::Node>();
        n->SetValue(static_cast<int>(gid));

        bool active = IsActive(uid);
        std::string desc = DescribeUid(uid, owning_op_id, active);
        n->SetName(desc);

        n->AddComponent<NodeId>(uid, gid);
        auto& flags = n->AddComponent<NodeFlags>();
        flags.SetActive(active);

        // NodeShape is attached for BOTH alive and historical nodes. Active
        // nodes carry the current bound shape; dead (consumed) nodes carry
        // the shape they had at consumption time, so clicking a historical
        // node in the UI can still preview the original geometry.
        auto sit = Uid2Shape().find(uid);
        if (sit != Uid2Shape().end())
            n->AddComponent<NodeShape>(std::make_shared<brepkit::TopoShape>(sit->second));

        n->AddComponent<NodeInfo>(std::move(desc));

        g->AddNode(n);
        uid2gid[uid] = gid;
    };

    // 1) Walk Op2Uids() in op_id ascending order -- gives a stable visual
    //    layout where outputs of earlier ops appear "above" later ones.
    const auto& op2uids = Op2Uids();
    std::vector<uint32_t> op_ids;
    op_ids.reserve(op2uids.size());
    for (auto& kv : op2uids) op_ids.push_back(kv.first);
    std::sort(op_ids.begin(), op_ids.end());

    for (uint32_t op_id : op_ids)
    {
        auto it = op2uids.find(op_id);
        for (uint32_t uid : it->second)
            add_node(uid, op_id);
    }

    // 2) Pick up any uid referenced by lineage but not owned by an op
    //    (e.g. an external input shape that was never our output). Mark
    //    its owner op as UINT32_MAX so DescribeUid notes the discrepancy.
    auto sweep_orphans = [&](const std::unordered_map<uint32_t, std::vector<uint32_t>>& m) {
        std::vector<uint32_t> keys;
        keys.reserve(m.size());
        for (auto& kv : m) keys.push_back(kv.first);
        std::sort(keys.begin(), keys.end());
        for (uint32_t k : keys) {
            add_node(k, (k >> 15) & 0x3FFF);
            auto& vec = m.find(k)->second;
            for (uint32_t v : vec) add_node(v, (v >> 15) & 0x3FFF);
        }
    };
    sweep_orphans(Forward());
    sweep_orphans(Backward());

    // 3) Add lineage edges (parent -> child).
    const auto& fwd = Forward();
    std::vector<uint32_t> parents;
    parents.reserve(fwd.size());
    for (auto& kv : fwd) parents.push_back(kv.first);
    std::sort(parents.begin(), parents.end());
    for (uint32_t parent : parents)
    {
        auto pit = uid2gid.find(parent);
        if (pit == uid2gid.end()) continue;
        for (uint32_t child : fwd.find(parent)->second)
        {
            auto cit = uid2gid.find(child);
            if (cit == uid2gid.end()) continue;
            g->AddEdge(pit->second, cit->second);
        }
    }

    graph::GraphLayout::OptimalHierarchy(*g);
    return g;
}

}  // namespace brepgraph
