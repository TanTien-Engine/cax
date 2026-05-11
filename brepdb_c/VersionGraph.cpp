#include "VersionGraph.h"

#include <graph/Graph.h>
#include <graph/Node.h>
#include <graph/GraphLayout.h>

#include <queue>
#include <set>
#include <unordered_map>

namespace brepdb
{

VersionGraph::VersionGraph(const VersionTree& tree)
{
    m_graph = BuildGraph(tree);
}

std::shared_ptr<graph::Graph>
VersionGraph::BuildGraph(const VersionTree& tree)
{
    auto graph = std::make_shared<graph::Graph>();
    graph->SetDirected(true);

    if (tree.GetNodeCount() == 0) { return graph; }

    // BFS from all roots (DAG-safe with visited set)
    std::unordered_map<uint32_t, size_t> id_to_idx;
    std::vector<uint32_t> bfs_order;
    std::set<uint32_t> visited;

    std::queue<uint32_t> q;
    for (uint32_t root : tree.GetRoots())
    {
        q.push(root);
        visited.insert(root);
    }

    while (!q.empty())
    {
        uint32_t vid = q.front();
        q.pop();

        const VersionNode* vnode = tree.GetNode(vid);
        if (!vnode) { continue; }

        size_t idx = bfs_order.size();
        id_to_idx[vid] = idx;
        bfs_order.push_back(vid);

        for (uint32_t child : vnode->children)
        {
            if (visited.insert(child).second) { q.push(child); }
        }
    }

    // Create graph nodes
    uint32_t current_id = tree.GetCurrentId();

    for (uint32_t vid : bfs_order)
    {
        const VersionNode* vnode = tree.GetNode(vid);

        auto gnode = std::make_shared<graph::Node>();
        gnode->SetValue(static_cast<int>(vid));
        gnode->SetName(vnode->op_desc);

        gnode->AddComponent<NodeVersionInfo>(
            vid,
            vnode->op_desc,
            vnode->op_type,
            vnode->timestamp,
            vid == current_id);

        graph->AddNode(gnode);
    }

    // Create edges (parent -> child)
    for (uint32_t vid : bfs_order)
    {
        const VersionNode* vnode = tree.GetNode(vid);
        size_t from_idx = id_to_idx[vid];

        for (uint32_t child : vnode->children)
        {
            auto it = id_to_idx.find(child);
            if (it != id_to_idx.end()) {
                graph->AddEdge(from_idx, it->second);
            }
        }
    }

    // Hierarchical layout (Sugiyama) fits a tree structure best
    graph::GraphLayout::OptimalHierarchy(*graph);

    return graph;
}

} // namespace brepdb
