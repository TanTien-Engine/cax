#pragma once

#include <memory>

namespace graph { class Graph; }

namespace brepdb
{

class VersionTree;

class VersionGraph
{
public:
    VersionGraph(const VersionTree& tree);

    auto GetGraph() { return m_graph; }

private:
    // Build a directed graph::Graph from the version tree.
    // Each VersionNode becomes a graph::Node with a NodeVersionInfo component.
    // Edges run from parent to child.
    // Layout is computed via OptimalHierarchy (Sugiyama).
    static std::shared_ptr<graph::Graph> BuildGraph(const VersionTree& tree);

private:
    std::shared_ptr<graph::Graph> m_graph;

}; // VersionGraph

} // namespace brepdb
