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
    std::shared_ptr<graph::Graph> m_graph;

}; // VersionGraph

} // namespace brepdb
