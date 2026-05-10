#include "VersionGraph.h"
#include "VersionTreeGraphBuilder.h"

namespace brepdb
{

VersionGraph::VersionGraph(const VersionTree& tree)
{
    m_graph = VersionTreeGraphBuilder::BuildGraph(tree);
}

} // namespace brepdb
