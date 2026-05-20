#include "brepgraph_c/topology/TopoGraph.h"
#include "brepgraph_c/topology/TopoGraphBuilder.h"

namespace brepgraph
{

TopoGraph::TopoGraph(const std::vector<std::shared_ptr<brepkit::TopoShape>>& shapes)
{
	m_graph = TopoGraphBuilder::BuildGraph(shapes);
}

}