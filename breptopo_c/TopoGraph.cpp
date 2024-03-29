#include "TopoGraph.h"
#include "TopoGraphBuilder.h"

namespace breptopo
{

TopoGraph::TopoGraph(const std::vector<std::shared_ptr<partgraph::TopoShape>>& shapes)
{
	m_graph = TopoGraphBuilder::BuildGraph(shapes);
}

}