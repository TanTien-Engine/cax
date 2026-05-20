#include "brepgraph_c/BrepGraph.h"
#include "brepgraph_c/common/NodeShape.h"
#include "brepgraph_c/common/NodeInfo.h"

#include "brepkit_c/TransHelper.h"

#include <graph/Node.h>
#include <wrapper/Graph.h>

namespace brepgraph
{

void init_cb()
{
	wrapper::Graph::Instance()->RegNodeGetCompCB("topo_shape", [](const graph::Node& node) 
	{
		if (node.HasComponent<NodeShape>())
		{
			auto& shape = node.GetComponent<NodeShape>();
			brepkit::return_topo_shape(shape.GetShape());
		}
		else
		{
			ves_set_nil(0);
		}
	});
	wrapper::Graph::Instance()->RegNodeGetCompCB("node_desc", [](const graph::Node& node)
	{
		if (node.HasComponent<NodeInfo>())
		{
			auto& info = node.GetComponent<NodeInfo>();
			auto desc = info.GetDesc();
			ves_set_lstring(0, desc.c_str(), desc.size());
		}
		else
		{
			ves_set_nil(0);
		}
	});
}

} 