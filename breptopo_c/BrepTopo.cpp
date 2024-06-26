#include "BrepTopo.h"
#include "NodeShape.h"
#include "NodeInfo.h"
#include "NodeShape.h"
#include "modules/graph/Graph.h"

#include "../partgraph_c/TransHelper.h"

#include <graph/Node.h>

namespace breptopo
{

void init_cb()
{
	tt::Graph::Instance()->RegNodeGetCompCB("topo_shape", [](const graph::Node& node) 
	{
		if (node.HasComponent<NodeShape>())
		{
			auto& shape = node.GetComponent<NodeShape>();
			partgraph::return_topo_shape(shape.GetShape());
		}
		else
		{
			ves_set_nil(0);
		}
	});
	tt::Graph::Instance()->RegNodeGetCompCB("node_desc", [](const graph::Node& node)
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