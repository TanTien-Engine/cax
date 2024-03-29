#include "BrepTopo.h"
#include "NodeShape.h"
#include "HistGraph.h"
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
			partgraph::return_topo_face(shape.GetFace());
		}
		else
		{
			ves_set_nil(0);
		}
	});
}

TT_SINGLETON_DEFINITION(Context)

Context::Context()
{
	m_hist = std::make_shared<HistGraph>();
}

Context::~Context()
{
}

} 