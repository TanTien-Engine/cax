#include "BrepTopo.h"
#include "TopoNodeShape.h"
#include "modules/graph/Graph.h"

#include "../partgraph_c/TransHelper.h"

#include <graph/Node.h>

namespace breptopo
{

void init_cb()
{
	tt::Graph::Instance()->RegNodeGetCompCB("topo_shape", [](const graph::Node& node) 
	{
		auto& shape = node.GetComponent<TopoNodeShape>();
        partgraph::return_topo_face(shape.GetFace());
	});
}

} 