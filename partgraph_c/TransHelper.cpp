#include "TransHelper.h"

namespace partgraph
{

void return_topo_shape(const std::shared_ptr<TopoShape>& shape) 
{
    return return_topo_obj<TopoShape>(shape, "TopoShape");
}

void return_topo_edge(const std::shared_ptr<TopoEdge>& edge) 
{
    return return_topo_obj<TopoEdge>(edge, "TopoEdge");
}

void return_topo_wire(const std::shared_ptr<TopoWire>& wire) 
{
    return return_topo_obj<TopoWire>(wire, "TopoWire");
}

void return_topo_face(const std::shared_ptr<TopoFace>& face) 
{
    return return_topo_obj<TopoFace>(face, "TopoFace");
}

void return_topo_shell(const std::shared_ptr<TopoShell>& shell)
{
    return return_topo_obj<TopoShell>(shell, "TopoShell");
}

}