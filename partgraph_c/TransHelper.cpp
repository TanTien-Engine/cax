#include "TransHelper.h"

namespace partgraph
{

void return_topo_shape(const std::shared_ptr<TopoShape>& shape) 
{
    return return_topo_obj<TopoShape>(shape, "TopoShape");
}

}