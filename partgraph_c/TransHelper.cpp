#include "TransHelper.h"

namespace partgraph
{

void return_topo_shape(const std::shared_ptr<TopoShape>& shape) 
{
    return return_topo_obj<TopoShape>(shape, "TopoShape");
}

void return_topo_shape_list(const std::vector<std::shared_ptr<TopoShape>>& list)
{
    ves_pop(ves_argnum());

    const int num = (int)list.size();
    ves_newlist(num);
    for (int i = 0; i < num; ++i)
    {
        ves_pushnil();
        ves_import_class("partgraph", "TopoShape");
        auto proxy = (wrapper::Proxy<TopoShape>*)ves_set_newforeign(1, 2, sizeof(wrapper::Proxy<TopoShape>));
        proxy->obj = list[i];
        ves_pop(1);
        ves_seti(-2, i);
        ves_pop(1);
    }
}

}