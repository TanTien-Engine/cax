#include "wrap_BrepTopo.h"
#include "TopoAdapter.h"
#include "modules/script/TransHelper.h"

#include "../partgraph_c/TopoDataset.h"

#include <memory>
#include <vector>

namespace
{

void w_TopoAdapter_build_graph()
{
    std::vector<std::shared_ptr<partgraph::TopoShape>> shapes;
    tt::list_to_foreigns(1, shapes);

    auto graph = breptopo::TopoAdapter::BuildGraph(shapes);

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("graph", "Graph");
    auto proxy = (tt::Proxy<graph::Graph>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<graph::Graph>));
    proxy->obj = graph;
    ves_pop(1);
}

}

namespace breptopo
{

VesselForeignMethodFn BrepTopoBindMethod(const char* signature)
{
    if (strcmp(signature, "static TopoAdapter.build_graph(_)") == 0) return w_TopoAdapter_build_graph;

    return nullptr;
}

void BrepTopoBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
}

}