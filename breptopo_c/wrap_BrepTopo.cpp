#include "wrap_BrepTopo.h"
#include "TopoGraphBuilder.h"
#include "TopoGraph.h"
#include "HistGraph.h"
#include "BrepTopo.h"
#include "modules/script/TransHelper.h"

#include "../partgraph_c/TopoDataset.h"

#include <memory>
#include <vector>

namespace
{

void w_TopoGraph_allocate()
{
    std::vector<std::shared_ptr<partgraph::TopoShape>> shapes;
    tt::list_to_foreigns(1, shapes);

    auto proxy = (tt::Proxy<breptopo::TopoGraph>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<graph::Graph>));
    proxy->obj = std::make_shared<breptopo::TopoGraph>(shapes);
}

int w_TopoGraph_finalize(void* data)
{
    auto proxy = (tt::Proxy<graph::Graph>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<graph::Graph>);
}

void w_TopoGraph_get_graph()
{
    auto tg = ((tt::Proxy<breptopo::TopoGraph>*)ves_toforeign(0))->obj;

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("graph", "Graph");
    auto proxy = (tt::Proxy<graph::Graph>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<graph::Graph>));
    proxy->obj = tg->GetGraph();
    ves_pop(1);
}

void w_HistGraph_get_hist_graph()
{
    auto hist = breptopo::Context::Instance()->GetHist();

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("graph", "Graph");
    auto proxy = (tt::Proxy<graph::Graph>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<graph::Graph>));
    proxy->obj = hist->GetGraph();
    ves_pop(1);
}

}

namespace breptopo
{

VesselForeignMethodFn BrepTopoBindMethod(const char* signature)
{
    if (strcmp(signature, "TopoGraph.get_graph()") == 0) return w_TopoGraph_get_graph;

    if (strcmp(signature, "static HistGraph.get_hist_graph()") == 0) return w_HistGraph_get_hist_graph;

    return nullptr;
}

void BrepTopoBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
    if (strcmp(class_name, "TopoGraph") == 0)
    {
        methods->allocate = w_TopoGraph_allocate;
        methods->finalize = w_TopoGraph_finalize;
        return;
    }
}

}