#include "wrap_BrepTopo.h"
#include "Graph.h"
#include "Node.h"
#include "TopoAdapter.h"
#include "GraphTools.h"
#include "modules/script/Proxy.h"
#include "modules/script/TransHelper.h"

#include "../partgraph_c/TopoDataset.h"
#include "../partgraph_c/TransHelper.h"

#include <SM_Vector.h>

#include <set>
#include <map>

namespace
{

void w_Graph_allocate()
{
    auto proxy = (tt::Proxy<breptopo::Graph>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<breptopo::Graph>));
    proxy->obj = std::make_shared<breptopo::Graph>();
}

int w_Graph_finalize(void* data)
{
    auto proxy = (tt::Proxy<breptopo::Graph>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<breptopo::Graph>);
}

void w_Graph_get_nodes()
{
    auto graph = ((tt::Proxy<breptopo::Graph>*)ves_toforeign(0))->obj;

    auto& nodes = graph->GetNodes();

    ves_pop(ves_argnum());

    const int num = (int)(nodes.size());
    ves_newlist(num);
    for (int i = 0; i < num; ++i)
    {
        ves_pushnil();
        ves_import_class("breptopo", "Node");
        auto proxy = (tt::Proxy<breptopo::Node>*)ves_set_newforeign(1, 2, sizeof(tt::Proxy<breptopo::Node>));
        proxy->obj = nodes[i];
        ves_pop(1);
        ves_seti(-2, i);
        ves_pop(1);
    }
}

void w_Graph_get_edges()
{
    auto graph = ((tt::Proxy<breptopo::Graph>*)ves_toforeign(0))->obj;

    auto& nodes = graph->GetNodes();

    std::map<std::shared_ptr<breptopo::Node>, size_t> node2idx;
    for (size_t i = 0, n = nodes.size(); i < n; ++i) {
        node2idx.insert({ nodes[i], i });
    }    

    std::set<std::pair<size_t, size_t>> edges;
    for (auto& node : nodes)
    {
        size_t i0 = node2idx.find(node)->second;
        for (auto& conn : node->GetConnects())
        {
            size_t i1 = node2idx.find(conn)->second;
            if (i0 < i1)
                edges.insert({ i0, i1 });
            else
                edges.insert({ i1, i0 });
        }
    }

    std::vector<sm::ivec2> list;
    for (auto& edge : edges) 
    {
        int n0 = static_cast<int>(edge.first);
        int n1 = static_cast<int>(edge.second);
        list.push_back({ n0, n1 });
    }
    tt::return_list(list);
}

void w_Graph_query_node()
{
    auto graph = ((tt::Proxy<breptopo::Graph>*)ves_toforeign(0))->obj;

    float x = (float)ves_tonumber(1);
    float y = (float)ves_tonumber(2);

    auto node = breptopo::GraphTools::QueryNode(*graph, sm::vec2(x, y));
    if (!node)
    {
        ves_set_nil(0);
        return;
    }

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("breptopo", "Node");
    auto proxy = (tt::Proxy<breptopo::Node>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<breptopo::Node>));
    proxy->obj = node;
    ves_pop(1);
}

void w_Node_allocate()
{
    auto proxy = (tt::Proxy<breptopo::Node>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<breptopo::Node>));
    proxy->obj = std::make_shared<breptopo::Node>();
}

int w_Node_finalize(void* data)
{
    auto proxy = (tt::Proxy<breptopo::Node>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<breptopo::Node>);
}

void w_Node_get_pos()
{
    auto node = ((tt::Proxy<breptopo::Node>*)ves_toforeign(0))->obj;
    tt::return_vec(node->GetPos());
}

void w_Node_set_pos()
{
    auto node = ((tt::Proxy<breptopo::Node>*)ves_toforeign(0))->obj;

    float x = (float)ves_tonumber(1);
    float y = (float)ves_tonumber(2);

    node->SetPos({ x, y });
}

void w_Node_get_face()
{
    auto node = ((tt::Proxy<breptopo::Node>*)ves_toforeign(0))->obj;
    partgraph::return_topo_face(node->GetFace());
}

void w_TopoAdapter_build_graph()
{
    auto shape = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;

    auto graph = breptopo::TopoAdapter::BuildGraph(shape);

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("breptopo", "Graph");
    auto proxy = (tt::Proxy<breptopo::Graph>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<breptopo::Graph>));
    proxy->obj = graph;
    ves_pop(1);
}

}

namespace breptopo
{

VesselForeignMethodFn BrepTopoBindMethod(const char* signature)
{
    if (strcmp(signature, "Graph.get_nodes()") == 0) return w_Graph_get_nodes;
    if (strcmp(signature, "Graph.get_edges()") == 0) return w_Graph_get_edges;
    if (strcmp(signature, "Graph.query_node(_,_)") == 0) return w_Graph_query_node;

    if (strcmp(signature, "Node.get_pos()") == 0) return w_Node_get_pos;
    if (strcmp(signature, "Node.set_pos(_,_)") == 0) return w_Node_set_pos;
    if (strcmp(signature, "Node.get_face()") == 0) return w_Node_get_face;

    if (strcmp(signature, "static TopoAdapter.build_graph(_)") == 0) return w_TopoAdapter_build_graph;

    return nullptr;
}

void BrepTopoBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
    if (strcmp(class_name, "Graph") == 0)
    {
        methods->allocate = w_Graph_allocate;
        methods->finalize = w_Graph_finalize;
        return;
    }

    if (strcmp(class_name, "Node") == 0)
    {
        methods->allocate = w_Node_allocate;
        methods->finalize = w_Node_finalize;
        return;
    }
}

}