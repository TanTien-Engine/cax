#include "wrap_BrepTopo.h"
#include "TopoGraphBuilder.h"
#include "TopoGraph.h"
#include "HistGraph.h"
#include "CompGraph.h"
#include "BrepTopo.h"
#include "NodeId.h"
#include "NodeShape.h"
#include "comp_nodes.h"
#include "NodeComp.h"
#include "modules/script/TransHelper.h"

#include "../partgraph_c/TopoShape.h"
#include "../partgraph_c/TransHelper.h"
#include "../partgraph_c/TopoShape.h"

#include <graph/Node.h>
#include <graph/Graph.h>

#include <memory>
#include <vector>

namespace
{

void w_TopoGraph_allocate()
{
    std::vector<std::shared_ptr<partgraph::TopoShape>> shapes;
    tt::list_to_foreigns(1, shapes);

    auto proxy = (tt::Proxy<breptopo::TopoGraph>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<breptopo::TopoGraph>));
    proxy->obj = std::make_shared<breptopo::TopoGraph>(shapes);
}

int w_TopoGraph_finalize(void* data)
{
    auto proxy = (tt::Proxy<breptopo::TopoGraph>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<breptopo::TopoGraph>);
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

void w_HistGraph_get_next_op_id()
{
    auto hist = breptopo::Context::Instance()->GetHist();
    ves_set_number(0, hist->NextOpId());
}

void w_HistGraph_get_node_uid()
{
    auto shape = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;

    auto hist = breptopo::Context::Instance()->GetHist();
    auto node = hist->QueryNode(shape);
    auto& cid = node->GetComponent<breptopo::NodeId>();
    ves_set_number(0, cid.GetUID());
}

void w_HistGraph_query_shapes()
{
    uint32_t uid = (uint32_t)ves_tonumber(1);

    auto hist = breptopo::Context::Instance()->GetHist();
    std::vector<std::shared_ptr<graph::Node>> nodes;
    if (hist->QueryNodes(uid, nodes))
    {
        assert(!nodes.empty());

        ves_pop(ves_argnum());

        const int num = (int)nodes.size();
        ves_newlist(num);
        for (int i = 0; i < num; ++i)
        {
            ves_pushnil();
            ves_import_class("partgraph", "TopoShape");
            auto proxy = (tt::Proxy<partgraph::TopoShape>*)ves_set_newforeign(1, 2, sizeof(tt::Proxy<partgraph::TopoShape>));
            proxy->obj = nodes[i]->GetComponent<breptopo::NodeShape>().GetShape();
            ves_pop(1);
            ves_seti(-2, i);
            ves_pop(1);
        }
    }
    else
    {
        ves_set_nil(0);
    }
}

void w_CompGraph_allocate()
{
    auto proxy = (tt::Proxy<breptopo::CompGraph>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<breptopo::CompGraph>));
    proxy->obj = std::make_shared<breptopo::CompGraph>();
}

int w_CompGraph_finalize(void* data)
{
    auto proxy = (tt::Proxy<breptopo::CompGraph>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<breptopo::CompGraph>);
}

void w_CompGraph_get_graph()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("graph", "Graph");
    auto proxy = (tt::Proxy<graph::Graph>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<graph::Graph>));
    proxy->obj = cg->GetGraph();
    ves_pop(1);
}

void w_CompGraph_eval()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int node_idx = (int)ves_tonumber(1);
    auto g = cg->GetGraph();
    auto node = g->GetNodes()[node_idx];
    auto& cnode = node->GetComponent<breptopo::NodeComp>();

    auto cvar = cnode.GetCompNode()->Eval(*g);
    switch (cvar->Type())
    {
    case breptopo::VAR_NUMBER:
        ves_set_number(0, std::static_pointer_cast<breptopo::VarNumber>(cvar)->val);
        break;
    case breptopo::VAR_BOOLEAN:
        ves_set_boolean(0, std::static_pointer_cast<breptopo::VarBoolean>(cvar)->val);
        break;
    case breptopo::VAR_SHAPE:
        partgraph::return_topo_shape(std::static_pointer_cast<breptopo::VarShape>(cvar)->val);
        break;
    case breptopo::VAR_ARRAY:
    {
        auto& array = std::static_pointer_cast<breptopo::VarArray>(cvar)->val;

        ves_pop(ves_argnum());

        const int num = (int)array.size();
        ves_newlist(num);
        for (int i = 0; i < num; ++i)
        {
            switch (array[i]->Type())
            {
            case breptopo::VAR_NUMBER:
                ves_pushnumber(std::static_pointer_cast<breptopo::VarNumber>(array[i])->val);
                break;
            case breptopo::VAR_BOOLEAN:
                ves_pushboolean(std::static_pointer_cast<breptopo::VarBoolean>(array[i])->val);
                break;
            case breptopo::VAR_SHAPE:
            {
                ves_pushnil();
                ves_import_class("partgraph", "TopoShape");
                auto proxy = (tt::Proxy<partgraph::TopoShape>*)ves_set_newforeign(1, 2, sizeof(tt::Proxy<partgraph::TopoShape>));
                proxy->obj = std::static_pointer_cast<breptopo::VarShape>(array[i])->val;
                ves_pop(1);
            }
                break;
            default:
                assert(0);
            }
            ves_seti(-2, i);
            ves_pop(1);
        }
    }
        break;
    default:
        assert(0);
    }
}

void w_CompGraph_add_integer_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int val = (int)ves_tonumber(1);
    const char* desc = ves_tostring(2);

    auto node = std::make_shared<breptopo::NodeInteger>(val);
    int id = cg->AddNode(node, desc);

    ves_set_number(0, id);
}

void w_CompGraph_add_number_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    float num = (float)ves_tonumber(1);
    const char* desc = ves_tostring(2);

    auto node = std::make_shared<breptopo::NodeNumber>(num);
    int id = cg->AddNode(node, desc);

    ves_set_number(0, id);
}

void w_CompGraph_add_number3_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    float x = (float)ves_tonumber(1);
    float y = (float)ves_tonumber(2);
    float z = (float)ves_tonumber(3);
    const char* desc = ves_tostring(4);

    auto node = std::make_shared<breptopo::NodeNumber3>(sm::vec3(x, y, z));
    int id = cg->AddNode(node, desc);

    ves_set_number(0, id);
}

void w_CompGraph_add_boolean_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    bool b = ves_toboolean(1);
    const char* desc = ves_tostring(2);

    auto node = std::make_shared<breptopo::NodeBoolean>(b);
    int id = cg->AddNode(node, desc);

    ves_set_number(0, id);
}

void w_CompGraph_add_shape_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    auto shape = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    const char* desc = ves_tostring(2);

    auto node = std::make_shared<breptopo::NodeTopoShape>(shape);
    int id = cg->AddNode(node, desc);

    ves_set_number(0, id);
}

void w_CompGraph_add_box_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int length = (int)ves_tonumber(1);
    int width  = (int)ves_tonumber(2);
    int height = (int)ves_tonumber(3);

    auto node = std::make_shared<breptopo::NodeBox>(length, width, height);
    int id = cg->AddNode(node, "box op");

    cg->AddEdge(length, id);
    cg->AddEdge(width, id);
    cg->AddEdge(height, id);

    ves_set_number(0, id);
}

void w_CompGraph_add_translate_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int shape = (int)ves_tonumber(1);
    int offset = (int)ves_tonumber(2);

    auto node = std::make_shared<breptopo::NodeTranslate>(shape, offset);
    int id = cg->AddNode(node, "translate op");

    cg->AddEdge(shape, id);
    cg->AddEdge(offset, id);

    ves_set_number(0, id);
}

void w_CompGraph_add_offset_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int shape = (int)ves_tonumber(1);
    int offset = (int)ves_tonumber(2);
    int is_solid = (int)ves_tonumber(3);

    auto node = std::make_shared<breptopo::NodeOffset>(shape, offset, is_solid);
    int id = cg->AddNode(node, "offset op");

    cg->AddEdge(shape, id);
    cg->AddEdge(offset, id);
    cg->AddEdge(is_solid, id);

    ves_set_number(0, id);
}

void w_CompGraph_add_cut_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int shp1 = (int)ves_tonumber(1);
    int shp2 = (int)ves_tonumber(2);

    auto node = std::make_shared<breptopo::NodeCut>(shp1, shp2);
    int id = cg->AddNode(node, "cut op");

    cg->AddEdge(shp1, id);
    cg->AddEdge(shp2, id);

    ves_set_number(0, id);
}

void w_CompGraph_add_selector_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int uid = (int)ves_tonumber(1);

    auto node = std::make_shared<breptopo::NodeSelector>(uid);
    int id = cg->AddNode(node, "selector");

    cg->AddEdge(uid, id);

    ves_set_number(0, id);
}

void w_CompGraph_add_merge_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    auto nodes = tt::list_to_array<int>(1);

    auto node = std::make_shared<breptopo::NodeMerge>(nodes);
    int id = cg->AddNode(node, "merge");

    for (auto i : nodes) {
        cg->AddEdge(i, id);
    }

    ves_set_number(0, id);
}

}

namespace breptopo
{

VesselForeignMethodFn BrepTopoBindMethod(const char* signature)
{
    if (strcmp(signature, "TopoGraph.get_graph()") == 0) return w_TopoGraph_get_graph;

    if (strcmp(signature, "static HistGraph.get_hist_graph()") == 0) return w_HistGraph_get_hist_graph;
    if (strcmp(signature, "static HistGraph.get_next_op_id()") == 0) return w_HistGraph_get_next_op_id;
    if (strcmp(signature, "static HistGraph.get_node_uid(_)") == 0) return w_HistGraph_get_node_uid;
    if (strcmp(signature, "static HistGraph.query_shapes(_)") == 0) return w_HistGraph_query_shapes;

    if (strcmp(signature, "CompGraph.get_graph()") == 0) return w_CompGraph_get_graph;
    if (strcmp(signature, "CompGraph.eval(_)") == 0) return w_CompGraph_eval;
    if (strcmp(signature, "CompGraph.add_integer_node(_,_)") == 0) return w_CompGraph_add_integer_node;
    if (strcmp(signature, "CompGraph.add_number_node(_,_)") == 0) return w_CompGraph_add_number_node;
    if (strcmp(signature, "CompGraph.add_number3_node(_,_,_,_)") == 0) return w_CompGraph_add_number3_node;
    if (strcmp(signature, "CompGraph.add_boolean_node(_,_)") == 0) return w_CompGraph_add_boolean_node;
    if (strcmp(signature, "CompGraph.add_shape_node(_,_)") == 0) return w_CompGraph_add_shape_node;
    if (strcmp(signature, "CompGraph.add_box_node(_,_,_)") == 0) return w_CompGraph_add_box_node;
    if (strcmp(signature, "CompGraph.add_translate_node(_,_)") == 0) return w_CompGraph_add_translate_node;
    if (strcmp(signature, "CompGraph.add_offset_node(_,_,_)") == 0) return w_CompGraph_add_offset_node;
    if (strcmp(signature, "CompGraph.add_cut_node(_,_)") == 0) return w_CompGraph_add_cut_node;
    if (strcmp(signature, "CompGraph.add_selector_node(_)") == 0) return w_CompGraph_add_selector_node;
    if (strcmp(signature, "CompGraph.add_merge_node(_)") == 0) return w_CompGraph_add_merge_node;

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

    if (strcmp(class_name, "CompGraph") == 0)
    {
        methods->allocate = w_CompGraph_allocate;
        methods->finalize = w_CompGraph_finalize;
        return;
    }
}

}