#include "wrap_BrepTopo.h"
#include "TopoGraphBuilder.h"
#include "TopoGraph.h"
#include "HistGraph.h"
#include "HistMgr.h"
#include "CompGraph.h"
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

void w_HistMgr_allocate()
{
    auto proxy = (tt::Proxy<breptopo::HistMgr>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<breptopo::HistMgr>));
    proxy->obj = std::make_shared<breptopo::HistMgr>();
}

int w_HistMgr_finalize(void* data)
{
    auto proxy = (tt::Proxy<breptopo::HistMgr>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<breptopo::HistMgr>);
}

void w_HistMgr_get_edge_graph()
{
    auto hm = ((tt::Proxy<breptopo::HistMgr>*)ves_toforeign(0))->obj;

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("breptopo", "HistGraph");
    auto proxy = (tt::Proxy<breptopo::HistGraph>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<breptopo::HistGraph>));
    proxy->obj = hm->GetEdgeGraph();
    ves_pop(1);
}

void w_HistMgr_get_face_graph()
{
    auto hm = ((tt::Proxy<breptopo::HistMgr>*)ves_toforeign(0))->obj;

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("breptopo", "HistGraph");
    auto proxy = (tt::Proxy<breptopo::HistGraph>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<breptopo::HistGraph>));
    proxy->obj = hm->GetFaceGraph();
    ves_pop(1);
}

void w_HistMgr_get_solid_graph()
{
    auto hm = ((tt::Proxy<breptopo::HistMgr>*)ves_toforeign(0))->obj;

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("breptopo", "HistGraph");
    auto proxy = (tt::Proxy<breptopo::HistGraph>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<breptopo::HistGraph>));
    proxy->obj = hm->GetSolidGraph();
    ves_pop(1);
}

void w_HistGraph_allocate()
{
    auto proxy = (tt::Proxy<breptopo::HistGraph>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<breptopo::HistGraph>));
    proxy->obj = std::make_shared<breptopo::HistGraph>(breptopo::HistGraph::Type::Face);
}

int w_HistGraph_finalize(void* data)
{
    auto proxy = (tt::Proxy<breptopo::HistGraph>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<breptopo::HistGraph>);
}

void w_HistGraph_get_hist_graph()
{
    auto tg = ((tt::Proxy<breptopo::HistGraph>*)ves_toforeign(0))->obj;

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("graph", "Graph");
    auto proxy = (tt::Proxy<graph::Graph>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<graph::Graph>));
    proxy->obj = tg->GetGraph();
    ves_pop(1);
}

void w_HistGraph_get_next_op_id()
{
    auto tg = ((tt::Proxy<breptopo::HistGraph>*)ves_toforeign(0))->obj;
    ves_set_number(0, tg->NextOpId());
}

void w_HistGraph_get_node_uid()
{
    auto tg = ((tt::Proxy<breptopo::HistGraph>*)ves_toforeign(0))->obj;

    auto shape = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;

    auto node = tg->QueryNode(shape);
    auto& cid = node->GetComponent<breptopo::NodeId>();
    ves_set_number(0, cid.GetUID());
}

void w_HistGraph_query_shapes()
{
    auto tg = ((tt::Proxy<breptopo::HistGraph>*)ves_toforeign(0))->obj;

    uint32_t uid = (uint32_t)ves_tonumber(1);

    std::vector<std::shared_ptr<graph::Node>> nodes;
    if (tg->QueryNodes(uid, nodes))
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

static void flatten_vars(const std::shared_ptr<breptopo::CompVariant>& src, std::vector<std::shared_ptr<breptopo::CompVariant>>& dst)
{
    if (src->Type() == breptopo::VAR_ARRAY)
    {
        auto& items = std::static_pointer_cast<breptopo::VarArray>(src)->val;
        for (auto item : items)
        {
            flatten_vars(item, dst);
        }
    }
    else
    {
        dst.push_back(src);
    }
}

void w_CompGraph_eval()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int node_idx = (int)ves_tonumber(1);
    auto node = cg->GetNode(node_idx);
    auto& cnode = node->GetComponent<breptopo::NodeComp>();

    std::shared_ptr<breptopo::HistMgr> hm = nullptr;
    auto v_hist = ves_toforeign(2);
    if (v_hist) {
        hm = ((tt::Proxy<breptopo::HistMgr>*)v_hist)->obj;
    }

    int node_num = cg->GetGraph()->GetNodes().size();
    auto cvar = cnode.GetCompNode()->Eval(*cg, hm, node_idx);
    if (!cvar) {
        return;
    }

    if (cg->GetGraph()->GetNodes().size() != node_num) {
        cg->Layout();
    }

    std::vector<std::shared_ptr<breptopo::CompVariant>> vars;
    flatten_vars(cvar, vars);
    if (vars.empty()) {
        return;
    }

    if (vars.size() == 1)
    {
        auto var = vars[0];
        switch (var->Type())
        {
        case breptopo::VAR_NUMBER:
            ves_set_number(0, std::static_pointer_cast<breptopo::VarNumber>(var)->val);
            break;
        case breptopo::VAR_BOOLEAN:
            ves_set_boolean(0, std::static_pointer_cast<breptopo::VarBoolean>(var)->val);
            break;
        case breptopo::VAR_SHAPE:
            partgraph::return_topo_shape(std::static_pointer_cast<breptopo::VarShape>(var)->val);
            break;
        }
    }
    else if (vars.size() > 1)
    {
        ves_pop(ves_argnum());

        const int num = (int)vars.size();
        ves_newlist(num);
        for (int i = 0; i < num; ++i)
        {
            switch (vars[i]->Type())
            {
            case breptopo::VAR_NUMBER:
                ves_pushnumber(std::static_pointer_cast<breptopo::VarNumber>(vars[i])->val);
                break;
            case breptopo::VAR_BOOLEAN:
                ves_pushboolean(std::static_pointer_cast<breptopo::VarBoolean>(vars[i])->val);
                break;
            case breptopo::VAR_SHAPE:
            {
                ves_pushnil();
                ves_import_class("partgraph", "TopoShape");
                auto proxy = (tt::Proxy<partgraph::TopoShape>*)ves_set_newforeign(1, 2, sizeof(tt::Proxy<partgraph::TopoShape>));
                proxy->obj = std::static_pointer_cast<breptopo::VarShape>(vars[i])->val;
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

    int shp = (int)ves_tonumber(1);
    int uid = (int)ves_tonumber(2);

    auto node = std::make_shared<breptopo::NodeSelector>(uid);
    int id = cg->AddNode(node, "selector");

    cg->AddEdge(shp, id);
    cg->AddEdge(uid, id);

    ves_set_number(0, id);
}

void w_CompGraph_add_merge_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    auto nodes = tt::list_to_array<size_t>(1);

    auto node = std::make_shared<breptopo::NodeMerge>(nodes);
    int id = cg->AddNode(node, "merge");

    for (auto i : nodes) {
        cg->AddEdge(i, id);
    }

    ves_set_number(0, id);
}

void w_CompGraph_update_integer_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int node_idx = (int)ves_tonumber(1);
    int val = (int)ves_tonumber(2);

    auto node = cg->GetNode(node_idx);
    auto& cnode = node->GetComponent<breptopo::NodeComp>();
    std::static_pointer_cast<breptopo::NodeInteger>(cnode.GetCompNode())->SetValue(val);
}

void w_CompGraph_update_number_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int node_idx = (int)ves_tonumber(1);
    float val = (float)ves_tonumber(2);

    auto node = cg->GetNode(node_idx);
    auto& cnode = node->GetComponent<breptopo::NodeComp>();
    std::static_pointer_cast<breptopo::NodeNumber>(cnode.GetCompNode())->SetValue(val);
}

void w_CompGraph_update_number3_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int node_idx = (int)ves_tonumber(1);
    double x = ves_tonumber(2);
    double y = ves_tonumber(3);
    double z = ves_tonumber(4);

    auto node = cg->GetNode(node_idx);
    auto& cnode = node->GetComponent<breptopo::NodeComp>();
    std::static_pointer_cast<breptopo::NodeNumber3>(cnode.GetCompNode())->SetValue(sm::vec3(x, y, z));
}

void w_CompGraph_update_boolean_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int node_idx = (int)ves_tonumber(1);
    bool val = ves_toboolean(2);

    auto node = cg->GetNode(node_idx);
    auto& cnode = node->GetComponent<breptopo::NodeComp>();
    std::static_pointer_cast<breptopo::NodeBoolean>(cnode.GetCompNode())->SetValue(val);
}

void w_CompGraph_update_shape_node()
{
    auto cg = ((tt::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int node_idx = (int)ves_tonumber(1);
    auto shape = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(2))->obj;

    auto node = cg->GetNode(node_idx);
    auto& cnode = node->GetComponent<breptopo::NodeComp>();
    std::static_pointer_cast<breptopo::NodeTopoShape>(cnode.GetCompNode())->SetValue(shape);
}

}

namespace breptopo
{

VesselForeignMethodFn BrepTopoBindMethod(const char* signature)
{
    if (strcmp(signature, "TopoGraph.get_graph()") == 0) return w_TopoGraph_get_graph;

    if (strcmp(signature, "HistMgr.get_edge_graph()") == 0) return w_HistMgr_get_edge_graph;
    if (strcmp(signature, "HistMgr.get_face_graph()") == 0) return w_HistMgr_get_face_graph;
    if (strcmp(signature, "HistMgr.get_solid_graph()") == 0) return w_HistMgr_get_solid_graph;

    if (strcmp(signature, "HistGraph.get_hist_graph()") == 0) return w_HistGraph_get_hist_graph;
    if (strcmp(signature, "HistGraph.get_next_op_id()") == 0) return w_HistGraph_get_next_op_id;
    if (strcmp(signature, "HistGraph.get_node_uid(_)") == 0) return w_HistGraph_get_node_uid;
    if (strcmp(signature, "HistGraph.query_shapes(_)") == 0) return w_HistGraph_query_shapes;

    if (strcmp(signature, "CompGraph.get_graph()") == 0) return w_CompGraph_get_graph;
    if (strcmp(signature, "CompGraph.eval(_,_)") == 0) return w_CompGraph_eval;
    if (strcmp(signature, "CompGraph.add_integer_node(_,_)") == 0) return w_CompGraph_add_integer_node;
    if (strcmp(signature, "CompGraph.add_number_node(_,_)") == 0) return w_CompGraph_add_number_node;
    if (strcmp(signature, "CompGraph.add_number3_node(_,_,_,_)") == 0) return w_CompGraph_add_number3_node;
    if (strcmp(signature, "CompGraph.add_boolean_node(_,_)") == 0) return w_CompGraph_add_boolean_node;
    if (strcmp(signature, "CompGraph.add_shape_node(_,_)") == 0) return w_CompGraph_add_shape_node;
    if (strcmp(signature, "CompGraph.add_box_node(_,_,_)") == 0) return w_CompGraph_add_box_node;
    if (strcmp(signature, "CompGraph.add_translate_node(_,_)") == 0) return w_CompGraph_add_translate_node;
    if (strcmp(signature, "CompGraph.add_offset_node(_,_,_)") == 0) return w_CompGraph_add_offset_node;
    if (strcmp(signature, "CompGraph.add_cut_node(_,_)") == 0) return w_CompGraph_add_cut_node;
    if (strcmp(signature, "CompGraph.add_selector_node(_,_)") == 0) return w_CompGraph_add_selector_node;
    if (strcmp(signature, "CompGraph.add_merge_node(_)") == 0) return w_CompGraph_add_merge_node;
    if (strcmp(signature, "CompGraph.update_integer_node(_,_)") == 0) return w_CompGraph_update_integer_node;
    if (strcmp(signature, "CompGraph.update_number_node(_,_)") == 0) return w_CompGraph_update_number_node;
    if (strcmp(signature, "CompGraph.update_number3_node(_,_,_,_)") == 0) return w_CompGraph_update_number3_node;
    if (strcmp(signature, "CompGraph.update_boolean_node(_,_)") == 0) return w_CompGraph_update_boolean_node;
    if (strcmp(signature, "CompGraph.update_shape_node(_,_)") == 0) return w_CompGraph_update_shape_node;

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

    if (strcmp(class_name, "HistMgr") == 0)
    {
        methods->allocate = w_HistMgr_allocate;
        methods->finalize = w_HistMgr_finalize;
        return;
    }

    if (strcmp(class_name, "HistGraph") == 0)
    {
        methods->allocate = w_HistGraph_allocate;
        methods->finalize = w_HistGraph_finalize;
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