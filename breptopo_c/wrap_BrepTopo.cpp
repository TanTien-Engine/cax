#include "wrap_BrepTopo.h"
#include "TopoGraphBuilder.h"
#include "TopoGraph.h"
#include "HistGraph.h"
#include "TopoNaming.h"
#include "CompGraph.h"
#include "CompGraphBuilder.h"
#include "NodeId.h"
#include "NodeShape.h"

#include "partgraph_c/TopoShape.h"
#include "partgraph_c/TransHelper.h"

#include <graph/Node.h>
#include <graph/Graph.h>
#include <wrapper/TransHelper.h>

#include <memory>
#include <vector>

namespace
{

void w_TopoGraph_allocate()
{
    std::vector<std::shared_ptr<partgraph::TopoShape>> shapes;
    wrapper::list_to_foreigns(1, shapes);

    auto proxy = (wrapper::Proxy<breptopo::TopoGraph>*)ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<breptopo::TopoGraph>));
    proxy->obj = std::make_shared<breptopo::TopoGraph>(shapes);
}

int w_TopoGraph_finalize(void* data)
{
    auto proxy = (wrapper::Proxy<breptopo::TopoGraph>*)(data);
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<breptopo::TopoGraph>);
}

void w_TopoGraph_get_graph()
{
    auto hg = ((wrapper::Proxy<breptopo::TopoGraph>*)ves_toforeign(0))->obj;

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("graph", "Graph");
    auto proxy = (wrapper::Proxy<graph::Graph>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<graph::Graph>));
    proxy->obj = hg->GetGraph();
    ves_pop(1);
}

void w_TopoNaming_allocate()
{
    auto proxy = (wrapper::Proxy<breptopo::TopoNaming>*)ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<breptopo::TopoNaming>));
    proxy->obj = std::make_shared<breptopo::TopoNaming>();
}

int w_TopoNaming_finalize(void* data)
{
    auto proxy = (wrapper::Proxy<breptopo::TopoNaming>*)(data);
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<breptopo::TopoNaming>);
}

void w_TopoNaming_get_vertex_graph()
{
    auto tn = ((wrapper::Proxy<breptopo::TopoNaming>*)ves_toforeign(0))->obj;

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("breptopo", "HistGraph");
    auto proxy = (wrapper::Proxy<breptopo::HistGraph>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<breptopo::HistGraph>));
    proxy->obj = tn->GetVertexGraph();
    ves_pop(1);
}

void w_TopoNaming_get_edge_graph()
{
    auto tn = ((wrapper::Proxy<breptopo::TopoNaming>*)ves_toforeign(0))->obj;

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("breptopo", "HistGraph");
    auto proxy = (wrapper::Proxy<breptopo::HistGraph>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<breptopo::HistGraph>));
    proxy->obj = tn->GetEdgeGraph();
    ves_pop(1);
}

void w_TopoNaming_get_face_graph()
{
    auto tn = ((wrapper::Proxy<breptopo::TopoNaming>*)ves_toforeign(0))->obj;

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("breptopo", "HistGraph");
    auto proxy = (wrapper::Proxy<breptopo::HistGraph>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<breptopo::HistGraph>));
    proxy->obj = tn->GetFaceGraph();
    ves_pop(1);
}

void w_TopoNaming_get_solid_graph()
{
    auto tn = ((wrapper::Proxy<breptopo::TopoNaming>*)ves_toforeign(0))->obj;

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("breptopo", "HistGraph");
    auto proxy = (wrapper::Proxy<breptopo::HistGraph>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<breptopo::HistGraph>));
    proxy->obj = tn->GetSolidGraph();
    ves_pop(1);
}

void w_TopoNaming_get_next_op_id()
{
    auto tn = ((wrapper::Proxy<breptopo::TopoNaming>*)ves_toforeign(0))->obj;
    ves_set_number(0, tn->NextOpId());
}

void w_HistGraph_allocate()
{
    auto proxy = (wrapper::Proxy<breptopo::HistGraph>*)ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<breptopo::HistGraph>));
    proxy->obj = std::make_shared<breptopo::HistGraph>();
}

int w_HistGraph_finalize(void* data)
{
    auto proxy = (wrapper::Proxy<breptopo::HistGraph>*)(data);
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<breptopo::HistGraph>);
}

void w_HistGraph_get_hist_graph()
{
    auto hg = ((wrapper::Proxy<breptopo::HistGraph>*)ves_toforeign(0))->obj;

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("graph", "Graph");
    auto proxy = (wrapper::Proxy<graph::Graph>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<graph::Graph>));
    proxy->obj = hg->GetGraph();
    ves_pop(1);
}

void w_HistGraph_get_node_uid()
{
    auto hg = ((wrapper::Proxy<breptopo::HistGraph>*)ves_toforeign(0))->obj;

    auto shape = ((wrapper::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;

    auto node = hg->QueryNode(shape);
    auto& cid = node->GetComponent<breptopo::NodeId>();
    ves_set_number(0, cid.GetUID());
}

void w_HistGraph_query_shapes()
{
    auto hg = ((wrapper::Proxy<breptopo::HistGraph>*)ves_toforeign(0))->obj;

    uint32_t uid = (uint32_t)ves_tonumber(1);

    std::vector<std::shared_ptr<graph::Node>> nodes;
    if (hg->QueryNodes(uid, nodes))
    {
        assert(!nodes.empty());

        ves_pop(ves_argnum());

        const int num = (int)nodes.size();
        ves_newlist(num);
        for (int i = 0; i < num; ++i)
        {
            ves_pushnil();
            ves_import_class("partgraph", "TopoShape");
            auto proxy = (wrapper::Proxy<partgraph::TopoShape>*)ves_set_newforeign(1, 2, sizeof(wrapper::Proxy<partgraph::TopoShape>));
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
    auto proxy = (wrapper::Proxy<breptopo::CompGraph>*)ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<breptopo::CompGraph>));
    proxy->obj = std::make_shared<breptopo::CompGraph>();
}

int w_CompGraph_finalize(void* data)
{
    auto proxy = (wrapper::Proxy<breptopo::CompGraph>*)(data);
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<breptopo::CompGraph>);
}

void w_CompGraph_get_graph()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("graph", "Graph");
    auto proxy = (wrapper::Proxy<graph::Graph>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<graph::Graph>));
    proxy->obj = breptopo::CompGraphBuilder::BuildGraph(*cg);
    ves_pop(1);
}

void w_CompGraph_get_graph_filtered()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;
    int root_step = (int)ves_tonumber(1);

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("graph", "Graph");
    auto proxy = (wrapper::Proxy<graph::Graph>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<graph::Graph>));
    proxy->obj = breptopo::CompGraphBuilder::BuildGraph(*cg, root_step);
    ves_pop(1);
}

void w_CompGraph_set_parallel()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;
    bool enabled = ves_toboolean(1);
    cg->SetParallel(enabled);
}

void w_CompGraph_eval()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int node_idx = (int)ves_tonumber(1);

    auto result = cg->Eval(node_idx);

    if (std::holds_alternative<std::monostate>(result)) {
        ves_set_nil(0);
        return;
    }

    if (auto* v = std::get_if<int>(&result)) {
        ves_set_number(0, *v);
    } else if (auto* v = std::get_if<double>(&result)) {
        ves_set_number(0, *v);
    } else if (auto* v = std::get_if<bool>(&result)) {
        ves_set_boolean(0, *v);
    } else if (auto* v = std::get_if<breptopo::ShapeVal>(&result)) {
        if (v->shape) {
            partgraph::return_topo_shape(v->shape);
        } else {
            ves_set_nil(0);
        }
    } else {
        ves_set_nil(0);
    }
}

void w_CompGraph_add_integer_node()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int val = (int)ves_tonumber(1);
    const char* desc = ves_tostring(2);

    int id = cg->AddConst(val, desc);
    ves_set_number(0, id);
}

void w_CompGraph_add_number_node()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    double num = ves_tonumber(1);
    const char* desc = ves_tostring(2);

    int id = cg->AddConst(num, desc);
    ves_set_number(0, id);
}

void w_CompGraph_add_number3_node()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    double x = ves_tonumber(1);
    double y = ves_tonumber(2);
    double z = ves_tonumber(3);
    const char* desc = ves_tostring(4);

    int id = cg->AddConst(breptopo::Vec3{x, y, z}, desc);
    ves_set_number(0, id);
}

void w_CompGraph_add_boolean_node()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    bool b = ves_toboolean(1);
    const char* desc = ves_tostring(2);

    int id = cg->AddConst(b, desc);
    ves_set_number(0, id);
}

void w_CompGraph_add_shape_node()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    auto shape = ((wrapper::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    const char* desc = ves_tostring(2);

    int id = cg->AddConst(shape, desc);
    ves_set_number(0, id);
}

void w_CompGraph_add_box_node()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int length = (int)ves_tonumber(1);
    int width  = (int)ves_tonumber(2);
    int height = (int)ves_tonumber(3);

    int id = cg->AddOp("box", {length, width, height}, {}, "box op");
    ves_set_number(0, id);
}

void w_CompGraph_add_translate_node()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int shape = (int)ves_tonumber(1);
    int offset = (int)ves_tonumber(2);

    int id = cg->AddOp("translate", {shape, offset}, {}, "translate op");
    ves_set_number(0, id);
}

void w_CompGraph_add_offset_node()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int shape = (int)ves_tonumber(1);
    int offset = (int)ves_tonumber(2);
    int is_solid = (int)ves_tonumber(3);

    int id = cg->AddOp("offset", {shape, offset, is_solid}, {}, "offset op");
    ves_set_number(0, id);
}

void w_CompGraph_add_cut_node()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int shp1 = (int)ves_tonumber(1);
    int shp2 = (int)ves_tonumber(2);

    int id = cg->AddOp("cut", {shp1, shp2}, {}, "cut op");
    ves_set_number(0, id);
}

void w_CompGraph_add_selector_node()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int shp = (int)ves_tonumber(1);
    int uid = (int)ves_tonumber(2);

    int id = cg->AddOp("selector", {shp, uid}, {}, "selector");
    ves_set_number(0, id);
}

void w_CompGraph_add_merge_node()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    auto nodes = wrapper::list_to_array<int>(1);

    int id = cg->AddOp("merge", {}, std::vector<int>(nodes.begin(), nodes.end()), "merge");
    ves_set_number(0, id);
}

void w_CompGraph_update_integer_node()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int node_idx = (int)ves_tonumber(1);
    int val = (int)ves_tonumber(2);

    cg->UpdateConst(node_idx, val);
}

void w_CompGraph_update_number_node()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int node_idx = (int)ves_tonumber(1);
    double val = ves_tonumber(2);

    cg->UpdateConst(node_idx, val);
}

void w_CompGraph_update_number3_node()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int node_idx = (int)ves_tonumber(1);
    double x = ves_tonumber(2);
    double y = ves_tonumber(3);
    double z = ves_tonumber(4);

    cg->UpdateConst(node_idx, breptopo::Vec3{x, y, z});
}

void w_CompGraph_update_boolean_node()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int node_idx = (int)ves_tonumber(1);
    bool val = ves_toboolean(2);

    cg->UpdateConst(node_idx, val);
}

void w_CompGraph_update_shape_node()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int node_idx = (int)ves_tonumber(1);
    auto shape = ((wrapper::Proxy<partgraph::TopoShape>*)ves_toforeign(2))->obj;

    breptopo::ShapeVal sv;
    sv.shape = shape;
    cg->UpdateConst(node_idx, std::move(sv));
}

// generic: add_op(op_name, inputs_list)
void w_CompGraph_add_op()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    const char* op_name = ves_tostring(1);
    auto inputs = wrapper::list_to_array<int>(2);

    int id = cg->AddOp(op_name, inputs, {}, op_name);
    ves_set_number(0, id);
}

// generic: add_op_v(op_name, inputs_list, var_inputs_list)
void w_CompGraph_add_op_v()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    const char* op_name = ves_tostring(1);
    auto inputs = wrapper::list_to_array<int>(2);
    auto var_inputs = wrapper::list_to_array<int>(3);

    int id = cg->AddOp(op_name, inputs, var_inputs, op_name);
    ves_set_number(0, id);
}

// generic: update_const(node_idx, value)
// value type is detected automatically
void w_CompGraph_update_const()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int node_idx = (int)ves_tonumber(1);

    if (ves_type(2) == VES_TYPE_NUM) {
        cg->UpdateConst(node_idx, ves_tonumber(2));
    } else if (ves_type(2) == VES_TYPE_BOOL) {
        cg->UpdateConst(node_idx, (bool)ves_toboolean(2));
    } else if (ves_type(2) == VES_TYPE_FOREIGN) {
        auto shape = ((wrapper::Proxy<partgraph::TopoShape>*)ves_toforeign(2))->obj;
        breptopo::ShapeVal sv;
        sv.shape = shape;
        cg->UpdateConst(node_idx, std::move(sv));
    }
}

void w_CompGraph_truncate()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;

    int keep = (int)ves_tonumber(1);
    if (keep < 0) keep = 0;
    cg->Truncate(static_cast<size_t>(keep));
}

void w_CompGraph_get_history_size()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;
    ves_set_number(0, static_cast<double>(cg->GetHistorySize()));
}

void w_CompGraph_get_step_op_name()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;
    int step_id = (int)ves_tonumber(1);
    auto& name = cg->GetStepOpName(step_id);
    ves_set_string(0, name.c_str(), name.size());
}

void w_CompGraph_get_step_inputs()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;
    int step_id = (int)ves_tonumber(1);
    auto inputs = cg->GetStepInputs(step_id);

    ves_pop(ves_argnum());
    ves_newlist(static_cast<int>(inputs.size()));
    for (int i = 0; i < (int)inputs.size(); ++i)
    {
        ves_pushnumber(inputs[i]);
        ves_seti(-2, i);
        ves_pop(1);
    }
}

void w_CompGraph_claim_step()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;
    int step_id = (int)ves_tonumber(1);
    cg->ClaimStep(step_id);
}

void w_CompGraph_is_step_claimed()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;
    int step_id = (int)ves_tonumber(1);
    ves_set_boolean(0, cg->IsStepClaimed(step_id));
}

void w_CompGraph_has_preloaded_history()
{
    auto cg = ((wrapper::Proxy<breptopo::CompGraph>*)ves_toforeign(0))->obj;
    ves_set_boolean(0, cg->HasPreloadedHistory());
}

}

namespace breptopo
{

VesselForeignMethodFn BrepTopoBindMethod(const char* signature)
{
    if (strcmp(signature, "TopoGraph.get_graph()") == 0) return w_TopoGraph_get_graph;

    if (strcmp(signature, "TopoNaming.get_vertex_graph()") == 0) return w_TopoNaming_get_vertex_graph;
    if (strcmp(signature, "TopoNaming.get_edge_graph()") == 0) return w_TopoNaming_get_edge_graph;
    if (strcmp(signature, "TopoNaming.get_face_graph()") == 0) return w_TopoNaming_get_face_graph;
    if (strcmp(signature, "TopoNaming.get_solid_graph()") == 0) return w_TopoNaming_get_solid_graph;
    if (strcmp(signature, "TopoNaming.get_next_op_id()") == 0) return w_TopoNaming_get_next_op_id;

    if (strcmp(signature, "HistGraph.get_hist_graph()") == 0) return w_HistGraph_get_hist_graph;
    if (strcmp(signature, "HistGraph.get_node_uid(_)") == 0) return w_HistGraph_get_node_uid;
    if (strcmp(signature, "HistGraph.query_shapes(_)") == 0) return w_HistGraph_query_shapes;

    if (strcmp(signature, "CompGraph.get_graph()") == 0) return w_CompGraph_get_graph;
    if (strcmp(signature, "CompGraph.get_graph(_)") == 0) return w_CompGraph_get_graph_filtered;
    if (strcmp(signature, "CompGraph.eval(_)") == 0) return w_CompGraph_eval;
    if (strcmp(signature, "CompGraph.set_parallel(_)") == 0) return w_CompGraph_set_parallel;
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

    if (strcmp(signature, "CompGraph.add_op(_,_)") == 0) return w_CompGraph_add_op;
    if (strcmp(signature, "CompGraph.add_op_v(_,_,_)") == 0) return w_CompGraph_add_op_v;
    if (strcmp(signature, "CompGraph.update_const(_,_)") == 0) return w_CompGraph_update_const;
    if (strcmp(signature, "CompGraph.truncate(_)") == 0) return w_CompGraph_truncate;

    if (strcmp(signature, "CompGraph.get_history_size()") == 0) return w_CompGraph_get_history_size;
    if (strcmp(signature, "CompGraph.get_step_op_name(_)") == 0) return w_CompGraph_get_step_op_name;
    if (strcmp(signature, "CompGraph.get_step_inputs(_)") == 0) return w_CompGraph_get_step_inputs;
    if (strcmp(signature, "CompGraph.claim_step(_)") == 0) return w_CompGraph_claim_step;
    if (strcmp(signature, "CompGraph.is_step_claimed(_)") == 0) return w_CompGraph_is_step_claimed;
    if (strcmp(signature, "CompGraph.has_preloaded_history()") == 0) return w_CompGraph_has_preloaded_history;

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

    if (strcmp(class_name, "TopoNaming") == 0)
    {
        methods->allocate = w_TopoNaming_allocate;
        methods->finalize = w_TopoNaming_finalize;
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