#include "wrap_PartGraph.h"
#include "TopoShape.h"
#include "MeshBuilder.h"
#include "modules/script/TransHelper.h"

#include <logger/logger.h>

// OCCT
#include <Precision.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>

namespace
{

void return_topo_shape(const std::shared_ptr<partgraph::TopoShape>& shape)
{
    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("partgraph", "TopoShape");
    auto proxy = (tt::Proxy<partgraph::TopoShape>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<partgraph::TopoShape>));
    proxy->obj = shape;
    ves_pop(1);
}

void w_BRepPrimAPI_box()
{
    double L = ves_tonumber(1);
    double W = ves_tonumber(2);
    double H = ves_tonumber(3);

    bool skip = false;
    if (L < Precision::Confusion()) {
        LOGI("Length of box too small");
        skip = true;
    }
    if (W < Precision::Confusion()) {
        LOGI("Width of box too small");
        skip = true;
    }
    if (H < Precision::Confusion()) {
        LOGI("Height of box too small");
        skip = true;
    }
    if (skip) {
        ves_set_nil(0);
        return;
    }

    std::shared_ptr<partgraph::TopoShape> shape = nullptr;
    try {
        // Build a box using the dimension attributes
        BRepPrimAPI_MakeBox mkBox(L, W, H);
        shape = std::make_shared<partgraph::TopoShape>(mkBox.Shape());
    } catch (Standard_Failure& e) {
        LOGI("Build box fail: %s", e.GetMessageString());
        ves_set_nil(0);
        return;
    }

    return_topo_shape(shape);
}

void w_BRepFilletAPI_make_fillet()
{
    auto topo = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    double thickness = ves_tonumber(2);

    auto src = topo->GetShape();
    BRepFilletAPI_MakeFillet fillet(src);
    TopExp_Explorer edge_explorer(src, TopAbs_EDGE);
    while (edge_explorer.More())
    {
        TopoDS_Edge edge = TopoDS::Edge(edge_explorer.Current());
        //Add edge to fillet algorithm
        fillet.Add(thickness / 12., edge);
        edge_explorer.Next();
    }

    auto dst = std::make_shared<partgraph::TopoShape>(fillet.Shape());

    return_topo_shape(dst);
}

void w_MeshBuilder_build_from_topo()
{
    auto topo = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;

    auto va = partgraph::MeshBuilder::Build(*topo);

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("render", "VertexArray");
    auto proxy = (tt::Proxy<ur::VertexArray>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<ur::VertexArray>));
    proxy->obj = va;
    ves_pop(1);
}

void w_TopoShape_allocate()
{
    auto proxy = (tt::Proxy<partgraph::TopoShape>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<partgraph::TopoShape>));
    proxy->obj = std::make_shared<partgraph::TopoShape>();
}

int w_TopoShape_finalize(void* data)
{
    auto proxy = (tt::Proxy<partgraph::TopoShape>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<partgraph::TopoShape>);
}

}

namespace partgraph
{

VesselForeignMethodFn PartGraphBindMethod(const char* signature)
{
    if (strcmp(signature, "static BRepPrimAPI.box(_,_,_)") == 0) return w_BRepPrimAPI_box;

    if (strcmp(signature, "static BRepFilletAPI.make_fillet(_,_)") == 0) return w_BRepFilletAPI_make_fillet;

    if (strcmp(signature, "static MeshBuilder.build_from_topo(_)") == 0) return w_MeshBuilder_build_from_topo;

    return nullptr;
}

void PartGraphBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
    if (strcmp(class_name, "TopoShape") == 0)
    {
        methods->allocate = w_TopoShape_allocate;
        methods->finalize = w_TopoShape_finalize;
        return;
    }
}

}