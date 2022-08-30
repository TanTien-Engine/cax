#include "wrap_PartGraph.h"
#include "TopoShape.h"
#include "MeshBuilder.h"
#include "PrimMaker.h"
#include "TopoAlgo.h"
#include "modules/script/TransHelper.h"

#include <logger/logger.h>

// OCCT
#include <Precision.hxx>

namespace
{

void return_topo_shape(const std::shared_ptr<partgraph::TopoShape>& shape)
{
    if (!shape) {
        ves_set_nil(0);
        return;
    }

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("partgraph", "TopoShape");
    auto proxy = (tt::Proxy<partgraph::TopoShape>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<partgraph::TopoShape>));
    proxy->obj = shape;
    ves_pop(1);
}

void w_PrimMaker_box()
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

    auto shape = partgraph::PrimMaker::Box(L, W, H);
    return_topo_shape(shape);
}

void w_PrimMaker_cylinder()
{
    double radius = ves_tonumber(1);
    double length = ves_tonumber(2);

    bool skip = false;
    if (radius < Precision::Confusion()) {
        LOGI("Radius of Cylinder too small");
        skip = true;
    }
    if (length < Precision::Confusion()) {
        LOGI("Length of Cylinder too small");
        skip = true;
    }
    if (skip) {
        ves_set_nil(0);
        return;
    }

    auto shape = partgraph::PrimMaker::Cylinder(radius, length);
    return_topo_shape(shape);
}

void w_TopoAlgo_fillet()
{
    auto topo = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    double thickness = ves_tonumber(2);

    auto shape = partgraph::TopoAlgo::Fillet(topo, thickness);
    return_topo_shape(shape);
}

void w_TopoAlgo_chamfer()
{
    auto topo = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    double dist = ves_tonumber(2);

    auto shape = partgraph::TopoAlgo::Chamfer(topo, dist);
    return_topo_shape(shape);
}

void w_TopoAlgo_cut()
{
    auto s1 = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    auto s2 = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(2))->obj;
    auto shape = partgraph::TopoAlgo::Cut(s1, s2);
    return_topo_shape(shape);
}

void w_TopoAlgo_fuse()
{
    auto s1 = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    auto s2 = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(2))->obj;
    auto shape = partgraph::TopoAlgo::Fuse(s1, s2);
    return_topo_shape(shape);
}

void w_TopoAlgo_section()
{
    auto s1 = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    auto s2 = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(2))->obj;
    auto shape = partgraph::TopoAlgo::Section(s1, s2);
    return_topo_shape(shape);
}

void w_TopoAlgo_translate()
{
    auto src = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    double x = ves_tonumber(2);
    double y = ves_tonumber(3);
    double z = ves_tonumber(4);
    auto dst = partgraph::TopoAlgo::Translate(src, x, y, z);
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
    if (strcmp(signature, "static PrimMaker.box(_,_,_)") == 0) return w_PrimMaker_box;
    if (strcmp(signature, "static PrimMaker.cylinder(_,_)") == 0) return w_PrimMaker_cylinder;

    if (strcmp(signature, "static TopoAlgo.fillet(_,_)") == 0) return w_TopoAlgo_fillet;
    if (strcmp(signature, "static TopoAlgo.chamfer(_,_)") == 0) return w_TopoAlgo_chamfer;
    if (strcmp(signature, "static TopoAlgo.cut(_,_)") == 0) return w_TopoAlgo_cut;
    if (strcmp(signature, "static TopoAlgo.fuse(_,_)") == 0) return w_TopoAlgo_fuse;
    if (strcmp(signature, "static TopoAlgo.section(_,_)") == 0) return w_TopoAlgo_section;
    if (strcmp(signature, "static TopoAlgo.translate(_,_,_,_)") == 0) return w_TopoAlgo_translate;

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