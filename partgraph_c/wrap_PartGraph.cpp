#include "wrap_PartGraph.h"
#include "TopoDataset.h"
#include "TopoAdapter.h"
#include "PrimMaker.h"
#include "BRepBuilder.h"
#include "TopoAlgo.h"
#include "modules/script/TransHelper.h"

#include <logger/logger.h>
#include <geoshape/Line3D.h>

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

void w_BRepBuilder_make_edge_from_line()
{
    auto line = ((tt::Proxy<gs::Line3D>*)ves_toforeign(1))->obj;

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("partgraph", "TopoEdge");
    auto proxy = (tt::Proxy<partgraph::TopoEdge>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<partgraph::TopoEdge>));
    proxy->obj = partgraph::BRepBuilder::MakeEdge(*line);
    ves_pop(1);
}

void w_BRepBuilder_make_edge_from_arc()
{
    auto arc = ((tt::Proxy<gs::Arc>*)ves_toforeign(1))->obj;

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("partgraph", "TopoEdge");
    auto proxy = (tt::Proxy<partgraph::TopoEdge>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<partgraph::TopoEdge>));
    proxy->obj = partgraph::BRepBuilder::MakeEdge(*arc);
    ves_pop(1);
}

void w_BRepBuilder_make_wire()
{
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
    auto src = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    double thickness = ves_tonumber(2);

    auto dst = partgraph::TopoAlgo::Fillet(src, thickness);
    return_topo_shape(dst);
}

void w_TopoAlgo_chamfer()
{
    auto src = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    double dist = ves_tonumber(2);

    auto dst = partgraph::TopoAlgo::Chamfer(src, dist);
    return_topo_shape(dst);
}

void w_TopoAlgo_extrude()
{
    auto src = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    double x = ves_tonumber(2);
    double y = ves_tonumber(3);
    double z = ves_tonumber(4);
    auto dst = partgraph::TopoAlgo::Prism(src, x, y, z);
    return_topo_shape(dst);
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

void w_TopoAlgo_common()
{
    auto s1 = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    auto s2 = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(2))->obj;
    auto shape = partgraph::TopoAlgo::Common(s1, s2);
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

void w_TopoAdapter_build_mesh()
{
    auto shape = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;

    auto va = partgraph::TopoAdapter::BuildMesh(*shape);

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("render", "VertexArray");
    auto proxy = (tt::Proxy<ur::VertexArray>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<ur::VertexArray>));
    proxy->obj = va;
    ves_pop(1);
}

void w_TopoAdapter_build_edge_geo()
{
    auto edge = ((tt::Proxy<partgraph::TopoEdge>*)ves_toforeign(1))->obj;

    auto geo = partgraph::TopoAdapter::BuildGeo(*edge);

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("geometry", "Line3D");
    auto proxy = (tt::Proxy<gs::Line3D>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<gs::Line3D>));
    proxy->obj = geo;
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

void w_TopoEdge_allocate()
{
    auto proxy = (tt::Proxy<partgraph::TopoEdge>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<partgraph::TopoEdge>));
    proxy->obj = std::make_shared<partgraph::TopoEdge>();
}

int w_TopoEdge_finalize(void* data)
{
    auto proxy = (tt::Proxy<partgraph::TopoEdge>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<partgraph::TopoEdge>);
}

void w_TopoWire_allocate()
{
    auto proxy = (tt::Proxy<partgraph::TopoWire>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<partgraph::TopoWire>));
    proxy->obj = std::make_shared<partgraph::TopoWire>();
}

int w_TopoWire_finalize(void* data)
{
    auto proxy = (tt::Proxy<partgraph::TopoWire>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<partgraph::TopoWire>);
}

}

namespace partgraph
{

VesselForeignMethodFn PartGraphBindMethod(const char* signature)
{
    if (strcmp(signature, "static BRepBuilder.make_edge_from_line(_)") == 0) return w_BRepBuilder_make_edge_from_line;
    if (strcmp(signature, "static BRepBuilder.make_edge_from_arc(_)") == 0) return w_BRepBuilder_make_edge_from_arc;
    if (strcmp(signature, "static BRepBuilder.make_wire(_)") == 0) return w_BRepBuilder_make_wire;

    if (strcmp(signature, "static PrimMaker.box(_,_,_)") == 0) return w_PrimMaker_box;
    if (strcmp(signature, "static PrimMaker.cylinder(_,_)") == 0) return w_PrimMaker_cylinder;

    if (strcmp(signature, "static TopoAlgo.fillet(_,_)") == 0) return w_TopoAlgo_fillet;
    if (strcmp(signature, "static TopoAlgo.chamfer(_,_)") == 0) return w_TopoAlgo_chamfer;
    if (strcmp(signature, "static TopoAlgo.extrude(_,_,_,_)") == 0) return w_TopoAlgo_extrude;
    if (strcmp(signature, "static TopoAlgo.cut(_,_)") == 0) return w_TopoAlgo_cut;
    if (strcmp(signature, "static TopoAlgo.fuse(_,_)") == 0) return w_TopoAlgo_fuse;
    if (strcmp(signature, "static TopoAlgo.common(_,_)") == 0) return w_TopoAlgo_common;
    if (strcmp(signature, "static TopoAlgo.section(_,_)") == 0) return w_TopoAlgo_section;
    if (strcmp(signature, "static TopoAlgo.translate(_,_,_,_)") == 0) return w_TopoAlgo_translate;

    if (strcmp(signature, "static TopoAdapter.build_mesh(_)") == 0) return w_TopoAdapter_build_mesh;
    if (strcmp(signature, "static TopoAdapter.build_edge_geo(_)") == 0) return w_TopoAdapter_build_edge_geo;

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

    if (strcmp(class_name, "TopoEdge") == 0)
    {
        methods->allocate = w_TopoEdge_allocate;
        methods->finalize = w_TopoEdge_finalize;
        return;
    }

    if (strcmp(class_name, "TopoWire") == 0)
    {
        methods->allocate = w_TopoWire_allocate;
        methods->finalize = w_TopoWire_finalize;
        return;
    }
}

}