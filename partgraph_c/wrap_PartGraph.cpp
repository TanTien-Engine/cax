#include "wrap_PartGraph.h"
#include "TopoDataset.h"
#include "TopoAdapter.h"
#include "PrimMaker.h"
#include "BRepBuilder.h"
#include "TopoAlgo.h"
#include "BRepSelector.h"
#include "GeomDataset.h"
#include "modules/script/TransHelper.h"

#include <logger/logger.h>
#include <geoshape/Line3D.h>

// OCCT
#include <Precision.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>

namespace
{

template<typename T>
void return_topo_obj(const std::shared_ptr<T>& obj, const char* class_name)
{
    if (!obj) {
        ves_set_nil(0);
        return;
    }

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("partgraph", class_name);
    auto proxy = (tt::Proxy<T>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<T>));
    proxy->obj = obj;
    ves_pop(1);
}

void return_topo_shape(const std::shared_ptr<partgraph::TopoShape>& shape) {
    return return_topo_obj<partgraph::TopoShape>(shape, "TopoShape");
}

void return_topo_edge(const std::shared_ptr<partgraph::TopoEdge>& edge) {
    return return_topo_obj<partgraph::TopoEdge>(edge, "TopoEdge");
}

void return_topo_wire(const std::shared_ptr<partgraph::TopoWire>& wire) {
    return return_topo_obj<partgraph::TopoWire>(wire, "TopoWire");
}

void return_topo_face(const std::shared_ptr<partgraph::TopoFace>& face) {
    return return_topo_obj<partgraph::TopoFace>(face, "TopoFace");
}

void w_BRepBuilder_make_edge_from_line()
{
    auto line = ((tt::Proxy<gs::Line3D>*)ves_toforeign(1))->obj;
    auto edge = partgraph::BRepBuilder::MakeEdge(*line);
    return_topo_edge(edge);
}

void w_BRepBuilder_make_edge_from_arc()
{
    auto arc = ((tt::Proxy<gs::Arc3D>*)ves_toforeign(1))->obj;
    auto edge = partgraph::BRepBuilder::MakeEdge(*arc);
    return_topo_edge(edge);
}

void w_BRepBuilder_make_edge_from_curve_surf()
{
    auto c = ((tt::Proxy<partgraph::TrimmedCurve>*)ves_toforeign(1))->obj;
    auto s = ((tt::Proxy<partgraph::CylindricalSurface>*)ves_toforeign(2))->obj;
    auto edge = partgraph::BRepBuilder::MakeEdge(*c, *s);
    return_topo_edge(edge);
}

void w_BRepBuilder_make_wire()
{
    std::vector<std::shared_ptr<partgraph::TopoEdge>> edges;
    tt::list_to_foreigns(1, edges);
    auto wire = partgraph::BRepBuilder::MakeWire(edges);
    return_topo_wire(wire);
}

void w_BRepBuilder_make_face()
{
    auto wire = ((tt::Proxy<partgraph::TopoWire>*)ves_toforeign(1))->obj;
    auto face = partgraph::BRepBuilder::MakeFace(*wire);
    return_topo_face(face);
}

void w_BRepBuilder_make_compound()
{
    std::vector<std::shared_ptr<partgraph::TopoShape>> shapes;
    tt::list_to_foreigns(1, shapes);
    auto shape = partgraph::BRepBuilder::MakeCompound(shapes);
    return_topo_shape(shape);
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

void w_PrimMaker_cone()
{
    double radius1 = ves_tonumber(1);
    double radius2 = ves_tonumber(2);
    double height = ves_tonumber(3);

    bool skip = false;
    if (radius1 < Precision::Confusion()) {
        LOGI("Radius1 of Cone too small");
        skip = true;
    }
    if (radius2 < Precision::Confusion()) {
        LOGI("Radius2 of Cone too small");
        skip = true;
    }
    if (height < Precision::Confusion()) {
        LOGI("Height of Cone too small");
        skip = true;
    }
    if (skip) {
        ves_set_nil(0);
        return;
    }

    auto shape = partgraph::PrimMaker::Cone(radius1, radius2, height);
    return_topo_shape(shape);
}

void w_PrimMaker_sphere()
{
    double radius = ves_tonumber(1);

    bool skip = false;
    if (radius < Precision::Confusion()) {
        LOGI("Radius of sphere too small");
        skip = true;
    }
    if (skip) {
        ves_set_nil(0);
        return;
    }

    auto shape = partgraph::PrimMaker::Sphere(radius);
    return_topo_shape(shape);
}

void w_PrimMaker_sphere_with_angle()
{
    double radius = ves_tonumber(1);
    double angle = ves_tonumber(2);

    bool skip = false;
    if (radius < Precision::Confusion()) {
        LOGI("Radius of sphere too small");
        skip = true;
    }
    if (angle < Precision::Confusion()) {
        LOGI("Angle of sphere too small");
        skip = true;
    }
    if (skip) {
        ves_set_nil(0);
        return;
    }

    auto shape = partgraph::PrimMaker::Sphere(radius, angle);
    return_topo_shape(shape);
}

void w_PrimMaker_torus()
{
    double r1 = ves_tonumber(1);
    double r2 = ves_tonumber(2);

    bool skip = false;
    if (r1 < Precision::Confusion()) {
        LOGI("R1 of torus too small");
        skip = true;
    }
    if (r2 < Precision::Confusion()) {
        LOGI("R2 of torus too small");
        skip = true;
    }
    if (skip) {
        ves_set_nil(0);
        return;
    }

    auto shape = partgraph::PrimMaker::Torus(r1, r2);
    return_topo_shape(shape);
}

void w_PrimMaker_torus_with_angle()
{
    double r1 = ves_tonumber(1);
    double r2 = ves_tonumber(2);
    double angle = ves_tonumber(3);

    bool skip = false;
    if (r1 < Precision::Confusion()) {
        LOGI("R1 of torus too small");
        skip = true;
    }
    if (r2 < Precision::Confusion()) {
        LOGI("R2 of torus too small");
        skip = true;
    }
    if (angle < Precision::Confusion()) {
        LOGI("Angle of torus too small");
        skip = true;
    }
    if (skip) {
        ves_set_nil(0);
        return;
    }

    auto shape = partgraph::PrimMaker::Torus(r1, r2, angle);
    return_topo_shape(shape);
}

void w_PrimMaker_threading()
{
    double thickness = ves_tonumber(1);
    double height = ves_tonumber(2);
    auto shape = partgraph::PrimMaker::Threading(thickness, height);
    return_topo_shape(shape);
}

void w_BRepSelector_select_face()
{
    auto shape = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    auto face = partgraph::BRepSelector::SelectFace(shape);
    return_topo_face(face);
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
    auto src = ((tt::Proxy<partgraph::TopoFace>*)ves_toforeign(1))->obj;
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

void w_TopoAlgo_mirror()
{
    auto src = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    auto pos = tt::list_to_vec3(2);
    auto dir = tt::list_to_vec3(3);
    auto dst = partgraph::TopoAlgo::Mirror(src, pos, dir);
    return_topo_shape(dst);
}

void w_TopoAlgo_draft()
{
    auto src = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    auto dir = tt::list_to_vec3(2);
    auto angle = (float)ves_tonumber(3);
    auto len_max = (float)ves_tonumber(4);
    auto dst = partgraph::TopoAlgo::Draft(src, dir, angle, len_max);
    return_topo_shape(dst);
}

void w_TopoAlgo_thick_solid()
{
    auto shape = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    auto face = ((tt::Proxy<partgraph::TopoFace>*)ves_toforeign(2))->obj;
    auto offset = (float)ves_tonumber(3);
    auto dst = partgraph::TopoAlgo::ThickSolid(shape, face, offset);
    return_topo_shape(dst);
}

void w_TopoAlgo_thru_sections()
{
    std::vector<std::shared_ptr<partgraph::TopoWire>> wires;
    tt::list_to_foreigns(1, wires);
    auto shape = partgraph::TopoAlgo::ThruSections(wires);
    return_topo_shape(shape);
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

void w_TopoAdapter_build_wire_geo()
{
    auto wire = ((tt::Proxy<partgraph::TopoWire>*)ves_toforeign(1))->obj;

    auto geo = partgraph::TopoAdapter::BuildGeo(*wire);

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("geometry", "Polyline3D");
    auto proxy = (tt::Proxy<gs::Polyline3D>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<gs::Polyline3D>));
    proxy->obj = geo;
    ves_pop(1);
}

void w_TopoAdapter_shape2wire()
{
    auto shape = ((tt::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    auto wire = partgraph::TopoAdapter::ToWire(*shape);
    return_topo_wire(wire);
}

void w_CylindricalSurface_allocate()
{
    auto pos = tt::list_to_vec3(1);
    auto dir = tt::list_to_vec3(2);
    float radius = (float)ves_tonumber(3);

    auto proxy = (tt::Proxy<partgraph::CylindricalSurface>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<partgraph::CylindricalSurface>));
    proxy->obj = std::make_shared<partgraph::CylindricalSurface>(pos, dir, radius);
}

int w_CylindricalSurface_finalize(void* data)
{
    auto proxy = (tt::Proxy<partgraph::CylindricalSurface>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<partgraph::CylindricalSurface>);
}

void w_EllipseCurve_allocate()
{
    auto pos = tt::list_to_vec2(1);
    auto dir = tt::list_to_vec2(2);
    float major_radius = (float)ves_tonumber(3);
    float minor_radius = (float)ves_tonumber(4);

    auto proxy = (tt::Proxy<partgraph::EllipseCurve>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<partgraph::EllipseCurve>));
    proxy->obj = std::make_shared<partgraph::EllipseCurve>(pos, dir, major_radius, minor_radius);
}

int w_EllipseCurve_finalize(void* data)
{
    auto proxy = (tt::Proxy<partgraph::EllipseCurve>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<partgraph::EllipseCurve>);
}

void w_TrimmedCurve_allocate()
{
    auto ellipse = ((tt::Proxy<partgraph::EllipseCurve>*)ves_toforeign(1))->obj;
    float u1 = (float)ves_tonumber(2);
    float u2 = (float)ves_tonumber(3);

    auto proxy = (tt::Proxy<partgraph::TrimmedCurve>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<partgraph::TrimmedCurve>));
    proxy->obj = std::make_shared<partgraph::TrimmedCurve>(*ellipse, u1, u2);
}

int w_TrimmedCurve_finalize(void* data)
{
    auto proxy = (tt::Proxy<partgraph::TrimmedCurve>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<partgraph::TrimmedCurve>);
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

void w_TopoFace_allocate()
{
    auto proxy = (tt::Proxy<partgraph::TopoFace>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<partgraph::TopoFace>));
    proxy->obj = std::make_shared<partgraph::TopoFace>();
}

int w_TopoFace_finalize(void* data)
{
    auto proxy = (tt::Proxy<partgraph::TopoFace>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<partgraph::TopoFace>);
}

void w_WireBuilder_allocate()
{
    auto proxy = (tt::Proxy<BRepBuilderAPI_MakeWire>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<BRepBuilderAPI_MakeWire>));
    proxy->obj = std::make_shared<BRepBuilderAPI_MakeWire>();
}

int w_WireBuilder_finalize(void* data)
{
    auto proxy = (tt::Proxy<BRepBuilderAPI_MakeWire>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<BRepBuilderAPI_MakeWire>);
}

void w_WireBuilder_add_edge()
{
    auto builder = ((tt::Proxy<BRepBuilderAPI_MakeWire>*)ves_toforeign(0))->obj;
    auto edge = ((tt::Proxy<partgraph::TopoEdge>*)ves_toforeign(1))->obj;
    builder->Add(edge->GetEdge());
}

void w_WireBuilder_add_wire()
{
    auto builder = ((tt::Proxy<BRepBuilderAPI_MakeWire>*)ves_toforeign(0))->obj;
    auto wire = ((tt::Proxy<partgraph::TopoWire>*)ves_toforeign(1))->obj;
    builder->Add(wire->GetWire());
}

void w_WireBuilder_gen_wire()
{
    auto builder = ((tt::Proxy<BRepBuilderAPI_MakeWire>*)ves_toforeign(0))->obj;
    if (!builder->IsDone()) {
        ves_set_nil(0);
        return;
    }

    auto wire = std::make_shared<partgraph::TopoWire>(builder->Wire());
    return_topo_wire(wire);
}

}

namespace partgraph
{

VesselForeignMethodFn PartGraphBindMethod(const char* signature)
{
    if (strcmp(signature, "WireBuilder.add_edge(_)") == 0) return w_WireBuilder_add_edge;
    if (strcmp(signature, "WireBuilder.add_wire(_)") == 0) return w_WireBuilder_add_wire;
    if (strcmp(signature, "WireBuilder.gen_wire()") == 0) return w_WireBuilder_gen_wire;

    if (strcmp(signature, "static BRepBuilder.make_edge_from_line(_)") == 0) return w_BRepBuilder_make_edge_from_line;
    if (strcmp(signature, "static BRepBuilder.make_edge_from_arc(_)") == 0) return w_BRepBuilder_make_edge_from_arc;
    if (strcmp(signature, "static BRepBuilder.make_edge_from_curve_surf(_,_)") == 0) return w_BRepBuilder_make_edge_from_curve_surf;
    if (strcmp(signature, "static BRepBuilder.make_wire(_)") == 0) return w_BRepBuilder_make_wire;
    if (strcmp(signature, "static BRepBuilder.make_face(_)") == 0) return w_BRepBuilder_make_face;
    if (strcmp(signature, "static BRepBuilder.make_compound(_)") == 0) return w_BRepBuilder_make_compound;

    if (strcmp(signature, "static PrimMaker.box(_,_,_)") == 0) return w_PrimMaker_box;
    if (strcmp(signature, "static PrimMaker.cylinder(_,_)") == 0) return w_PrimMaker_cylinder;
    if (strcmp(signature, "static PrimMaker.cone(_,_,_)") == 0) return w_PrimMaker_cone;
    if (strcmp(signature, "static PrimMaker.sphere(_)") == 0) return w_PrimMaker_sphere;
    if (strcmp(signature, "static PrimMaker.sphere_with_angle(_,_)") == 0) return w_PrimMaker_sphere_with_angle;
    if (strcmp(signature, "static PrimMaker.torus(_,_)") == 0) return w_PrimMaker_torus;
    if (strcmp(signature, "static PrimMaker.torus_with_angle(_,_,_)") == 0) return w_PrimMaker_torus_with_angle;
    if (strcmp(signature, "static PrimMaker.threading(_,_)") == 0) return w_PrimMaker_threading;

    if (strcmp(signature, "static BRepSelector.select_face(_)") == 0) return w_BRepSelector_select_face;

    if (strcmp(signature, "static TopoAlgo.fillet(_,_)") == 0) return w_TopoAlgo_fillet;
    if (strcmp(signature, "static TopoAlgo.chamfer(_,_)") == 0) return w_TopoAlgo_chamfer;
    if (strcmp(signature, "static TopoAlgo.extrude(_,_,_,_)") == 0) return w_TopoAlgo_extrude;
    if (strcmp(signature, "static TopoAlgo.cut(_,_)") == 0) return w_TopoAlgo_cut;
    if (strcmp(signature, "static TopoAlgo.fuse(_,_)") == 0) return w_TopoAlgo_fuse;
    if (strcmp(signature, "static TopoAlgo.common(_,_)") == 0) return w_TopoAlgo_common;
    if (strcmp(signature, "static TopoAlgo.section(_,_)") == 0) return w_TopoAlgo_section;
    if (strcmp(signature, "static TopoAlgo.translate(_,_,_,_)") == 0) return w_TopoAlgo_translate;
    if (strcmp(signature, "static TopoAlgo.mirror(_,_,_)") == 0) return w_TopoAlgo_mirror;
    if (strcmp(signature, "static TopoAlgo.draft(_,_,_,_)") == 0) return w_TopoAlgo_draft;
    if (strcmp(signature, "static TopoAlgo.thick_solid(_,_,_)") == 0) return w_TopoAlgo_thick_solid;
    if (strcmp(signature, "static TopoAlgo.thru_sections(_)") == 0) return w_TopoAlgo_thru_sections;

    if (strcmp(signature, "static TopoAdapter.build_mesh(_)") == 0) return w_TopoAdapter_build_mesh;
    if (strcmp(signature, "static TopoAdapter.build_edge_geo(_)") == 0) return w_TopoAdapter_build_edge_geo;
    if (strcmp(signature, "static TopoAdapter.build_wire_geo(_)") == 0) return w_TopoAdapter_build_wire_geo;
    if (strcmp(signature, "static TopoAdapter.shape2wire(_)") == 0) return w_TopoAdapter_shape2wire;

    return nullptr;
}

void PartGraphBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
    if (strcmp(class_name, "CylindricalSurface") == 0)
    {
        methods->allocate = w_CylindricalSurface_allocate;
        methods->finalize = w_CylindricalSurface_finalize;
        return;
    }

    if (strcmp(class_name, "EllipseCurve") == 0)
    {
        methods->allocate = w_EllipseCurve_allocate;
        methods->finalize = w_EllipseCurve_finalize;
        return;
    }

    if (strcmp(class_name, "TrimmedCurve") == 0)
    {
        methods->allocate = w_TrimmedCurve_allocate;
        methods->finalize = w_TrimmedCurve_finalize;
        return;
    }

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

    if (strcmp(class_name, "TopoFace") == 0)
    {
        methods->allocate = w_TopoFace_allocate;
        methods->finalize = w_TopoFace_finalize;
        return;
    }

    if (strcmp(class_name, "WireBuilder") == 0)
    {
        methods->allocate = w_WireBuilder_allocate;
        methods->finalize = w_WireBuilder_finalize;
        return;
    }
}

}