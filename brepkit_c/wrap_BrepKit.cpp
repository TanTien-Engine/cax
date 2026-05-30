#include "wrap_BrepKit.h"
#include "TopoShape.h"
#include "TopoAdapter.h"
#include "PrimMaker.h"
#include "ShapeBuilder.h"
#include "TopoAlgo.h"
#include "TopoAlgo_Ext.h"
#include "ShapeSelector.h"
#include "ShapeTools.h"
#include "GeomDataset.h"
#include "TransHelper.h"
#include "GlobalConfig.h"
#include <brepgraph_c/computation/CalcGraph.h>

#include <logger/logger.h>
#include <SM_Vector.h>

#include <cstring>
#include <geoshape/Line3D.h>
#include <wrapper/TransHelper.h>

// OCCT
#include <Precision.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>

namespace
{

void w_ShapeBuilder_make_edge_from_line()
{
    auto line = ((wrapper::Proxy<gs::Line3D>*)ves_toforeign(1))->obj;
    auto edge = brepkit::ShapeBuilder::MakeEdge(*line);
    brepkit::return_topo_shape(edge);
}

void w_ShapeBuilder_make_edge_from_arc()
{
    auto arc = ((wrapper::Proxy<gs::Arc3D>*)ves_toforeign(1))->obj;
    auto edge = brepkit::ShapeBuilder::MakeEdge(*arc);
    brepkit::return_topo_shape(edge);
}

void w_ShapeBuilder_make_edge_from_curve_surf()
{
    auto c = ((wrapper::Proxy<brepkit::TrimmedCurve>*)ves_toforeign(1))->obj;
    auto s = ((wrapper::Proxy<brepkit::CylindricalSurface>*)ves_toforeign(2))->obj;
    auto edge = brepkit::ShapeBuilder::MakeEdge(*c, *s);
    brepkit::return_topo_shape(edge);
}

void w_ShapeBuilder_make_wire()
{
    std::vector<std::shared_ptr<brepkit::TopoShape>> edges;
    wrapper::list_to_foreigns(1, edges);
    auto wire = brepkit::ShapeBuilder::MakeWire(edges);
    brepkit::return_topo_shape(wire);
}

void w_ShapeBuilder_make_face()
{
    auto wire = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto face = brepkit::ShapeBuilder::MakeFace(*wire);
    brepkit::return_topo_shape(face);
}

void w_ShapeBuilder_make_shell()
{
    std::vector<std::shared_ptr<brepkit::TopoShape>> faces;
    wrapper::list_to_foreigns(1, faces);
    auto shell = brepkit::ShapeBuilder::MakeShell(faces);
    brepkit::return_topo_shape(shell);
}

void w_ShapeBuilder_make_compound()
{
    std::vector<std::shared_ptr<brepkit::TopoShape>> shapes;
    wrapper::list_to_foreigns(1, shapes);
    auto shape = brepkit::ShapeBuilder::MakeCompound(shapes);
    brepkit::return_topo_shape(shape);
}

void w_PrimMaker_plane()
{
    double x = ves_tonumber(1);
    double y = ves_tonumber(2);
    double z = ves_tonumber(3);
    double nx = ves_tonumber(4);
    double ny = ves_tonumber(5);
    double nz = ves_tonumber(6);
    uint32_t op_id = (uint32_t)ves_tonumber(7);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto shape = brepkit::PrimMaker::Plane(x, y, z, nx, ny, nz, op_id, naming, vt);
    brepkit::return_topo_shape(shape);
}

void w_PrimMaker_box()
{
    double L = ves_tonumber(1);
    double W = ves_tonumber(2);
    double H = ves_tonumber(3);
    uint32_t op_id = (uint32_t)ves_tonumber(4);

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

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto shape = brepkit::PrimMaker::Box(L, W, H, op_id, naming, vt);

    brepkit::return_topo_shape(shape);
}

void w_PrimMaker_cylinder()
{
    double radius = ves_tonumber(1);
    double length = ves_tonumber(2);
    uint32_t op_id = (uint32_t)ves_tonumber(3);

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

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto shape = brepkit::PrimMaker::Cylinder(radius, length, op_id, naming, vt);
    brepkit::return_topo_shape(shape);
}

void w_PrimMaker_cone()
{
    double radius1 = ves_tonumber(1);
    double radius2 = ves_tonumber(2);
    double height = ves_tonumber(3);
    uint32_t op_id = (uint32_t)ves_tonumber(4);

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

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto shape = brepkit::PrimMaker::Cone(radius1, radius2, height, op_id, naming, vt);
    brepkit::return_topo_shape(shape);
}

void w_PrimMaker_sphere()
{
    double radius = ves_tonumber(1);
    uint32_t op_id = (uint32_t)ves_tonumber(2);

    bool skip = false;
    if (radius < Precision::Confusion()) {
        LOGI("Radius of sphere too small");
        skip = true;
    }
    if (skip) {
        ves_set_nil(0);
        return;
    }

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto shape = brepkit::PrimMaker::Sphere(radius, op_id, naming, vt);
    brepkit::return_topo_shape(shape);
}

void w_PrimMaker_sphere_with_angle()
{
    double radius = ves_tonumber(1);
    double angle = ves_tonumber(2);
    uint32_t op_id = (uint32_t)ves_tonumber(3);

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

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto shape = brepkit::PrimMaker::Sphere(radius, angle, op_id, naming, vt);
    brepkit::return_topo_shape(shape);
}

void w_PrimMaker_torus()
{
    double r1 = ves_tonumber(1);
    double r2 = ves_tonumber(2);
    uint32_t op_id = (uint32_t)ves_tonumber(3);

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

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto shape = brepkit::PrimMaker::Torus(r1, r2, op_id, naming, vt);
    brepkit::return_topo_shape(shape);
}

void w_PrimMaker_torus_with_angle()
{
    double r1 = ves_tonumber(1);
    double r2 = ves_tonumber(2);
    double angle = ves_tonumber(3);
    uint32_t op_id = (uint32_t)ves_tonumber(4);

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

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto shape = brepkit::PrimMaker::Torus(r1, r2, angle, op_id, naming, vt);
    brepkit::return_topo_shape(shape);
}

void w_PrimMaker_threading()
{
    double thickness = ves_tonumber(1);
    double height = ves_tonumber(2);
    uint32_t op_id = (uint32_t)ves_tonumber(3);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto shape = brepkit::PrimMaker::Threading(thickness, height, op_id, naming, vt);
    brepkit::return_topo_shape(shape);
}

void w_ShapeSelector_select_face()
{
    auto shape = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto pos = wrapper::list_to_vec3(2);
    auto dir = wrapper::list_to_vec3(3);

    auto face = brepkit::ShapeSelector::SelectFace(shape, sm::Ray(pos, dir));
    brepkit::return_topo_shape(face);
}

void w_ShapeSelector_select_edge()
{
    auto shape = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto pos = wrapper::list_to_vec3(2);
    auto dir = wrapper::list_to_vec3(3);

    auto edge = brepkit::ShapeSelector::SelectEdge(shape, sm::Ray(pos, dir));
    brepkit::return_topo_shape(edge);
}

void w_ShapeTools_find_edge_idx()
{
    auto shape = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto edge = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(2))->obj;
    int idx = brepkit::ShapeTools::FindEdgeIdx(shape, edge);
    ves_set_number(0, idx);
}

void w_ShapeTools_find_edge_key()
{
    auto shape = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    int idx = (int)ves_tonumber(2);
    auto edge = brepkit::ShapeTools::FindEdgeKey(shape, idx);
    brepkit::return_topo_shape(edge);
}

void w_ShapeTools_find_face_idx()
{
    auto shape = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto face = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(2))->obj;
    int idx = brepkit::ShapeTools::FindFaceIdx(shape, face);
    ves_set_number(0, idx);
}

void w_ShapeTools_find_face_key()
{
    auto shape = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    int idx = (int)ves_tonumber(2);
    auto face = brepkit::ShapeTools::FindFaceKey(shape, idx);
    brepkit::return_topo_shape(face);
}

void w_ShapeTools_map_shells()
{
    auto shape = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto shells = brepkit::ShapeTools::MapShells(shape);
    brepkit::return_topo_shape_list(shells);
}

void w_ShapeTools_map_faces()
{
    auto shape = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto faces = brepkit::ShapeTools::MapFaces(shape);
    brepkit::return_topo_shape_list(faces);
}

void w_ShapeTools_map_edges()
{
    auto shape = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto edges = brepkit::ShapeTools::MapEdges(shape);
    brepkit::return_topo_shape_list(edges);
}

void w_TopoAlgo_fillet()
{
    auto src = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    double thickness = ves_tonumber(2);
    std::vector<std::shared_ptr<brepkit::TopoShape>> edges;
    wrapper::list_to_foreigns(3, edges);
    uint32_t op_id = (uint32_t)ves_tonumber(4);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto dst = brepkit::TopoAlgo::Fillet(src, thickness, edges, op_id, naming, vt);
    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_chamfer()
{
    auto src = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    double dist = ves_tonumber(2);
    std::vector<std::shared_ptr<brepkit::TopoShape>> edges;
    wrapper::list_to_foreigns(3, edges);
    uint32_t op_id = (uint32_t)ves_tonumber(4);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto dst = brepkit::TopoAlgo::Chamfer(src, dist, edges, op_id, naming, vt);
    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_extrude()
{
    auto src = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    double x = ves_tonumber(2);
    double y = ves_tonumber(3);
    double z = ves_tonumber(4);
    uint32_t op_id = (uint32_t)ves_tonumber(5);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto dst = brepkit::TopoAlgo::Prism(src, x, y, z, op_id, naming, vt);
    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_split()
{
    auto base = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto tool = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(2))->obj;
    uint32_t op_id = (uint32_t)ves_tonumber(3);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto shape = brepkit::TopoAlgo::Split(base, tool, op_id, naming, vt);

    brepkit::return_topo_shape(shape);
}

void w_TopoAlgo_sew()
{
    auto s1 = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto s2 = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(2))->obj;
    uint32_t op_id = (uint32_t)ves_tonumber(3);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto shape = brepkit::TopoAlgo::Sew(s1, s2, op_id, naming, vt);

    brepkit::return_topo_shape(shape);
}

void w_TopoAlgo_unify_same_domain()
{
    auto src = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    uint32_t op_id = (uint32_t)ves_tonumber(2);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto dst = brepkit::TopoAlgo::UnifySameDomain(src, op_id, naming, vt);

    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_cut()
{
    auto s1 = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto s2 = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(2))->obj;
    uint32_t op_id = (uint32_t)ves_tonumber(3);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto shape = brepkit::TopoAlgo::Cut(s1, s2, op_id, naming, vt);

    brepkit::return_topo_shape(shape);
}

void w_TopoAlgo_fuse()
{
    auto s1 = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto s2 = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(2))->obj;
    uint32_t op_id = (uint32_t)ves_tonumber(3);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto shape = brepkit::TopoAlgo::Fuse(s1, s2, op_id, naming, vt);
    brepkit::return_topo_shape(shape);
}

void w_TopoAlgo_common()
{
    auto s1 = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto s2 = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(2))->obj;
    uint32_t op_id = (uint32_t)ves_tonumber(3);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto shape = brepkit::TopoAlgo::Common(s1, s2, op_id, naming, vt);
    brepkit::return_topo_shape(shape);
}

void w_TopoAlgo_section()
{
    auto s1 = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto s2 = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(2))->obj;
    uint32_t op_id = (uint32_t)ves_tonumber(3);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto shape = brepkit::TopoAlgo::Section(s1, s2, op_id, naming, vt);
    brepkit::return_topo_shape(shape);
}

void w_TopoAlgo_translate()
{
    auto src = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    double x = ves_tonumber(2);
    double y = ves_tonumber(3);
    double z = ves_tonumber(4);
    uint32_t op_id = (uint32_t)ves_tonumber(5);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto dst = brepkit::TopoAlgo::Translate(src, x, y, z, op_id, naming, vt);

    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_mirror()
{
    auto src = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto pos = wrapper::list_to_vec3(2);
    auto dir = wrapper::list_to_vec3(3);
    uint32_t op_id = (uint32_t)ves_tonumber(4);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto dst = brepkit::TopoAlgo::Mirror(src, pos, dir, op_id, naming, vt);
    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_rotate()
{
    auto src = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto pos = wrapper::list_to_vec3(2);
    auto dir = wrapper::list_to_vec3(3);
    double angle = ves_tonumber(4);
    uint32_t op_id = (uint32_t)ves_tonumber(5);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto dst = brepkit::TopoAlgo::Rotate(src, pos, dir, angle, op_id, naming, vt);
    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_scale()
{
    auto src = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto center = wrapper::list_to_vec3(2);
    double factor = ves_tonumber(3);
    uint32_t op_id = (uint32_t)ves_tonumber(4);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto dst = brepkit::TopoAlgo::Scale(src, center, factor, op_id, naming, vt);
    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_transform()
{
    auto src = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;

    double mat[12];
    for (int i = 0; i < 12; ++i) {
        mat[i] = ves_tonumber(2 + i);
    }
    uint32_t op_id = (uint32_t)ves_tonumber(14);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto dst = brepkit::TopoAlgo::Transform(src, mat, op_id, naming, vt);
    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_draft()
{
    auto src = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto dir = wrapper::list_to_vec3(2);
    auto angle = (float)ves_tonumber(3);
    auto len_max = (float)ves_tonumber(4);
    uint32_t op_id = (uint32_t)ves_tonumber(5);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto dst = brepkit::TopoAlgo::Draft(src, dir, angle, len_max, op_id, naming, vt);
    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_thick_solid()
{
    auto shape = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;

    std::vector<std::shared_ptr<brepkit::TopoShape>> faces;
    wrapper::list_to_foreigns(2, faces);

    auto offset = (float)ves_tonumber(3);
    uint32_t op_id = (uint32_t)ves_tonumber(4);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto dst = brepkit::TopoAlgo::ThickSolid(shape, faces, offset, op_id, naming, vt);
    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_thru_sections()
{
    std::vector<std::shared_ptr<brepkit::TopoShape>> wires;
    wrapper::list_to_foreigns(1, wires);
    uint32_t op_id = (uint32_t)ves_tonumber(2);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto shape = brepkit::TopoAlgo::ThruSections(wires, false, op_id, naming, vt);
    brepkit::return_topo_shape(shape);
}

void w_TopoAlgo_offset_shape()
{
    auto shape = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto offset = (float)ves_tonumber(2);
    auto is_solid = ves_toboolean(3);
    uint32_t op_id = (uint32_t)ves_tonumber(4);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto vt = brepkit::GlobalConfig::Instance()->GetVersionTree();
    auto dst = brepkit::TopoAlgo::OffsetShape(shape, offset, is_solid, op_id, naming, vt);

    brepkit::return_topo_shape(dst);
}

static brepkit::ExtrudeEndType ParseEndType(const char* s)
{
    if (!s) {
        return brepkit::ExtrudeEndType::Blind;
    }
    if (std::strcmp(s, "blind") == 0) {
        return brepkit::ExtrudeEndType::Blind;
    }
    if (std::strcmp(s, "through_all") == 0) {
        return brepkit::ExtrudeEndType::ThroughAll;
    }
    if (std::strcmp(s, "up_to_surface") == 0) {
        return brepkit::ExtrudeEndType::UpToSurface;
    }
    if (std::strcmp(s, "up_to_vertex") == 0) {
        return brepkit::ExtrudeEndType::UpToVertex;
    }
    if (std::strcmp(s, "offset_from_surface") == 0) {
        return brepkit::ExtrudeEndType::OffsetFromSurface;
    }
    if (std::strcmp(s, "mid_plane") == 0) {
        return brepkit::ExtrudeEndType::MidPlane;
    }
    if (std::strcmp(s, "up_to_first") == 0) {
        return brepkit::ExtrudeEndType::UpToFirst;
    }
    return brepkit::ExtrudeEndType::Blind;
}

void w_TopoAlgo_extrude_ex()
{
    auto src = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;

    double dx = ves_tonumber(2);
    double dy = ves_tonumber(3);
    double dz = ves_tonumber(4);

    double dist1 = ves_tonumber(5);
    double dist2 = ves_tonumber(6);

    const char* s_end1 = ves_tostring(7);
    const char* s_end2 = ves_tostring(8);
    auto end1 = ParseEndType(s_end1);
    auto end2 = ParseEndType(s_end2);

    std::shared_ptr<brepkit::TopoShape> ref = nullptr;
    if (ves_type(9) == VES_TYPE_FOREIGN) {
        ref = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(9))->obj;
    }

    uint32_t op_id = (uint32_t)ves_tonumber(10);
    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();

    auto dst = brepkit::TopoAlgo_Ext::ExtrudeEx(
        src, dx, dy, dz, dist1, dist2, end1, end2, ref, op_id, naming);
    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_revolve()
{
    auto src = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;

    sm::vec3 origin;
    origin.x = ves_tonumber(2);
    origin.y = ves_tonumber(3);
    origin.z = ves_tonumber(4);

    sm::vec3 dir;
    dir.x = ves_tonumber(5);
    dir.y = ves_tonumber(6);
    dir.z = ves_tonumber(7);

    double angle = ves_tonumber(8);
    bool is_full = ves_toboolean(9);
    uint32_t op_id = (uint32_t)ves_tonumber(10);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto dst = brepkit::TopoAlgo_Ext::Revolve(src, origin, dir, angle, is_full, op_id, naming);
    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_sweep()
{
    auto profile = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto path    = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(2))->obj;
    bool is_solid = ves_toboolean(3);
    uint32_t op_id = (uint32_t)ves_tonumber(4);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto dst = brepkit::TopoAlgo_Ext::Sweep(profile, path, is_solid, op_id, naming);
    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_linear_pattern()
{
    auto base = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;

    sm::vec3 dir1;
    dir1.x = ves_tonumber(2);
    dir1.y = ves_tonumber(3);
    dir1.z = ves_tonumber(4);
    int count1 = (int)ves_tonumber(5);
    double spacing1 = ves_tonumber(6);

    sm::vec3 dir2;
    dir2.x = ves_tonumber(7);
    dir2.y = ves_tonumber(8);
    dir2.z = ves_tonumber(9);
    int count2 = (int)ves_tonumber(10);
    double spacing2 = ves_tonumber(11);

    uint32_t op_id = (uint32_t)ves_tonumber(12);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto dst = brepkit::TopoAlgo_Ext::LinearPattern(
        base, dir1, count1, spacing1, dir2, count2, spacing2, op_id, naming);
    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_circular_pattern()
{
    auto base = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;

    sm::vec3 origin;
    origin.x = ves_tonumber(2);
    origin.y = ves_tonumber(3);
    origin.z = ves_tonumber(4);

    sm::vec3 dir;
    dir.x = ves_tonumber(5);
    dir.y = ves_tonumber(6);
    dir.z = ves_tonumber(7);

    int count = (int)ves_tonumber(8);
    double angle = ves_tonumber(9);
    uint32_t op_id = (uint32_t)ves_tonumber(10);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto dst = brepkit::TopoAlgo_Ext::CircularPattern(
        base, origin, dir, count, angle, op_id, naming);
    brepkit::return_topo_shape(dst);
}

static brepkit::TopoAlgo_Ext::HoleType ParseHoleType(const char* s)
{
    if (!s) {
        return brepkit::TopoAlgo_Ext::HoleType::Simple;
    }
    if (std::strcmp(s, "counterbore") == 0) {
        return brepkit::TopoAlgo_Ext::HoleType::Counterbore;
    }
    if (std::strcmp(s, "countersink") == 0) {
        return brepkit::TopoAlgo_Ext::HoleType::Countersink;
    }
    if (std::strcmp(s, "tapped") == 0) {
        return brepkit::TopoAlgo_Ext::HoleType::Tapped;
    }
    return brepkit::TopoAlgo_Ext::HoleType::Simple;
}

void w_TopoAlgo_hole_wizard()
{
    auto body = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;

    sm::vec3 pos;
    pos.x = ves_tonumber(2);
    pos.y = ves_tonumber(3);
    pos.z = ves_tonumber(4);

    sm::vec3 dir;
    dir.x = ves_tonumber(5);
    dir.y = ves_tonumber(6);
    dir.z = ves_tonumber(7);

    double diameter = ves_tonumber(8);
    double depth = ves_tonumber(9);

    const char* s_type = ves_tostring(10);
    auto hole_type = ParseHoleType(s_type);

    double cb_diameter = ves_tonumber(11);
    double cb_depth = ves_tonumber(12);
    double cs_diameter = ves_tonumber(13);
    double cs_angle = ves_tonumber(14);

    uint32_t op_id = (uint32_t)ves_tonumber(15);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto dst = brepkit::TopoAlgo_Ext::HoleWizard(
        body, pos, dir, diameter, depth, hole_type,
        cb_diameter, cb_depth, cs_diameter, cs_angle, op_id, naming);
    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_variable_fillet()
{
    auto src = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;

    std::vector<std::shared_ptr<brepkit::TopoShape>> edges;
    wrapper::list_to_foreigns(2, edges);

    // params is a flat list of numbers [u0, r0, u1, r1, ...]
    std::vector<double> params;
    int params_count = ves_len(3);
    params.reserve(params_count);
    for (int i = 0; i < params_count; ++i) {
        ves_geti(3, i);
        params.push_back(ves_tonumber(-1));
        ves_pop(1);
    }

    uint32_t op_id = (uint32_t)ves_tonumber(4);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto dst = brepkit::TopoAlgo_Ext::VariableFillet(
        src, edges, params, op_id, naming);
    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_sweep_with_guide()
{
    auto profile = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto path    = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(2))->obj;

    std::vector<std::shared_ptr<brepkit::TopoShape>> guides;
    wrapper::list_to_foreigns(3, guides);

    bool is_solid = ves_toboolean(4);
    uint32_t op_id = (uint32_t)ves_tonumber(5);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto dst = brepkit::TopoAlgo_Ext::SweepWithGuide(
        profile, path, guides, is_solid, op_id, naming);
    brepkit::return_topo_shape(dst);
}

void w_TopoAlgo_rib()
{
    auto body    = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto profile = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(2))->obj;

    sm::vec3 dir;
    dir.x = ves_tonumber(3);
    dir.y = ves_tonumber(4);
    dir.z = ves_tonumber(5);

    double thickness = ves_tonumber(6);
    bool is_symmetric = ves_toboolean(7);
    uint32_t op_id = (uint32_t)ves_tonumber(8);

    auto naming = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    auto dst = brepkit::TopoAlgo_Ext::Rib(
        body, profile, dir, thickness, is_symmetric, op_id, naming);
    brepkit::return_topo_shape(dst);
}

void w_TopoAdapter_build_mesh()
{
    auto dev = ((wrapper::Proxy<ur::Device>*)ves_toforeign(1))->obj;
    auto shape = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(2))->obj;

    auto va = brepkit::TopoAdapter::BuildMeshFromShape(dev, *shape);

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("render", "VertexArray");
    auto proxy = (wrapper::Proxy<ur::VertexArray>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<ur::VertexArray>));
    proxy->obj = va;
    ves_pop(1);
}

// 3-arg variant: build_mesh(dev, shape, alpha). alpha is baked into
// every vertex (per-part transparency). The 2-arg form above keeps
// alpha at the 1.0 default for all existing callers.
void w_TopoAdapter_build_mesh_alpha()
{
    auto dev = ((wrapper::Proxy<ur::Device>*)ves_toforeign(1))->obj;
    auto shape = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(2))->obj;
    float alpha = (float)ves_tonumber(3);

    auto va = brepkit::TopoAdapter::BuildMeshFromShape(dev, *shape, alpha);

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("render", "VertexArray");
    auto proxy = (wrapper::Proxy<ur::VertexArray>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<ur::VertexArray>));
    proxy->obj = va;
    ves_pop(1);
}

void w_TopoAdapter_build_edges()
{
    auto dev = ((wrapper::Proxy<ur::Device>*)ves_toforeign(1))->obj;
    auto shape = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(2))->obj;

    auto va = brepkit::TopoAdapter::BuildEdgesFromShape(dev, *shape);

    ves_pop(ves_argnum());

    if (!va) {
        ves_set_nil(0);
        return;
    }

    ves_pushnil();
    ves_import_class("render", "VertexArray");
    auto proxy = (wrapper::Proxy<ur::VertexArray>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<ur::VertexArray>));
    proxy->obj = va;
    ves_pop(1);
}

// 3-arg variant: build_edges(dev, shape, alpha). alpha is baked into
// every edge vertex so transparent parts draw their edges at the same
// opacity as their faces. The 2-arg form keeps alpha at 1.0.
void w_TopoAdapter_build_edges_alpha()
{
    auto dev = ((wrapper::Proxy<ur::Device>*)ves_toforeign(1))->obj;
    auto shape = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(2))->obj;
    float alpha = (float)ves_tonumber(3);

    auto va = brepkit::TopoAdapter::BuildEdgesFromShape(dev, *shape, alpha);

    ves_pop(ves_argnum());

    if (!va) {
        ves_set_nil(0);
        return;
    }

    ves_pushnil();
    ves_import_class("render", "VertexArray");
    auto proxy = (wrapper::Proxy<ur::VertexArray>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<ur::VertexArray>));
    proxy->obj = va;
    ves_pop(1);
}

void w_TopoAdapter_build_edge_geo()
{
    auto edge = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;

    auto geo = brepkit::TopoAdapter::BuildGeoFromEdge(*edge);

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("geometry", "Line3D");
    auto proxy = (wrapper::Proxy<gs::Line3D>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<gs::Line3D>));
    proxy->obj = geo;
    ves_pop(1);
}

void w_TopoAdapter_build_wire_geo()
{
    auto wire = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;

    auto geo = brepkit::TopoAdapter::BuildGeoFromWire(*wire);

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("geometry", "Polyline3D");
    auto proxy = (wrapper::Proxy<gs::Polyline3D>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<gs::Polyline3D>));
    proxy->obj = geo;
    ves_pop(1);
}

void w_TopoAdapter_shape2wire()
{
    auto shape = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    auto wire = brepkit::TopoAdapter::ToWire(*shape);
    brepkit::return_topo_shape(wire);
}

void w_CylindricalSurface_allocate()
{
    auto pos = wrapper::list_to_vec3(1);
    auto dir = wrapper::list_to_vec3(2);
    float radius = (float)ves_tonumber(3);

    auto proxy = (wrapper::Proxy<brepkit::CylindricalSurface>*)ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<brepkit::CylindricalSurface>));
    proxy->obj = std::make_shared<brepkit::CylindricalSurface>(pos, dir, radius);
}

int w_CylindricalSurface_finalize(void* data)
{
    auto proxy = (wrapper::Proxy<brepkit::CylindricalSurface>*)(data);
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<brepkit::CylindricalSurface>);
}

void w_EllipseCurve_allocate()
{
    auto pos = wrapper::list_to_vec2(1);
    auto dir = wrapper::list_to_vec2(2);
    float major_radius = (float)ves_tonumber(3);
    float minor_radius = (float)ves_tonumber(4);

    auto proxy = (wrapper::Proxy<brepkit::EllipseCurve>*)ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<brepkit::EllipseCurve>));
    proxy->obj = std::make_shared<brepkit::EllipseCurve>(pos, dir, major_radius, minor_radius);
}

int w_EllipseCurve_finalize(void* data)
{
    auto proxy = (wrapper::Proxy<brepkit::EllipseCurve>*)(data);
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<brepkit::EllipseCurve>);
}

void w_TrimmedCurve_allocate()
{
    auto ellipse = ((wrapper::Proxy<brepkit::EllipseCurve>*)ves_toforeign(1))->obj;
    float u1 = (float)ves_tonumber(2);
    float u2 = (float)ves_tonumber(3);

    auto proxy = (wrapper::Proxy<brepkit::TrimmedCurve>*)ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<brepkit::TrimmedCurve>));
    proxy->obj = std::make_shared<brepkit::TrimmedCurve>(*ellipse, u1, u2);
}

int w_TrimmedCurve_finalize(void* data)
{
    auto proxy = (wrapper::Proxy<brepkit::TrimmedCurve>*)(data);
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<brepkit::TrimmedCurve>);
}

void w_TopoShape_allocate()
{
    auto proxy = (wrapper::Proxy<brepkit::TopoShape>*)ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<brepkit::TopoShape>));
    proxy->obj = std::make_shared<brepkit::TopoShape>();
}

int w_TopoShape_finalize(void* data)
{
    auto proxy = (wrapper::Proxy<brepkit::TopoShape>*)(data);
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<brepkit::TopoShape>);
}

void w_WireBuilder_allocate()
{
    auto proxy = (wrapper::Proxy<BRepBuilderAPI_MakeWire>*)ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<BRepBuilderAPI_MakeWire>));
    proxy->obj = std::make_shared<BRepBuilderAPI_MakeWire>();
}

int w_WireBuilder_finalize(void* data)
{
    auto proxy = (wrapper::Proxy<BRepBuilderAPI_MakeWire>*)(data);
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<BRepBuilderAPI_MakeWire>);
}

void w_WireBuilder_add_edge()
{
    auto builder = ((wrapper::Proxy<BRepBuilderAPI_MakeWire>*)ves_toforeign(0))->obj;
    auto edge = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    builder->Add(edge->ToEdge());
}

void w_WireBuilder_add_wire()
{
    auto builder = ((wrapper::Proxy<BRepBuilderAPI_MakeWire>*)ves_toforeign(0))->obj;
    auto wire = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    builder->Add(wire->ToWire());
}

void w_WireBuilder_gen_wire()
{
    auto builder = ((wrapper::Proxy<BRepBuilderAPI_MakeWire>*)ves_toforeign(0))->obj;
    if (!builder->IsDone()) {
        ves_set_nil(0);
        return;
    }

    auto wire = std::make_shared<brepkit::TopoShape>(builder->Wire());
    brepkit::return_topo_shape(wire);
}

void w_GlobalConfig_get_topo_naming()
{
    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("brepgraph", "TopoNaming");
    auto proxy = (wrapper::Proxy<brepgraph::TopoNaming>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<brepgraph::TopoNaming>));
    proxy->obj = brepkit::GlobalConfig::Instance()->GetTopoNaming();
    ves_pop(1);
}

void w_GlobalConfig_get_version_tree()
{
    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("brepdb", "VersionTree");
    auto proxy = (wrapper::Proxy<brepdb::VersionTree>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<brepdb::VersionTree>));
    proxy->obj = brepkit::GlobalConfig::Instance()->GetVersionTree();
    ves_pop(1);
}

void w_GlobalConfig_get_calc_graph()
{
    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("brepgraph", "CalcGraph");
    auto proxy = (wrapper::Proxy<brepgraph::CalcGraph>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<brepgraph::CalcGraph>));
    proxy->obj = brepkit::GlobalConfig::Instance()->GetCalcGraph();
    ves_pop(1);
}

}

namespace brepkit
{

VesselForeignMethodFn BrepKitBindMethod(const char* signature)
{
    if (strcmp(signature, "WireBuilder.add_edge(_)") == 0) return w_WireBuilder_add_edge;
    if (strcmp(signature, "WireBuilder.add_wire(_)") == 0) return w_WireBuilder_add_wire;
    if (strcmp(signature, "WireBuilder.gen_wire()") == 0) return w_WireBuilder_gen_wire;

    if (strcmp(signature, "static ShapeBuilder.make_edge_from_line(_)") == 0) return w_ShapeBuilder_make_edge_from_line;
    if (strcmp(signature, "static ShapeBuilder.make_edge_from_arc(_)") == 0) return w_ShapeBuilder_make_edge_from_arc;
    if (strcmp(signature, "static ShapeBuilder.make_edge_from_curve_surf(_,_)") == 0) return w_ShapeBuilder_make_edge_from_curve_surf;
    if (strcmp(signature, "static ShapeBuilder.make_wire(_)") == 0) return w_ShapeBuilder_make_wire;
    if (strcmp(signature, "static ShapeBuilder.make_face(_)") == 0) return w_ShapeBuilder_make_face;
    if (strcmp(signature, "static ShapeBuilder.make_shell(_)") == 0) return w_ShapeBuilder_make_shell;
    if (strcmp(signature, "static ShapeBuilder.make_compound(_)") == 0) return w_ShapeBuilder_make_compound;

    if (strcmp(signature, "static PrimMaker.plane(_,_,_,_,_,_,_)") == 0) return w_PrimMaker_plane;
    if (strcmp(signature, "static PrimMaker.box(_,_,_,_)") == 0) return w_PrimMaker_box;
    if (strcmp(signature, "static PrimMaker.cylinder(_,_,_)") == 0) return w_PrimMaker_cylinder;
    if (strcmp(signature, "static PrimMaker.cone(_,_,_,_)") == 0) return w_PrimMaker_cone;
    if (strcmp(signature, "static PrimMaker.sphere(_,_)") == 0) return w_PrimMaker_sphere;
    if (strcmp(signature, "static PrimMaker.sphere_with_angle(_,_,_)") == 0) return w_PrimMaker_sphere_with_angle;
    if (strcmp(signature, "static PrimMaker.torus(_,_,_)") == 0) return w_PrimMaker_torus;
    if (strcmp(signature, "static PrimMaker.torus_with_angle(_,_,_,_)") == 0) return w_PrimMaker_torus_with_angle;
    if (strcmp(signature, "static PrimMaker.threading(_,_,_)") == 0) return w_PrimMaker_threading;

    if (strcmp(signature, "static ShapeSelector.select_face(_,_,_)") == 0) return w_ShapeSelector_select_face;
    if (strcmp(signature, "static ShapeSelector.select_edge(_,_,_)") == 0) return w_ShapeSelector_select_edge;

    if (strcmp(signature, "static ShapeTools.find_edge_idx(_,_)") == 0) return w_ShapeTools_find_edge_idx;
    if (strcmp(signature, "static ShapeTools.find_edge_key(_,_)") == 0) return w_ShapeTools_find_edge_key;
    if (strcmp(signature, "static ShapeTools.find_face_idx(_,_)") == 0) return w_ShapeTools_find_face_idx;
    if (strcmp(signature, "static ShapeTools.find_face_key(_,_)") == 0) return w_ShapeTools_find_face_key;
    if (strcmp(signature, "static ShapeTools.map_shells(_)") == 0) return w_ShapeTools_map_shells;
    if (strcmp(signature, "static ShapeTools.map_faces(_)") == 0) return w_ShapeTools_map_faces;
    if (strcmp(signature, "static ShapeTools.map_edges(_)") == 0) return w_ShapeTools_map_edges;

    if (strcmp(signature, "static TopoAlgo.fillet(_,_,_,_)") == 0) return w_TopoAlgo_fillet;
    if (strcmp(signature, "static TopoAlgo.chamfer(_,_,_,_)") == 0) return w_TopoAlgo_chamfer;
    if (strcmp(signature, "static TopoAlgo.extrude(_,_,_,_,_)") == 0) return w_TopoAlgo_extrude;
    if (strcmp(signature, "static TopoAlgo.split(_,_,_)") == 0) return w_TopoAlgo_split;
    if (strcmp(signature, "static TopoAlgo.cut(_,_,_)") == 0) return w_TopoAlgo_cut;
    if (strcmp(signature, "static TopoAlgo.fuse(_,_,_)") == 0) return w_TopoAlgo_fuse;
    if (strcmp(signature, "static TopoAlgo.common(_,_,_)") == 0) return w_TopoAlgo_common;
    if (strcmp(signature, "static TopoAlgo.section(_,_,_)") == 0) return w_TopoAlgo_section;
    if (strcmp(signature, "static TopoAlgo.sew(_,_,_)") == 0) return w_TopoAlgo_sew;
    if (strcmp(signature, "static TopoAlgo.unify_same_domain(_,_)") == 0) return w_TopoAlgo_unify_same_domain;
    if (strcmp(signature, "static TopoAlgo.translate(_,_,_,_,_)") == 0) return w_TopoAlgo_translate;
    if (strcmp(signature, "static TopoAlgo.rotate(_,_,_,_,_)") == 0) return w_TopoAlgo_rotate;
    if (strcmp(signature, "static TopoAlgo.scale(_,_,_,_)") == 0) return w_TopoAlgo_scale;
    if (strcmp(signature, "static TopoAlgo.transform(_,_,_,_,_,_,_,_,_,_,_,_,_,_)") == 0) return w_TopoAlgo_transform;
    if (strcmp(signature, "static TopoAlgo.mirror(_,_,_,_)") == 0) return w_TopoAlgo_mirror;
    if (strcmp(signature, "static TopoAlgo.draft(_,_,_,_,_)") == 0) return w_TopoAlgo_draft;
    if (strcmp(signature, "static TopoAlgo.thick_solid(_,_,_,_)") == 0) return w_TopoAlgo_thick_solid;
    if (strcmp(signature, "static TopoAlgo.thru_sections(_,_)") == 0) return w_TopoAlgo_thru_sections;
    if (strcmp(signature, "static TopoAlgo.offset_shape(_,_,_,_)") == 0) return w_TopoAlgo_offset_shape;

    if (strcmp(signature, "static TopoAlgo.extrude_ex(_,_,_,_,_,_,_,_,_,_)") == 0) return w_TopoAlgo_extrude_ex;
    if (strcmp(signature, "static TopoAlgo.revolve(_,_,_,_,_,_,_,_,_,_)") == 0) return w_TopoAlgo_revolve;
    if (strcmp(signature, "static TopoAlgo.sweep(_,_,_,_)") == 0) return w_TopoAlgo_sweep;
    if (strcmp(signature, "static TopoAlgo.linear_pattern(_,_,_,_,_,_,_,_,_,_,_,_)") == 0) return w_TopoAlgo_linear_pattern;
    if (strcmp(signature, "static TopoAlgo.circular_pattern(_,_,_,_,_,_,_,_,_,_)") == 0) return w_TopoAlgo_circular_pattern;
    if (strcmp(signature, "static TopoAlgo.hole_wizard(_,_,_,_,_,_,_,_,_,_,_,_,_,_,_)") == 0) return w_TopoAlgo_hole_wizard;
    if (strcmp(signature, "static TopoAlgo.variable_fillet(_,_,_,_)") == 0) return w_TopoAlgo_variable_fillet;
    if (strcmp(signature, "static TopoAlgo.sweep_with_guide(_,_,_,_,_)") == 0) return w_TopoAlgo_sweep_with_guide;
    if (strcmp(signature, "static TopoAlgo.rib(_,_,_,_,_,_,_,_)") == 0) return w_TopoAlgo_rib;

    if (strcmp(signature, "static TopoAdapter.build_mesh(_,_)") == 0) return w_TopoAdapter_build_mesh;
    if (strcmp(signature, "static TopoAdapter.build_mesh(_,_,_)") == 0) return w_TopoAdapter_build_mesh_alpha;
    if (strcmp(signature, "static TopoAdapter.build_edges(_,_)") == 0) return w_TopoAdapter_build_edges;
    if (strcmp(signature, "static TopoAdapter.build_edges(_,_,_)") == 0) return w_TopoAdapter_build_edges_alpha;
    if (strcmp(signature, "static TopoAdapter.build_edge_geo(_)") == 0) return w_TopoAdapter_build_edge_geo;
    if (strcmp(signature, "static TopoAdapter.build_wire_geo(_)") == 0) return w_TopoAdapter_build_wire_geo;
    if (strcmp(signature, "static TopoAdapter.shape2wire(_)") == 0) return w_TopoAdapter_shape2wire;

    if (strcmp(signature, "static GlobalConfig.get_topo_naming()") == 0) return w_GlobalConfig_get_topo_naming;
    if (strcmp(signature, "static GlobalConfig.get_version_tree()") == 0) return w_GlobalConfig_get_version_tree;
    if (strcmp(signature, "static GlobalConfig.get_calc_graph()") == 0) return w_GlobalConfig_get_calc_graph;

    return nullptr;
}

void BrepKitBindClass(const char* class_name, VesselForeignClassMethods* methods)
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

    if (strcmp(class_name, "WireBuilder") == 0)
    {
        methods->allocate = w_WireBuilder_allocate;
        methods->finalize = w_WireBuilder_finalize;
        return;
    }
}

}