// ============================================================
// test_topoalgo_ext.cpp
//
// Unit tests for the extended TopoAlgo operations.
// Verifies correctness via volume / bounding-box checks.
//
// ============================================================

#include "brepkit_c/TopoAlgo_Ext.h"
#include "brepkit_c/TopoShape.h"
#include "brepkit_c/PrimMaker.h"
#include "brepkit_c/ShapeBuilder.h"

#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>

#include <cstdio>
#include <cmath>
#include <memory>

using namespace brepkit;

static int g_pass = 0;
static int g_fail = 0;

static void Check(bool cond, const char* msg)
{
    if (cond) {
        ++g_pass;
    } else {
        ++g_fail;
        printf("  FAIL: %s\n", msg);
    }
}

static bool Approx(double a, double b, double tol = 1e-3) {
    return std::fabs(a - b) < tol;
}

// Helper: compute volume of a shape
static double Volume(const std::shared_ptr<TopoShape>& shape)
{
    if (!shape) {
        return 0.0;
    }
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape->GetShape(), props);
    return props.Mass();
}

// Helper: build a unit square face on Z=0 plane
static std::shared_ptr<TopoShape> MakeUnitSquareFace()
{
    BRepBuilderAPI_MakePolygon poly;
    poly.Add(gp_Pnt(0, 0, 0));
    poly.Add(gp_Pnt(1, 0, 0));
    poly.Add(gp_Pnt(1, 1, 0));
    poly.Add(gp_Pnt(0, 1, 0));
    poly.Close();
    BRepBuilderAPI_MakeFace face(poly.Wire());
    return std::make_shared<TopoShape>(face.Face());
}

// Helper: build a small rectangle profile for sweep
static std::shared_ptr<TopoShape> MakeRectFace(double w, double h)
{
    BRepBuilderAPI_MakePolygon poly;
    poly.Add(gp_Pnt(0, 0, 0));
    poly.Add(gp_Pnt(w, 0, 0));
    poly.Add(gp_Pnt(w, h, 0));
    poly.Add(gp_Pnt(0, h, 0));
    poly.Close();
    BRepBuilderAPI_MakeFace face(poly.Wire());
    return std::make_shared<TopoShape>(face.Face());
}


// ============================================================
// Test: Blind extrude — basic distance check
// ============================================================

static void Test_Extrude_Blind()
{
    printf("[Extrude] Blind...\n");

    auto profile = MakeUnitSquareFace();
    auto result = TopoAlgo_Ext::ExtrudeEx(
        profile,
        0, 0, 1,        // direction +Z
        2.0, 0.0,       // dist1=2, dist2=0
        ExtrudeEndType::Blind,
        ExtrudeEndType::Blind,
        nullptr,
        1, nullptr);

    Check(result != nullptr, "blind extrude returns shape");
    Check(Approx(Volume(result), 2.0), "blind volume = 1*1*2 = 2");
}


// ============================================================
// Test: MidPlane — symmetric extrude
// ============================================================

static void Test_Extrude_MidPlane()
{
    printf("[Extrude] MidPlane...\n");

    auto profile = MakeUnitSquareFace();
    auto result = TopoAlgo_Ext::ExtrudeEx(
        profile,
        0, 0, 1,
        2.0, 0.0,
        ExtrudeEndType::MidPlane,
        ExtrudeEndType::Blind,
        nullptr,
        1, nullptr);

    Check(result != nullptr, "mid-plane extrude returns shape");
    Check(Approx(Volume(result), 2.0), "mid-plane volume = 2");

    // Bounding box should span Z = -1 to +1 (symmetric around Z=0)
    Bnd_Box box;
    BRepBndLib::Add(result->GetShape(), box);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    Check(Approx(zmin, -1.0, 1e-2), "mid-plane zmin = -1");
    Check(Approx(zmax,  1.0, 1e-2), "mid-plane zmax = +1");
}


// ============================================================
// Test: ThroughAll — should pass through reference
// ============================================================

static void Test_Extrude_ThroughAll()
{
    printf("[Extrude] ThroughAll...\n");

    auto profile = MakeUnitSquareFace();
    auto box_ref = PrimMaker::Box(10, 10, 10, 0);

    auto result = TopoAlgo_Ext::ExtrudeEx(
        profile,
        0, 0, 1,
        0.0, 0.0,
        ExtrudeEndType::ThroughAll,
        ExtrudeEndType::Blind,
        box_ref,
        1, nullptr);

    Check(result != nullptr, "through-all extrude returns shape");
    // Volume should be at least the diagonal of the bbox
    double v = Volume(result);
    Check(v > 10.0, "through-all volume > 10");
}


// ============================================================
// Test: Revolve — full revolution = solid of revolution
// ============================================================

static void Test_Revolve_Full()
{
    printf("[Revolve] Full 2*PI...\n");

    // Profile: square at x=2, in YZ plane (revolve around Z axis)
    BRepBuilderAPI_MakePolygon poly;
    poly.Add(gp_Pnt(2, 0, 0));
    poly.Add(gp_Pnt(3, 0, 0));
    poly.Add(gp_Pnt(3, 0, 1));
    poly.Add(gp_Pnt(2, 0, 1));
    poly.Close();
    BRepBuilderAPI_MakeFace face(poly.Wire());
    auto profile = std::make_shared<TopoShape>(face.Face());

    auto result = TopoAlgo_Ext::Revolve(
        profile,
        sm::vec3(0, 0, 0),
        sm::vec3(0, 0, 1),
        0.0,
        true,           // is_full
        1, nullptr);

    Check(result != nullptr, "revolve returns shape");

    // Expected: hollow cylinder (annulus * height)
    // Volume = PI * (3^2 - 2^2) * 1 = 5*PI ≈ 15.708
    double v = Volume(result);
    Check(Approx(v, 5.0 * M_PI, 0.5), "revolve volume ~ 5*PI");
}


// ============================================================
// Test: Revolve — partial angle
// ============================================================

static void Test_Revolve_Partial()
{
    printf("[Revolve] Half (PI)...\n");

    BRepBuilderAPI_MakePolygon poly;
    poly.Add(gp_Pnt(2, 0, 0));
    poly.Add(gp_Pnt(3, 0, 0));
    poly.Add(gp_Pnt(3, 0, 1));
    poly.Add(gp_Pnt(2, 0, 1));
    poly.Close();
    BRepBuilderAPI_MakeFace face(poly.Wire());
    auto profile = std::make_shared<TopoShape>(face.Face());

    auto result = TopoAlgo_Ext::Revolve(
        profile,
        sm::vec3(0, 0, 0),
        sm::vec3(0, 0, 1),
        M_PI,
        false,
        1, nullptr);

    Check(result != nullptr, "half revolve returns shape");
    // Half of full = 2.5*PI ≈ 7.854
    Check(Approx(Volume(result), 2.5 * M_PI, 0.5), "half volume ~ 2.5*PI");
}


// ============================================================
// Test: Sweep — small profile along straight path
// ============================================================

static void Test_Sweep_Straight()
{
    printf("[Sweep] Straight path...\n");

    auto profile = MakeRectFace(0.5, 0.5);

    // Path: a single straight edge from origin to (5,0,0)
    BRepBuilderAPI_MakeEdge edge_maker(gp_Pnt(0.25, 0.25, 0), gp_Pnt(5, 0.25, 0));
    BRepBuilderAPI_MakeWire wire_maker(edge_maker.Edge());
    auto path = std::make_shared<TopoShape>(wire_maker.Wire());

    auto result = TopoAlgo_Ext::Sweep(profile, path, true, 1, nullptr);

    Check(result != nullptr, "sweep returns shape");
    // Approximate volume = 0.5 * 0.5 * 5 = 1.25 (with some tolerance for sweep ends)
    if (result) {
        double v = Volume(result);
        Check(v > 1.0 && v < 1.5, "sweep volume in range [1.0, 1.5]");
    }
}


// ============================================================
// Test: LinearPattern 1D
// ============================================================

static void Test_LinearPattern_1D()
{
    printf("[LinearPattern] 1D, 3 instances...\n");

    auto box = PrimMaker::Box(1, 1, 1, 0);
    double v_base = Volume(box);

    auto result = TopoAlgo_Ext::LinearPattern(
        box,
        sm::vec3(2, 0, 0), 3, 2.0,   // dir1, count1=3, spacing=2
        sm::vec3(0, 0, 0), 1, 0.0,   // dir2 disabled
        1, nullptr);

    Check(result != nullptr, "1D pattern returns shape");
    if (result) {
        double v = Volume(result);
        // 3 separate boxes (no overlap with spacing=2 > size=1)
        Check(Approx(v, 3.0 * v_base, 0.01), "1D pattern volume = 3 * base");
    }
}


// ============================================================
// Test: LinearPattern 2D
// ============================================================

static void Test_LinearPattern_2D()
{
    printf("[LinearPattern] 2D grid 2x2...\n");

    auto box = PrimMaker::Box(1, 1, 1, 0);

    auto result = TopoAlgo_Ext::LinearPattern(
        box,
        sm::vec3(1, 0, 0), 2, 2.0,
        sm::vec3(0, 1, 0), 2, 2.0,
        1, nullptr);

    Check(result != nullptr, "2D pattern returns shape");
    if (result) {
        Check(Approx(Volume(result), 4.0, 0.01), "2D 2x2 pattern volume = 4");
    }
}


// ============================================================
// Test: CircularPattern
// ============================================================

static void Test_CircularPattern()
{
    printf("[CircularPattern] 4 instances around Z...\n");

    // A box offset from the axis so instances don't overlap
    auto box = PrimMaker::Box(0.5, 0.5, 1, 0);

    // First, translate it to be away from axis
    // (omitted here — assume PrimMaker::Box places at origin)

    auto result = TopoAlgo_Ext::CircularPattern(
        box,
        sm::vec3(2, 0, 0),     // offset axis from box
        sm::vec3(0, 0, 1),
        4,                       // 4 copies
        2.0 * M_PI,
        1, nullptr);

    Check(result != nullptr, "circular pattern returns shape");
    if (result) {
        // 4 copies, no overlap → volume should be 4 * base
        double v = Volume(result);
        Check(v > 0.5 && v < 2.5, "circular pattern volume in range");
    }
}


// ============================================================
// Main
// ============================================================

int main()
{
    printf("======================================\n");
    printf(" TopoAlgo_Ext Tests\n");
    printf("======================================\n\n");

    Test_Extrude_Blind();
    Test_Extrude_MidPlane();
    Test_Extrude_ThroughAll();

    printf("\n");

    Test_Revolve_Full();
    Test_Revolve_Partial();

    printf("\n");

    Test_Sweep_Straight();

    printf("\n");

    Test_LinearPattern_1D();
    Test_LinearPattern_2D();

    printf("\n");

    Test_CircularPattern();

    printf("\n======================================\n");
    printf(" Results: %d passed, %d failed\n", g_pass, g_fail);
    printf("======================================\n");

    return g_fail > 0 ? 1 : 0;
}
