#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "partgraph_c/BRepSelector.h"
#include "partgraph_c/TopoShape.h"
#include <BRepPrimAPI_MakeBox.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Plane.hxx>
#include <TopoDS.hxx>
#include <SM_Ray.h>

using namespace partgraph;

static std::shared_ptr<TopoShape> MakeBox()
{
    BRepPrimAPI_MakeBox mkBox(10.0, 20.0, 30.0);
    mkBox.Build();
    return std::make_shared<TopoShape>(mkBox.Shape());
}

TEST_CASE("SelectFace by position Z_MAX")
{
    auto shape = MakeBox();
    auto result = BRepSelector::SelectFace(shape, BRepSelector::FacePos::Z_MAX);
    REQUIRE(result != nullptr);

    TopoDS_Face face = TopoDS::Face(result->GetShape());
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    Handle(Geom_Plane) plane = Handle(Geom_Plane)::DownCast(surf);
    REQUIRE_FALSE(plane.IsNull());

    gp_Pnt loc = plane->Location();
    REQUIRE(loc.Z() == Catch::Approx(30.0));
}

TEST_CASE("SelectFace by position Z_MIN")
{
    auto shape = MakeBox();
    auto result = BRepSelector::SelectFace(shape, BRepSelector::FacePos::Z_MIN);
    REQUIRE(result != nullptr);

    TopoDS_Face face = TopoDS::Face(result->GetShape());
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    Handle(Geom_Plane) plane = Handle(Geom_Plane)::DownCast(surf);
    REQUIRE_FALSE(plane.IsNull());

    gp_Pnt loc = plane->Location();
    REQUIRE(loc.Z() == Catch::Approx(0.0));
}

TEST_CASE("SelectFace by position X_MAX")
{
    auto shape = MakeBox();
    auto result = BRepSelector::SelectFace(shape, BRepSelector::FacePos::X_MAX);
    REQUIRE(result != nullptr);

    TopoDS_Face face = TopoDS::Face(result->GetShape());
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    Handle(Geom_Plane) plane = Handle(Geom_Plane)::DownCast(surf);
    REQUIRE_FALSE(plane.IsNull());

    gp_Pnt loc = plane->Location();
    REQUIRE(loc.X() == Catch::Approx(10.0));
}

TEST_CASE("SelectFace by position Y_MAX")
{
    auto shape = MakeBox();
    auto result = BRepSelector::SelectFace(shape, BRepSelector::FacePos::Y_MAX);
    REQUIRE(result != nullptr);

    TopoDS_Face face = TopoDS::Face(result->GetShape());
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    Handle(Geom_Plane) plane = Handle(Geom_Plane)::DownCast(surf);
    REQUIRE_FALSE(plane.IsNull());

    gp_Pnt loc = plane->Location();
    REQUIRE(loc.Y() == Catch::Approx(20.0));
}

TEST_CASE("SelectFace by index")
{
    auto shape = MakeBox();

    // Valid index: 1 is within [1, Extent-1)
    auto result = BRepSelector::SelectFace(shape, 1);
    REQUIRE(result != nullptr);

    // Out of range: 0 is below minimum
    auto result0 = BRepSelector::SelectFace(shape, 0);
    REQUIRE(result0 == nullptr);

    // Out of range: 100 is way above extent
    auto result100 = BRepSelector::SelectFace(shape, 100);
    REQUIRE(result100 == nullptr);
}

TEST_CASE("SelectFace by ray")
{
    auto shape = MakeBox();

    // Ray from above pointing down, should hit Z_MAX face
    sm::Ray ray;
    ray.origin = sm::vec3(5, 10, 50);
    ray.dir = sm::vec3(0, 0, -1);

    auto result = BRepSelector::SelectFace(shape, ray);
    REQUIRE(result != nullptr);

    TopoDS_Face face = TopoDS::Face(result->GetShape());
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    Handle(Geom_Plane) plane = Handle(Geom_Plane)::DownCast(surf);
    REQUIRE_FALSE(plane.IsNull());

    gp_Pnt loc = plane->Location();
    REQUIRE(loc.Z() == Catch::Approx(30.0));
}
