#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "brepkit_c/PrimMaker.h"
#include "brepkit_c/TopoShape.h"

#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

using namespace brepkit;

static int count_shapes(const TopoDS_Shape& s, TopAbs_ShapeEnum type) {
    TopTools_IndexedMapOfShape map;
    TopExp::MapShapes(s, type, map);
    return map.Extent();
}

static double volume_of(const TopoDS_Shape& s) {
    GProp_GProps props;
    BRepGProp::VolumeProperties(s, props);
    return props.Mass();
}

TEST_CASE("PrimMaker Box") {
    auto shape = PrimMaker::Box(10, 20, 30);
    REQUIRE(shape != nullptr);

    const auto& s = shape->GetShape();
    CHECK(count_shapes(s, TopAbs_FACE) == 6);
    CHECK(count_shapes(s, TopAbs_EDGE) == 12);
    CHECK(count_shapes(s, TopAbs_VERTEX) == 8);
    CHECK(volume_of(s) == Catch::Approx(6000.0).epsilon(0.01));
}

TEST_CASE("PrimMaker Cylinder") {
    auto shape = PrimMaker::Cylinder(5, 20);
    REQUIRE(shape != nullptr);

    const auto& s = shape->GetShape();
    CHECK(count_shapes(s, TopAbs_FACE) == 3);
    CHECK(volume_of(s) == Catch::Approx(1570.8).margin(1.0));
}

TEST_CASE("PrimMaker Cone") {
    auto shape = PrimMaker::Cone(5, 2, 15);
    REQUIRE(shape != nullptr);

    const auto& s = shape->GetShape();
    CHECK(count_shapes(s, TopAbs_SOLID) >= 1);
    CHECK(volume_of(s) == Catch::Approx(613.2).margin(1.0));
}

TEST_CASE("PrimMaker Sphere") {
    auto shape = PrimMaker::Sphere(10);
    REQUIRE(shape != nullptr);

    const auto& s = shape->GetShape();
    CHECK(volume_of(s) == Catch::Approx(4188.8).margin(1.0));
}

TEST_CASE("PrimMaker Torus") {
    auto shape = PrimMaker::Torus(10, 3);
    REQUIRE(shape != nullptr);

    const auto& s = shape->GetShape();
    CHECK(volume_of(s) == Catch::Approx(1776.5).margin(1.0));
}

TEST_CASE("PrimMaker Plane") {
    auto shape = PrimMaker::Plane(0, 0, 0, 0, 0, 1);
    REQUIRE(shape != nullptr);

    const auto& s = shape->GetShape();
    CHECK(count_shapes(s, TopAbs_FACE) >= 1);
    CHECK(count_shapes(s, TopAbs_SOLID) == 0);
}
