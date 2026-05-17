#include <catch2/catch_test_macros.hpp>

#include "partgraph_c/BRepTools.h"
#include "partgraph_c/BRepBuilder.h"
#include "partgraph_c/TopoShape.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <gp_Pnt.hxx>

using namespace partgraph;

static std::shared_ptr<TopoShape> make_box(double dx, double dy, double dz) {
    BRepPrimAPI_MakeBox maker(dx, dy, dz);
    return std::make_shared<TopoShape>(maker.Shape());
}

TEST_CASE("MapFaces returns 6 faces for a box") {
    auto shape = make_box(10, 20, 30);
    auto faces = BRepTools::MapFaces(shape);
    REQUIRE(faces.size() == 6);
}

TEST_CASE("MapEdges returns 12 edges for a box") {
    auto shape = make_box(10, 20, 30);
    auto edges = BRepTools::MapEdges(shape);
    REQUIRE(edges.size() == 12);
}

TEST_CASE("MapShells returns 1 shell for a box") {
    auto shape = make_box(10, 20, 30);
    auto shells = BRepTools::MapShells(shape);
    REQUIRE(shells.size() == 1);
}

TEST_CASE("FindFaceKey and FindFaceIdx roundtrip") {
    auto shape = make_box(10, 20, 30);

    auto face_key = BRepTools::FindFaceKey(shape, 1);
    REQUIRE(face_key != nullptr);

    int idx = BRepTools::FindFaceIdx(shape, face_key);
    CHECK(idx == 1);

    CHECK(BRepTools::FindFaceKey(shape, 0) == nullptr);
    CHECK(BRepTools::FindFaceKey(shape, 100) == nullptr);
}

TEST_CASE("FindEdgeKey and FindEdgeIdx roundtrip") {
    auto shape = make_box(10, 20, 30);

    auto edge_key = BRepTools::FindEdgeKey(shape, 1);
    REQUIRE(edge_key != nullptr);

    int idx = BRepTools::FindEdgeIdx(shape, edge_key);
    CHECK(idx == 1);
}

TEST_CASE("BRepBuilder MakeCompound") {
    auto box1 = make_box(10, 20, 30);
    auto box2 = make_box(5, 5, 5);

    std::vector<std::shared_ptr<TopoShape>> shapes = { box1, box2 };
    auto compound = BRepBuilder::MakeCompound(shapes);
    REQUIRE(compound != nullptr);

    const auto& s = compound->GetShape();
    CHECK(s.ShapeType() == TopAbs_COMPOUND);

    TopTools_IndexedMapOfShape solids;
    TopExp::MapShapes(s, TopAbs_SOLID, solids);
    CHECK(solids.Extent() == 2);
}

TEST_CASE("FindFaceIdx returns 0 for unrelated face") {
    auto box1 = make_box(10, 20, 30);
    auto box2 = make_box(5, 5, 5);

    auto face_from_box2 = BRepTools::FindFaceKey(box2, 1);
    REQUIRE(face_from_box2 != nullptr);

    int idx = BRepTools::FindFaceIdx(box1, face_from_box2);
    CHECK(idx == 0);
}
