#include <catch2/catch_test_macros.hpp>

#include "breptopo_c/TopoGraphBuilder.h"
#include "brepkit_c/TopoShape.h"

#include <graph/Graph.h>
#include <graph/Node.h>

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

using namespace breptopo;

static std::shared_ptr<brepkit::TopoShape> make_box(double dx, double dy, double dz)
{
    BRepPrimAPI_MakeBox maker(dx, dy, dz);
    return std::make_shared<brepkit::TopoShape>(maker.Shape());
}

static std::shared_ptr<brepkit::TopoShape> make_cylinder(double r, double h)
{
    BRepPrimAPI_MakeCylinder maker(r, h);
    return std::make_shared<brepkit::TopoShape>(maker.Shape());
}

TEST_CASE("Box face adjacency graph has 6 nodes")
{
    auto box = make_box(10, 20, 30);

    std::vector<std::shared_ptr<brepkit::TopoShape>> shapes;
    shapes.push_back(box);

    auto graph = TopoGraphBuilder::BuildGraph(shapes);
    REQUIRE(graph != nullptr);
    CHECK(graph->GetNodesNum() == 6);
}

TEST_CASE("Box adjacency graph has 12 edges")
{
    // A box has 6 faces, each adjacent to 4 others.
    // Undirected edge count = (6 * 4) / 2 = 12.
    auto box = make_box(10, 20, 30);

    std::vector<std::shared_ptr<brepkit::TopoShape>> shapes;
    shapes.push_back(box);

    auto graph = TopoGraphBuilder::BuildGraph(shapes);
    REQUIRE(graph != nullptr);
    CHECK(graph->GetEdges().size() == 12);
}

TEST_CASE("Cylinder face adjacency graph has 3 nodes")
{
    // A cylinder has 3 faces: top disk, bottom disk, lateral surface.
    auto cyl = make_cylinder(5, 20);

    std::vector<std::shared_ptr<brepkit::TopoShape>> shapes;
    shapes.push_back(cyl);

    auto graph = TopoGraphBuilder::BuildGraph(shapes);
    REQUIRE(graph != nullptr);
    CHECK(graph->GetNodesNum() == 3);
}

TEST_CASE("Multiple shapes produce combined graph")
{
    auto box1 = make_box(10, 20, 30);
    auto box2 = make_box(5, 5, 5);

    std::vector<std::shared_ptr<brepkit::TopoShape>> shapes;
    shapes.push_back(box1);
    shapes.push_back(box2);

    auto graph = TopoGraphBuilder::BuildGraph(shapes);
    REQUIRE(graph != nullptr);
    // Two boxes: 6 + 6 = 12 face nodes.
    CHECK(graph->GetNodesNum() == 12);
}
