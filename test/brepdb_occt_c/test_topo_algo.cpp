#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "brepkit_c/TopoAlgo.h"
#include "brepkit_c/TopoShape.h"
#include "brepkit_c/ShapeHistory.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>

using namespace brepkit;

namespace
{

double volume_of(const TopoDS_Shape& s)
{
    GProp_GProps props;
    BRepGProp::VolumeProperties(s, props);
    return props.Mass();
}

int count_shapes(const TopoDS_Shape& s, TopAbs_ShapeEnum type)
{
    TopTools_IndexedMapOfShape map;
    TopExp::MapShapes(s, type, map);
    return map.Extent();
}

std::shared_ptr<TopoShape> make_box(double dx, double dy, double dz)
{
    return std::make_shared<TopoShape>(BRepPrimAPI_MakeBox(dx, dy, dz).Shape());
}

} // anonymous

TEST_CASE("Cut removes volume", "[topo_algo]")
{
    auto box1 = make_box(20, 20, 20);
    auto box2 = make_box(10, 10, 10);
    auto result = TopoAlgo::Cut(box1, box2, 0);
    REQUIRE(result != nullptr);
    CHECK(volume_of(result->GetShape()) == Catch::Approx(7000.0).margin(1.0));
}

TEST_CASE("Fuse adds volume", "[topo_algo]")
{
    auto box1 = make_box(10, 10, 10);
    auto box2_raw = make_box(10, 10, 10);
    auto box2 = TopoAlgo::Translate(box2_raw, 5, 0, 0, 0);
    REQUIRE(box2 != nullptr);
    auto result = TopoAlgo::Fuse(box1, box2, 0);
    REQUIRE(result != nullptr);
    CHECK(volume_of(result->GetShape()) == Catch::Approx(1500.0).margin(1.0));
}

TEST_CASE("Common gives intersection", "[topo_algo]")
{
    auto box1 = make_box(10, 10, 10);
    auto box2_raw = make_box(10, 10, 10);
    auto box2 = TopoAlgo::Translate(box2_raw, 5, 0, 0, 0);
    REQUIRE(box2 != nullptr);
    auto result = TopoAlgo::Common(box1, box2, 0);
    REQUIRE(result != nullptr);
    CHECK(volume_of(result->GetShape()) == Catch::Approx(500.0).margin(1.0));
}

TEST_CASE("Translate moves shape", "[topo_algo]")
{
    auto box = make_box(10, 10, 10);
    auto result = TopoAlgo::Translate(box, 100, 0, 0, 0);
    REQUIRE(result != nullptr);

    Bnd_Box bbox;
    BRepBndLib::Add(result->GetShape(), bbox);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    CHECK(xmin == Catch::Approx(100.0).margin(0.01));
    CHECK(xmax == Catch::Approx(110.0).margin(0.01));
}

TEST_CASE("Scale changes volume", "[topo_algo]")
{
    auto box = make_box(10, 10, 10);
    auto result = TopoAlgo::Scale(box, sm::vec3(0, 0, 0), 2.0, 0);
    REQUIRE(result != nullptr);
    CHECK(volume_of(result->GetShape()) == Catch::Approx(8000.0).margin(1.0));
}

TEST_CASE("Fillet rounds edges", "[topo_algo]")
{
    auto box = make_box(10, 10, 10);

    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(box->GetShape(), TopAbs_EDGE, edges);
    REQUIRE(edges.Extent() >= 1);

    std::vector<std::shared_ptr<TopoShape>> edge_vec;
    edge_vec.push_back(std::make_shared<TopoShape>(edges(1)));

    auto result = TopoAlgo::Fillet(box, 1.0, edge_vec, 0);
    REQUIRE(result != nullptr);
    CHECK(count_shapes(result->GetShape(), TopAbs_SOLID) == 1);
    CHECK(count_shapes(result->GetShape(), TopAbs_FACE) > 6);
}

TEST_CASE("Mirror reflects shape", "[topo_algo]")
{
    // SetMirror(gp_Ax1) mirrors about an axis (180° rotation).
    // Mirroring Box(0..10, 0..10, 0..10) about the X axis flips Y and Z.
    auto box = make_box(10, 10, 10);
    auto result = TopoAlgo::Mirror(box, sm::vec3(0, 0, 0), sm::vec3(1, 0, 0), 0);
    REQUIRE(result != nullptr);

    Bnd_Box bbox;
    BRepBndLib::Add(result->GetShape(), bbox);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    CHECK(xmin == Catch::Approx(0.0).margin(0.01));
    CHECK(xmax == Catch::Approx(10.0).margin(0.01));
    CHECK(ymin == Catch::Approx(-10.0).margin(0.01));
    CHECK(ymax == Catch::Approx(0.0).margin(0.01));
}

TEST_CASE("ShapeHistory tracks Boolean modifications", "[brep_history]")
{
    auto box1_shape = BRepPrimAPI_MakeBox(20, 20, 20).Shape();
    auto box2_shape = BRepPrimAPI_MakeBox(10, 10, 10).Shape();
    BRepAlgoAPI_Cut cutter(box1_shape, box2_shape);

    TopoShape old_ts(box1_shape);
    TopoShape new_ts(cutter.Shape());
    ShapeHistory hist(cutter, TopAbs_FACE, new_ts, old_ts);

    auto& idx_map = hist.GetIdxMap();
    CHECK(idx_map.size() > 0);
    CHECK(hist.GetNewMap().Extent() > 6);
}
