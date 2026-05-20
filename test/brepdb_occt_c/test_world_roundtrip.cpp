#include <catch2/catch_test_macros.hpp>

#include "brepdb_c/WorldSender.h"
#include "brepdb_c/WorldReceiver.h"
#include "brepdb_c/TypedPool.h"
#include "brepgraph_c/TopoNaming.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <BRep_Tool.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>

using namespace brepdb;

namespace
{

int count_shapes(const TopoDS_Shape& shape, TopAbs_ShapeEnum type)
{
    TopTools_IndexedMapOfShape map;
    TopExp::MapShapes(shape, type, map);
    return map.Extent();
}

// Helper: serialize a shape through WorldSender and reconstruct via WorldReceiver::GetAll()
TopoDS_Shape roundtrip(const TopoDS_Shape& shape, BRepWorld& world)
{
    auto tn = std::make_shared<brepgraph::TopoNaming>();
    WorldSender sender(tn);
    sender.Serialize(shape, world);

    WorldReceiver receiver(world);
    return receiver.GetAll();
}

} // anonymous

TEST_CASE("Box roundtrip preserves topology", "[world_roundtrip]")
{
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();

    BRepWorld world;
    TopoDS_Shape result = roundtrip(box, world);

    REQUIRE_FALSE(result.IsNull());
    CHECK(count_shapes(result, TopAbs_SOLID)  == 1);
    CHECK(count_shapes(result, TopAbs_FACE)   == 6);
    CHECK(count_shapes(result, TopAbs_EDGE)   == 12);
    CHECK(count_shapes(result, TopAbs_VERTEX) == 8);
}

TEST_CASE("Cylinder roundtrip preserves topology", "[world_roundtrip]")
{
    TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(5.0, 20.0).Shape();

    BRepWorld world;
    TopoDS_Shape result = roundtrip(cyl, world);

    REQUIRE_FALSE(result.IsNull());
    CHECK(count_shapes(result, TopAbs_SOLID) == 1);
    CHECK(count_shapes(result, TopAbs_FACE)  == 3);
    CHECK(count_shapes(result, TopAbs_EDGE)  > 0);
}

TEST_CASE("Sphere roundtrip", "[world_roundtrip]")
{
    TopoDS_Shape sphere = BRepPrimAPI_MakeSphere(10.0).Shape();

    BRepWorld world;
    TopoDS_Shape result = roundtrip(sphere, world);

    REQUIRE_FALSE(result.IsNull());
    CHECK(count_shapes(result, TopAbs_SOLID) == 1);
    CHECK(count_shapes(result, TopAbs_FACE)  == 1);
}

TEST_CASE("Cone roundtrip", "[world_roundtrip]")
{
    TopoDS_Shape cone = BRepPrimAPI_MakeCone(5.0, 2.0, 15.0).Shape();

    BRepWorld world;
    TopoDS_Shape result = roundtrip(cone, world);

    REQUIRE_FALSE(result.IsNull());
    CHECK(count_shapes(result, TopAbs_SOLID) == 1);
    CHECK(count_shapes(result, TopAbs_FACE)  >= 3);
}

TEST_CASE("Torus roundtrip", "[world_roundtrip]")
{
    TopoDS_Shape torus = BRepPrimAPI_MakeTorus(10.0, 3.0).Shape();

    BRepWorld world;
    TopoDS_Shape result = roundtrip(torus, world);

    REQUIRE_FALSE(result.IsNull());
    CHECK(count_shapes(result, TopAbs_SOLID) == 1);
    CHECK(count_shapes(result, TopAbs_FACE)  == 1);
}

TEST_CASE("WorldReceiver vertex positions match", "[world_roundtrip]")
{
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();

    BRepWorld world;
    roundtrip(box, world);

    int vertex_count = 0;
    for (uint32_t id : world.AliveEntities()) {
        const Type* t = world.Types().Get(id);
        if (!t || *t != Type::Vertex) continue;

        const PositionComp* pos = world.Positions().Get(id);
        REQUIRE(pos != nullptr);

        CHECK(pos->x >= 0.0);
        CHECK(pos->x <= 10.0);
        CHECK(pos->y >= 0.0);
        CHECK(pos->y <= 20.0);
        CHECK(pos->z >= 0.0);
        CHECK(pos->z <= 30.0);

        ++vertex_count;
    }

    CHECK(vertex_count == 8);
}

TEST_CASE("WorldReceiver edge curves preserved", "[world_roundtrip]")
{
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();

    BRepWorld world;
    roundtrip(box, world);

    int edge_count = 0;
    for (uint32_t id : world.AliveEntities()) {
        const Type* t = world.Types().Get(id);
        if (!t || *t != Type::Edge) continue;

        const CurveComp* curve = world.Curves().Get(id);
        REQUIRE(curve != nullptr);
        CHECK(curve->curve_type == Type::Line);

        ++edge_count;
    }

    CHECK(edge_count == 12);
}

TEST_CASE("Multiple solids roundtrip", "[world_roundtrip]")
{
    TopoDS_Shape box1 = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    TopoDS_Shape box2 = BRepPrimAPI_MakeBox(5.0, 5.0, 5.0).Shape();

    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    builder.Add(compound, box1);
    builder.Add(compound, box2);

    BRepWorld world;
    TopoDS_Shape result = roundtrip(compound, world);

    REQUIRE_FALSE(result.IsNull());
    CHECK(count_shapes(result, TopAbs_SOLID) == 2);
}
