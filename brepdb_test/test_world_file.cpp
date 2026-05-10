#include <catch2/catch_test_macros.hpp>

#include "TypedPool.h"
#include "WorldFile.h"

#include <cstdio>
#include <fstream>

using namespace brepdb;

// ============================================================
// Helpers
// ============================================================

static BRepWorld make_test_world()
{
    BRepWorld world;

    // Vertex
    uint32_t v1 = 1;
    world.RegisterEntity(v1);
    world.Types().Set(v1, Type::Vertex);
    world.Aabbs().Set(v1, {{0, 0, 0}, {0.1, 0.1, 0.1}});
    world.Positions().Set(v1, {0.05, 0.05, 0.05});
    world.Tolerances().Set(v1, {0.001});

    uint32_t v2 = 2;
    world.RegisterEntity(v2);
    world.Types().Set(v2, Type::Vertex);
    world.Aabbs().Set(v2, {{1, 1, 1}, {1.1, 1.1, 1.1}});
    world.Positions().Set(v2, {1.05, 1.05, 1.05});
    world.Tolerances().Set(v2, {0.001});

    // Edge with curve
    uint32_t e1 = 10;
    world.RegisterEntity(e1);
    world.Types().Set(e1, Type::Edge);
    world.Aabbs().Set(e1, {{0, 0, 0}, {1.1, 1.1, 1.1}});
    world.Tolerances().Set(e1, {0.01});

    EdgeTopoComp etopo;
    etopo.v_first = v1;
    etopo.v_last  = v2;
    etopo.t_first = 0.0;
    etopo.t_last  = 1.0;
    world.EdgeTopos().Set(e1, etopo);

    CurveComp curve;
    curve.curve_type = Type::Line;
    curve.data = {0, 0, 0, 1, 1, 1};  // location + direction
    world.Curves().Set(e1, curve);

    // Face with surface and topo
    uint32_t f1 = 100;
    world.RegisterEntity(f1);
    world.Types().Set(f1, Type::Face);
    world.Aabbs().Set(f1, {{0, 0, 0}, {2, 2, 0}});
    world.Tolerances().Set(f1, {0.01});

    SurfaceComp surf;
    surf.surface_type = Type::Plane;
    surf.data = {0, 0, 0, 0, 0, 1, 1, 0, 0};  // location + normal + xdir
    world.Surfaces().Set(f1, surf);

    FaceTopoComp ftopo;
    ftopo.orientation = 0;
    ftopo.has_outer_wire = true;
    ftopo.outer_wire_orientation = 0;

    FaceTopoComp::WireEdgeRef ref;
    ref.edge_uid = e1;
    ref.orientation = 0;
    ref.pcurve.curve_type = Type::Line;
    ref.pcurve.first = 0.0;
    ref.pcurve.last = 1.0;
    ref.pcurve.data = {0, 0, 1, 0};  // 2d line
    ftopo.outer_wire_edges.push_back(ref);
    world.FaceTopos().Set(f1, std::move(ftopo));

    // Solid
    uint32_t s1 = 1000;
    world.RegisterEntity(s1);
    world.Types().Set(s1, Type::Solid);
    world.Aabbs().Set(s1, {{0, 0, 0}, {2, 2, 2}});

    SolidTopoComp stopo;
    SolidTopoComp::ShellComp shell;
    shell.orientation = 0;
    shell.face_uids = {f1};
    stopo.shells.push_back(shell);
    world.SolidTopos().Set(s1, std::move(stopo));

    return world;
}

// ============================================================
// Tests
// ============================================================

TEST_CASE("WorldFile save and load roundtrip", "[world_file]")
{
    auto world = make_test_world();
    const char* filepath = "/tmp/test_brepworld.cwld";

    REQUIRE(WorldFile::Save(filepath, world));

    BRepWorld loaded;
    REQUIRE(WorldFile::Load(filepath, loaded));

    CHECK(loaded.EntityCount() == world.EntityCount());

    // Vertex check
    CHECK(loaded.IsAlive(1));
    CHECK(*loaded.Types().Get(1) == Type::Vertex);
    CHECK(loaded.Positions().Get(1)->x == 0.05);
    CHECK(loaded.Tolerances().Get(1)->value == 0.001);

    // Edge check
    CHECK(loaded.IsAlive(10));
    CHECK(*loaded.Types().Get(10) == Type::Edge);
    CHECK(loaded.EdgeTopos().Get(10)->v_first == 1);
    CHECK(loaded.EdgeTopos().Get(10)->v_last == 2);
    CHECK(loaded.EdgeTopos().Get(10)->t_first == 0.0);
    CHECK(loaded.EdgeTopos().Get(10)->t_last == 1.0);
    CHECK(loaded.Curves().Get(10)->curve_type == Type::Line);
    CHECK(loaded.Curves().Get(10)->data.size() == 6);

    // Face check
    CHECK(loaded.IsAlive(100));
    CHECK(*loaded.Types().Get(100) == Type::Face);
    CHECK(loaded.Surfaces().Get(100)->surface_type == Type::Plane);
    CHECK(loaded.Surfaces().Get(100)->data.size() == 9);

    const auto* ft = loaded.FaceTopos().Get(100);
    REQUIRE(ft != nullptr);
    CHECK(ft->orientation == 0);
    CHECK(ft->has_outer_wire);
    REQUIRE(ft->outer_wire_edges.size() == 1);
    CHECK(ft->outer_wire_edges[0].edge_uid == 10);
    CHECK(ft->outer_wire_edges[0].pcurve.curve_type == Type::Line);
    CHECK(ft->outer_wire_edges[0].pcurve.data.size() == 4);

    // Solid check
    CHECK(loaded.IsAlive(1000));
    CHECK(*loaded.Types().Get(1000) == Type::Solid);
    const auto* st = loaded.SolidTopos().Get(1000);
    REQUIRE(st != nullptr);
    REQUIRE(st->shells.size() == 1);
    CHECK(st->shells[0].orientation == 0);
    REQUIRE(st->shells[0].face_uids.size() == 1);
    CHECK(st->shells[0].face_uids[0] == 100);

    // AABB check
    CHECK(loaded.Aabbs().Get(1000)->max_pt[2] == 2.0);

    std::remove(filepath);
}

TEST_CASE("WorldFile load nonexistent file returns false", "[world_file]")
{
    BRepWorld world;
    CHECK_FALSE(WorldFile::Load("/tmp/nonexistent_12345.cwld", world));
}

TEST_CASE("WorldFile load bad magic returns false", "[world_file]")
{
    const char* filepath = "/tmp/test_bad_magic.cwld";
    {
        std::ofstream os(filepath, std::ios::binary);
        uint32_t bad = 0xDEADBEEF;
        os.write(reinterpret_cast<const char*>(&bad), 4);
    }
    BRepWorld world;
    CHECK_FALSE(WorldFile::Load(filepath, world));
    std::remove(filepath);
}

TEST_CASE("BRepWorld ExportToPool produces valid data_pool from typed components", "[world_file]")
{
    BRepWorld world;

    // Vertex with typed components
    world.RegisterEntity(1);
    world.Types().Set(1, Type::Vertex);
    world.Aabbs().Set(1, {{0,0,0}, {0.1,0.1,0.1}});
    world.Positions().Set(1, {1.0, 2.0, 3.0});
    world.Tolerances().Set(1, {0.01});

    // Edge with typed components
    world.RegisterEntity(10);
    world.Types().Set(10, Type::Edge);
    world.Aabbs().Set(10, {{0,0,0}, {1,1,1}});
    world.Tolerances().Set(10, {0.05});
    EdgeTopoComp et;
    et.v_first = 1; et.v_last = UINT32_MAX;
    et.t_first = 0.0; et.t_last = 1.0;
    world.EdgeTopos().Set(10, et);
    CurveComp curve;
    curve.curve_type = Type::Line;
    curve.data = {0, 0, 0, 1, 0, 0};
    world.Curves().Set(10, curve);

    GeometryPool pool = world.ExportToPool();

    REQUIRE(pool.headers.size() == 2);
    CHECK(pool.headers[0].persistent_id == 1);
    CHECK(pool.headers[0].type == Type::Vertex);
    CHECK(pool.headers[0].param_count == 4);  // xyz + tol

    CHECK(pool.headers[1].persistent_id == 10);
    CHECK(pool.headers[1].type == Type::Edge);
    CHECK(pool.headers[1].param_count > 0);

    // Vertex data: x, y, z, tolerance
    uint32_t off = pool.headers[0].param_offset;
    CHECK(pool.data_pool[off]     == 1.0);
    CHECK(pool.data_pool[off + 1] == 2.0);
    CHECK(pool.data_pool[off + 2] == 3.0);
    CHECK(pool.data_pool[off + 3] == 0.01);

    // Edge data: v_first, v_last, tol, t_first, t_last, curve_type, curve_data...
    uint32_t eoff = pool.headers[1].param_offset;
    CHECK(pool.data_pool[eoff]     == 1.0);   // v_first uid
    CHECK(pool.data_pool[eoff + 1] == -1.0);  // v_last = UINT32_MAX -> -1
    CHECK(pool.data_pool[eoff + 2] == 0.05);  // tolerance
    CHECK(pool.data_pool[eoff + 3] == 0.0);   // t_first
    CHECK(pool.data_pool[eoff + 4] == 1.0);   // t_last
    CHECK(pool.data_pool[eoff + 5] == static_cast<double>(Type::Line)); // curve type
    CHECK(pool.data_pool[eoff + 6] == 0.0);   // curve data[0]
}

TEST_CASE("BRepWorld save/load then ExportToPool roundtrip", "[world_file]")
{
    auto world = make_test_world();
    const char* filepath = "/tmp/test_export_roundtrip.cwld";

    REQUIRE(WorldFile::Save(filepath, world));

    BRepWorld loaded;
    REQUIRE(WorldFile::Load(filepath, loaded));

    GeometryPool pool = loaded.ExportToPool();
    CHECK(pool.headers.size() == loaded.EntityCount());
    CHECK_FALSE(pool.data_pool.empty());

    // Every entity should have non-zero param_count
    for (auto& h : pool.headers)
        CHECK(h.param_count > 0);

    std::remove(filepath);
}

TEST_CASE("WorldFile empty world roundtrip", "[world_file]")
{
    BRepWorld world;
    const char* filepath = "/tmp/test_empty_world.cwld";

    REQUIRE(WorldFile::Save(filepath, world));

    BRepWorld loaded;
    REQUIRE(WorldFile::Load(filepath, loaded));
    CHECK(loaded.EntityCount() == 0);

    std::remove(filepath);
}
