#include <catch2/catch_test_macros.hpp>

#include "TypedPool.h"

using namespace brepdb;

// ============================================================
// ComponentPool<T> basic tests
// ============================================================

TEST_CASE("ComponentPool add and get", "[typed_pool]")
{
    ComponentPool<AabbComp> pool;
    REQUIRE(pool.Size() == 0);

    AabbComp a;
    a.min_pt[0] = 1; a.min_pt[1] = 2; a.min_pt[2] = 3;
    a.max_pt[0] = 4; a.max_pt[1] = 5; a.max_pt[2] = 6;
    pool.Set(10, a);

    REQUIRE(pool.Size() == 1);
    CHECK(pool.Has(10));
    CHECK_FALSE(pool.Has(99));

    const AabbComp* got = pool.Get(10);
    REQUIRE(got != nullptr);
    CHECK(got->min_pt[0] == 1);
    CHECK(got->max_pt[2] == 6);
}

TEST_CASE("ComponentPool overwrite existing", "[typed_pool]")
{
    ComponentPool<ToleranceComp> pool;
    pool.Set(5, {0.01});
    pool.Set(5, {0.02});

    REQUIRE(pool.Size() == 1);
    CHECK(pool.Get(5)->value == 0.02);
}

TEST_CASE("ComponentPool remove", "[typed_pool]")
{
    ComponentPool<ToleranceComp> pool;
    pool.Set(1, {0.1});
    pool.Set(2, {0.2});
    pool.Set(3, {0.3});
    REQUIRE(pool.Size() == 3);

    pool.Remove(2);
    REQUIRE(pool.Size() == 2);
    CHECK_FALSE(pool.Has(2));
    CHECK(pool.Has(1));
    CHECK(pool.Has(3));

    // Values still correct after swap-remove
    CHECK(pool.Get(1)->value == 0.1);
    CHECK(pool.Get(3)->value == 0.3);
}

TEST_CASE("ComponentPool remove last element", "[typed_pool]")
{
    ComponentPool<ToleranceComp> pool;
    pool.Set(7, {0.7});
    pool.Remove(7);
    REQUIRE(pool.Size() == 0);
    CHECK_FALSE(pool.Has(7));
}

TEST_CASE("ComponentPool remove nonexistent is no-op", "[typed_pool]")
{
    ComponentPool<ToleranceComp> pool;
    pool.Set(1, {0.1});
    pool.Remove(999);
    REQUIRE(pool.Size() == 1);
}

TEST_CASE("ComponentPool linear iteration", "[typed_pool]")
{
    ComponentPool<ToleranceComp> pool;
    pool.Set(10, {1.0});
    pool.Set(20, {2.0});
    pool.Set(30, {3.0});

    double sum = 0;
    for (size_t i = 0; i < pool.Size(); ++i)
        sum += pool.Data()[i].value;

    CHECK(sum == 6.0);
}

TEST_CASE("ComponentPool clear", "[typed_pool]")
{
    ComponentPool<PositionComp> pool;
    pool.Set(1, {1, 2, 3});
    pool.Set(2, {4, 5, 6});
    pool.Clear();
    REQUIRE(pool.Size() == 0);
    CHECK_FALSE(pool.Has(1));
}

// ============================================================
// BRepWorld tests
// ============================================================

TEST_CASE("BRepWorld create and destroy entities", "[typed_pool]")
{
    BRepWorld world;
    uint32_t e1 = world.CreateEntity();
    uint32_t e2 = world.CreateEntity();

    CHECK(world.EntityCount() == 2);
    CHECK(world.IsAlive(e1));
    CHECK(world.IsAlive(e2));

    world.DestroyEntity(e1);
    CHECK(world.EntityCount() == 1);
    CHECK_FALSE(world.IsAlive(e1));
    CHECK(world.IsAlive(e2));
}

TEST_CASE("BRepWorld component attach and query", "[typed_pool]")
{
    BRepWorld world;
    uint32_t e = world.CreateEntity();

    world.Types().Set(e, Type::Face);
    world.Tolerances().Set(e, {0.001});

    AabbComp aabb;
    aabb.min_pt[0] = 0; aabb.min_pt[1] = 0; aabb.min_pt[2] = 0;
    aabb.max_pt[0] = 1; aabb.max_pt[1] = 1; aabb.max_pt[2] = 1;
    world.Aabbs().Set(e, aabb);

    CHECK(*world.Types().Get(e) == Type::Face);
    CHECK(world.Tolerances().Get(e)->value == 0.001);
    CHECK(world.Aabbs().Get(e)->max_pt[0] == 1.0);
    CHECK(world.Params().Get(e) == nullptr);
}

TEST_CASE("BRepWorld destroy removes all components", "[typed_pool]")
{
    BRepWorld world;
    uint32_t e = world.CreateEntity();
    world.Types().Set(e, Type::Edge);
    world.Tolerances().Set(e, {0.01});

    ParamsComp pc;
    pc.data = {1.0, 2.0, 3.0};
    world.Params().Set(e, pc);

    world.DestroyEntity(e);
    CHECK_FALSE(world.Types().Has(e));
    CHECK_FALSE(world.Tolerances().Has(e));
    CHECK_FALSE(world.Params().Has(e));
}

// ============================================================
// Import / Export roundtrip
// ============================================================

static GeomHeader make_header(Type type, uint32_t pid,
                               uint32_t offset, uint32_t count,
                               double min_val = 0.0, double max_val = 1.0)
{
    GeomHeader h{};
    h.type          = type;
    h.persistent_id = pid;
    h.param_offset  = offset;
    h.param_count   = count;
    h.min_pt[0] = h.min_pt[1] = h.min_pt[2] = min_val;
    h.max_pt[0] = h.max_pt[1] = h.max_pt[2] = max_val;
    return h;
}

TEST_CASE("BRepWorld import from GeometryPool", "[typed_pool]")
{
    GeometryPool pool;
    pool.headers.push_back(make_header(Type::Vertex, 1, 0, 3, 0.0, 0.1));
    pool.headers.push_back(make_header(Type::Edge,  10, 3, 5, 0.0, 1.0));
    pool.headers.push_back(make_header(Type::Face, 100, 8, 4, 0.0, 2.0));
    pool.data_pool = {0, 0, 0, 1, 2, 3, 4, 5, 10, 20, 30, 40};

    BRepWorld world;
    world.ImportFromPool(pool);

    CHECK(world.EntityCount() == 3);
    CHECK(world.IsAlive(1));
    CHECK(world.IsAlive(10));
    CHECK(world.IsAlive(100));

    CHECK(*world.Types().Get(1) == Type::Vertex);
    CHECK(*world.Types().Get(10) == Type::Edge);
    CHECK(*world.Types().Get(100) == Type::Face);

    CHECK(world.Aabbs().Get(100)->max_pt[0] == 2.0);
    CHECK(world.Params().Get(10)->data.size() == 5);
    CHECK(world.Params().Get(10)->data[0] == 1.0);
}

TEST_CASE("BRepWorld export to GeometryPool roundtrip", "[typed_pool]")
{
    GeometryPool original;
    original.headers.push_back(make_header(Type::Face, 10, 0, 3, 0.0, 1.0));
    original.headers.push_back(make_header(Type::Edge, 20, 3, 2, 1.0, 2.0));
    original.data_pool = {1.0, 2.0, 3.0, 4.0, 5.0};

    BRepWorld world;
    world.ImportFromPool(original);

    GeometryPool exported = world.ExportToPool();

    REQUIRE(exported.headers.size() == 2);
    CHECK(exported.headers[0].persistent_id == 10);
    CHECK(exported.headers[1].persistent_id == 20);
    CHECK(*exported.headers[0].min_pt == 0.0);
    CHECK(*exported.headers[1].max_pt == 2.0);

    // Params preserved
    uint32_t off0 = exported.headers[0].param_offset;
    uint32_t off1 = exported.headers[1].param_offset;
    CHECK(exported.data_pool[off0]     == 1.0);
    CHECK(exported.data_pool[off0 + 2] == 3.0);
    CHECK(exported.data_pool[off1]     == 4.0);
    CHECK(exported.data_pool[off1 + 1] == 5.0);
}

TEST_CASE("BRepWorld modify component then export", "[typed_pool]")
{
    GeometryPool original;
    original.headers.push_back(make_header(Type::Face, 10, 0, 2, 0.0, 1.0));
    original.data_pool = {1.0, 2.0};

    BRepWorld world;
    world.ImportFromPool(original);

    // Modify only the AABB
    AabbComp* aabb = world.Aabbs().Get(10);
    aabb->max_pt[0] = 99.0;

    GeometryPool exported = world.ExportToPool();
    CHECK(exported.headers[0].max_pt[0] == 99.0);
    // Params unchanged
    CHECK(exported.data_pool[0] == 1.0);
    CHECK(exported.data_pool[1] == 2.0);
}

TEST_CASE("BRepWorld add entity after import then export", "[typed_pool]")
{
    GeometryPool original;
    original.headers.push_back(make_header(Type::Face, 10, 0, 2));
    original.data_pool = {1.0, 2.0};

    BRepWorld world;
    world.ImportFromPool(original);

    uint32_t e = world.CreateEntity();
    world.Types().Set(e, Type::Vertex);
    AabbComp aabb;
    aabb.min_pt[0] = aabb.min_pt[1] = aabb.min_pt[2] = 5.0;
    aabb.max_pt[0] = aabb.max_pt[1] = aabb.max_pt[2] = 5.0;
    world.Aabbs().Set(e, aabb);
    ParamsComp pc;
    pc.data = {5.0, 5.0, 5.0};
    world.Params().Set(e, pc);

    GeometryPool exported = world.ExportToPool();
    REQUIRE(exported.headers.size() == 2);

    // Find the new entity
    bool found = false;
    for (auto& h : exported.headers)
    {
        if (h.persistent_id == e)
        {
            found = true;
            CHECK(h.type == Type::Vertex);
            CHECK(h.min_pt[0] == 5.0);
            CHECK(h.param_count == 3);
        }
    }
    CHECK(found);
}

TEST_CASE("BRepWorld destroy entity then export", "[typed_pool]")
{
    GeometryPool original;
    original.headers.push_back(make_header(Type::Face, 10, 0, 2));
    original.headers.push_back(make_header(Type::Edge, 20, 2, 3));
    original.data_pool = {1, 2, 3, 4, 5};

    BRepWorld world;
    world.ImportFromPool(original);
    world.DestroyEntity(10);

    GeometryPool exported = world.ExportToPool();
    REQUIRE(exported.headers.size() == 1);
    CHECK(exported.headers[0].persistent_id == 20);
}

// ============================================================
// Spatial query using typed pools directly
// ============================================================

TEST_CASE("BRepWorld spatial query via Aabbs pool", "[typed_pool]")
{
    BRepWorld world;

    // Create 3 entities with known bboxes
    uint32_t e1 = world.CreateEntity();
    AabbComp a1 = {{0,0,0}, {1,1,1}};
    world.Aabbs().Set(e1, a1);

    uint32_t e2 = world.CreateEntity();
    AabbComp a2 = {{5,5,5}, {6,6,6}};
    world.Aabbs().Set(e2, a2);

    uint32_t e3 = world.CreateEntity();
    AabbComp a3 = {{0.5,0.5,0.5}, {2,2,2}};
    world.Aabbs().Set(e3, a3);

    // Query box (0,0,0)-(1.5,1.5,1.5) should hit e1 and e3
    double qmin[3] = {0, 0, 0};
    double qmax[3] = {1.5, 1.5, 1.5};

    std::vector<uint32_t> hits;
    const auto& aabbs = world.Aabbs();
    for (size_t i = 0; i < aabbs.Size(); ++i)
    {
        const AabbComp& ab = aabbs.Data()[i];
        if (ab.max_pt[0] < qmin[0] || ab.min_pt[0] > qmax[0]) continue;
        if (ab.max_pt[1] < qmin[1] || ab.min_pt[1] > qmax[1]) continue;
        if (ab.max_pt[2] < qmin[2] || ab.min_pt[2] > qmax[2]) continue;
        hits.push_back(aabbs.EntityIds()[i]);
    }

    REQUIRE(hits.size() == 2);
    CHECK(std::find(hits.begin(), hits.end(), e1) != hits.end());
    CHECK(std::find(hits.begin(), hits.end(), e3) != hits.end());
}
