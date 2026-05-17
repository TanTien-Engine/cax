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
// RegisterEntity and component roundtrip
// ============================================================

static void add_entity(BRepWorld& w, uint32_t pid, Type type,
                       const std::vector<double>& params,
                       double min_val = 0.0, double max_val = 1.0)
{
    w.RegisterEntity(pid);
    w.Types().Set(pid, type);
    AabbComp aabb;
    aabb.min_pt[0] = aabb.min_pt[1] = aabb.min_pt[2] = min_val;
    aabb.max_pt[0] = aabb.max_pt[1] = aabb.max_pt[2] = max_val;
    w.Aabbs().Set(pid, aabb);
    if (!params.empty()) {
        ParamsComp pc;
        pc.data = params;
        w.Params().Set(pid, pc);
    }
}

TEST_CASE("BRepWorld RegisterEntity with components", "[typed_pool]")
{
    BRepWorld world;
    add_entity(world, 1,   Type::Vertex, {0, 0, 0},       0.0, 0.1);
    add_entity(world, 10,  Type::Edge,   {1, 2, 3, 4, 5}, 0.0, 1.0);
    add_entity(world, 100, Type::Face,   {10, 20, 30, 40}, 0.0, 2.0);

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

TEST_CASE("BRepWorld params roundtrip", "[typed_pool]")
{
    BRepWorld world;
    add_entity(world, 10, Type::Face, {1.0, 2.0, 3.0}, 0.0, 1.0);
    add_entity(world, 20, Type::Edge, {4.0, 5.0},       1.0, 2.0);

    CHECK(world.EntityCount() == 2);
    CHECK(world.Aabbs().Get(10)->min_pt[0] == 0.0);
    CHECK(world.Aabbs().Get(20)->max_pt[0] == 2.0);
    CHECK(world.Params().Get(10)->data[2] == 3.0);
    CHECK(world.Params().Get(20)->data[1] == 5.0);
}

TEST_CASE("BRepWorld modify component", "[typed_pool]")
{
    BRepWorld world;
    add_entity(world, 10, Type::Face, {1.0, 2.0}, 0.0, 1.0);

    AabbComp* aabb = world.Aabbs().Get(10);
    aabb->max_pt[0] = 99.0;

    CHECK(world.Aabbs().Get(10)->max_pt[0] == 99.0);
    CHECK(world.Params().Get(10)->data[0] == 1.0);
    CHECK(world.Params().Get(10)->data[1] == 2.0);
}

TEST_CASE("BRepWorld add entity then verify", "[typed_pool]")
{
    BRepWorld world;
    add_entity(world, 10, Type::Face, {1.0, 2.0});

    uint32_t e = world.CreateEntity();
    world.Types().Set(e, Type::Vertex);
    AabbComp aabb;
    aabb.min_pt[0] = aabb.min_pt[1] = aabb.min_pt[2] = 5.0;
    aabb.max_pt[0] = aabb.max_pt[1] = aabb.max_pt[2] = 5.0;
    world.Aabbs().Set(e, aabb);
    ParamsComp pc;
    pc.data = {5.0, 5.0, 5.0};
    world.Params().Set(e, pc);

    REQUIRE(world.EntityCount() == 2);
    CHECK(*world.Types().Get(e) == Type::Vertex);
    CHECK(world.Aabbs().Get(e)->min_pt[0] == 5.0);
    CHECK(world.Params().Get(e)->data.size() == 3);
}

TEST_CASE("BRepWorld destroy entity", "[typed_pool]")
{
    BRepWorld world;
    add_entity(world, 10, Type::Face, {1, 2});
    add_entity(world, 20, Type::Edge, {3, 4, 5});
    world.DestroyEntity(10);

    REQUIRE(world.EntityCount() == 1);
    CHECK_FALSE(world.IsAlive(10));
    CHECK(world.IsAlive(20));
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
