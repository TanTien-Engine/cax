#include <catch2/catch_test_macros.hpp>

#include "ComponentView.h"

using namespace brepdb;

// ============================================================
// Helpers
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

    ParamsComp pc;
    pc.data = params;
    w.Params().Set(pid, pc);
}

static BRepWorld make_mixed_world()
{
    BRepWorld w;
    // vertex pid=1: point at (0,0,0)-(0.1,0.1,0.1), 3 params
    add_entity(w, 1, Type::Vertex, {0.0, 0.5, 1.0}, 0.0, 0.1);
    // vertex pid=2: point at (1,1,1)-(1.1,1.1,1.1), 3 params
    add_entity(w, 2, Type::Vertex, {1.5, 2.0, 2.5}, 1.0, 1.1);
    // edge pid=10: bbox (0,0,0)-(1.1,1.1,1.1), 5 params
    add_entity(w, 10, Type::Edge, {3.0, 3.5, 4.0, 4.5, 5.0}, 0.0, 1.1);
    // face pid=100: bbox (0,0,0)-(2,2,2), 8 params
    add_entity(w, 100, Type::Face, {5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0}, 0.0, 2.0);
    // face pid=101: bbox (5,5,5)-(6,6,6), 4 params
    add_entity(w, 101, Type::Face, {9.5, 10.0, 10.5, 11.0}, 5.0, 6.0);
    return w;
}

// ============================================================
// AabbView tests
// ============================================================

TEST_CASE("AabbView builds from world", "[component_view]")
{
    auto world = make_mixed_world();
    AabbView view;
    view.Build(world);

    REQUIRE(view.Size() == 5);
    CHECK(view.min_x[0] == 0.0);
}

TEST_CASE("AabbView QueryBox returns intersecting entities", "[component_view]")
{
    auto world = make_mixed_world();
    AabbView view;
    view.Build(world);

    SECTION("Query that hits vertices and edge and face100")
    {
        double qmin[3] = {0.0, 0.0, 0.0};
        double qmax[3] = {0.5, 0.5, 0.5};
        auto result = view.QueryBox(qmin, qmax);
        REQUIRE(result.size() == 3);
        CHECK(std::find(result.begin(), result.end(), 1) != result.end());
        CHECK(std::find(result.begin(), result.end(), 10) != result.end());
        CHECK(std::find(result.begin(), result.end(), 100) != result.end());
    }

    SECTION("Query that hits only face101")
    {
        double qmin[3] = {5.5, 5.5, 5.5};
        double qmax[3] = {5.8, 5.8, 5.8};
        auto result = view.QueryBox(qmin, qmax);
        REQUIRE(result.size() == 1);
        CHECK(result[0] == 101);
    }

    SECTION("Query that hits nothing")
    {
        double qmin[3] = {10.0, 10.0, 10.0};
        double qmax[3] = {11.0, 11.0, 11.0};
        auto result = view.QueryBox(qmin, qmax);
        REQUIRE(result.empty());
    }

    SECTION("Query that hits everything")
    {
        double qmin[3] = {-100.0, -100.0, -100.0};
        double qmax[3] = {100.0, 100.0, 100.0};
        auto result = view.QueryBox(qmin, qmax);
        REQUIRE(result.size() == 5);
    }
}

TEST_CASE("AabbView handles empty world", "[component_view]")
{
    BRepWorld world;
    AabbView view;
    view.Build(world);
    REQUIRE(view.Size() == 0);

    double qmin[3] = {0, 0, 0};
    double qmax[3] = {1, 1, 1};
    REQUIRE(view.QueryBox(qmin, qmax).empty());
}

// ============================================================
// TypeView tests
// ============================================================

TEST_CASE("TypeView filters by type", "[component_view]")
{
    auto world = make_mixed_world();
    TypeView view;
    view.Build(world);

    REQUIRE(view.Size() == 5);

    auto vertices = view.GetByType(Type::Vertex);
    REQUIRE(vertices.size() == 2);

    auto edges = view.GetByType(Type::Edge);
    REQUIRE(edges.size() == 1);
    CHECK(edges[0] == 10);

    auto faces = view.GetByType(Type::Face);
    REQUIRE(faces.size() == 2);

    auto solids = view.GetByType(Type::Solid);
    REQUIRE(solids.empty());
}

// ============================================================
// ParamView tests
// ============================================================

TEST_CASE("ParamView gives access to params by index", "[component_view]")
{
    auto world = make_mixed_world();
    ParamView view;
    view.Build(world);

    REQUIRE(view.Size() == 5);

    auto idx = view.FindIndex(1);
    REQUIRE(idx >= 0);
    const auto& p0 = view.GetParams(idx);
    REQUIRE(p0.size() == 3);
    CHECK(p0[0] == 0.0);
    CHECK(p0[1] == 0.5);
    CHECK(p0[2] == 1.0);

    auto idx101 = view.FindIndex(101);
    REQUIRE(idx101 >= 0);
    const auto& p4 = view.GetParams(idx101);
    REQUIRE(p4.size() == 4);
}

TEST_CASE("ParamView FindIndex by persistent_id", "[component_view]")
{
    auto world = make_mixed_world();
    ParamView view;
    view.Build(world);

    CHECK(view.FindIndex(1)   >= 0);
    CHECK(view.FindIndex(2)   >= 0);
    CHECK(view.FindIndex(10)  >= 0);
    CHECK(view.FindIndex(100) >= 0);
    CHECK(view.FindIndex(101) >= 0);
    CHECK(view.FindIndex(999) == -1);
}

// ============================================================
// WorldViews composite tests
// ============================================================

TEST_CASE("WorldViews builds all views together", "[component_view]")
{
    auto world = make_mixed_world();
    WorldViews views;
    views.Build(world);

    CHECK(views.aabbs.Size() == 5);
    CHECK(views.types.Size() == 5);
    CHECK(views.params.Size() == 5);

    for (size_t i = 0; i < 5; ++i)
    {
        CHECK(views.aabbs.entity_ids[i] == views.types.entity_ids[i]);
        CHECK(views.aabbs.entity_ids[i] == views.params.entity_ids[i]);
    }
}
