#include <catch2/catch_test_macros.hpp>

#include "ComponentView.h"

using namespace brepdb;

// ============================================================
// Helpers
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

static GeometryPool make_mixed_pool()
{
    GeometryPool pool;
    // vertex pid=1: point at (0,0,0)-(0.1,0.1,0.1), 3 params
    pool.headers.push_back(make_header(Type::Vertex, 1, 0, 3, 0.0, 0.1));
    // vertex pid=2: point at (1,1,1)-(1.1,1.1,1.1), 3 params
    pool.headers.push_back(make_header(Type::Vertex, 2, 3, 3, 1.0, 1.1));
    // edge pid=10: bbox (0,0,0)-(1.1,1.1,1.1), 5 params (curve data)
    pool.headers.push_back(make_header(Type::Edge, 10, 6, 5, 0.0, 1.1));
    // face pid=100: bbox (0,0,0)-(2,2,2), 8 params (surface data)
    pool.headers.push_back(make_header(Type::Face, 100, 11, 8, 0.0, 2.0));
    // face pid=101: bbox (5,5,5)-(6,6,6), 4 params
    pool.headers.push_back(make_header(Type::Face, 101, 19, 4, 5.0, 6.0));

    pool.data_pool.resize(23);
    for (size_t i = 0; i < 23; ++i)
        pool.data_pool[i] = static_cast<double>(i) * 0.5;
    return pool;
}

// ============================================================
// AabbView tests
// ============================================================

TEST_CASE("AabbView builds from pool", "[component_view]")
{
    auto pool = make_mixed_pool();
    AabbView view;
    view.Build(pool);

    REQUIRE(view.Size() == 5);
    REQUIRE(view.entity_ids[0] == 1);
    REQUIRE(view.entity_ids[4] == 101);
    REQUIRE(view.min_x[0] == 0.0);
    REQUIRE(view.max_x[3] == 2.0);
}

TEST_CASE("AabbView QueryBox returns intersecting entities", "[component_view]")
{
    auto pool = make_mixed_pool();
    AabbView view;
    view.Build(pool);

    SECTION("Query that hits vertices and edge and face100")
    {
        double qmin[3] = {0.0, 0.0, 0.0};
        double qmax[3] = {0.5, 0.5, 0.5};
        auto result = view.QueryBox(qmin, qmax);
        // Should hit: vertex1 (0-0.1), edge10 (0-1.1), face100 (0-2)
        // Should NOT hit: vertex2 (1-1.1), face101 (5-6)
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

TEST_CASE("AabbView handles empty pool", "[component_view]")
{
    GeometryPool pool;
    AabbView view;
    view.Build(pool);
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
    auto pool = make_mixed_pool();
    TypeView view;
    view.Build(pool);

    REQUIRE(view.Size() == 5);

    auto vertices = view.GetByType(Type::Vertex);
    REQUIRE(vertices.size() == 2);
    CHECK(vertices[0] == 1);
    CHECK(vertices[1] == 2);

    auto edges = view.GetByType(Type::Edge);
    REQUIRE(edges.size() == 1);
    CHECK(edges[0] == 10);

    auto faces = view.GetByType(Type::Face);
    REQUIRE(faces.size() == 2);
    CHECK(faces[0] == 100);
    CHECK(faces[1] == 101);

    auto solids = view.GetByType(Type::Solid);
    REQUIRE(solids.empty());
}

// ============================================================
// ParamView tests
// ============================================================

TEST_CASE("ParamView gives access to params by index", "[component_view]")
{
    auto pool = make_mixed_pool();
    ParamView view;
    view.Build(pool);

    REQUIRE(view.Size() == 5);

    uint32_t count0 = 0;
    const double* p0 = view.GetParams(0, count0);
    REQUIRE(count0 == 3);
    CHECK(p0[0] == 0.0);
    CHECK(p0[1] == 0.5);
    CHECK(p0[2] == 1.0);

    uint32_t count4 = 0;
    view.GetParams(4, count4);
    REQUIRE(count4 == 4);
}

TEST_CASE("ParamView FindIndex by persistent_id", "[component_view]")
{
    auto pool = make_mixed_pool();
    ParamView view;
    view.Build(pool);

    CHECK(view.FindIndex(1)   == 0);
    CHECK(view.FindIndex(2)   == 1);
    CHECK(view.FindIndex(10)  == 2);
    CHECK(view.FindIndex(100) == 3);
    CHECK(view.FindIndex(101) == 4);
    CHECK(view.FindIndex(999) == -1);
}

// ============================================================
// PoolViews composite tests
// ============================================================

TEST_CASE("PoolViews builds all views together", "[component_view]")
{
    auto pool = make_mixed_pool();
    PoolViews views;
    views.Build(pool);

    CHECK(views.aabbs.Size() == 5);
    CHECK(views.types.Size() == 5);
    CHECK(views.params.Size() == 5);

    // Cross-check: entity_ids consistent across views
    for (size_t i = 0; i < 5; ++i)
    {
        CHECK(views.aabbs.entity_ids[i] == views.types.entity_ids[i]);
        CHECK(views.aabbs.entity_ids[i] == views.params.entity_ids[i]);
    }
}
