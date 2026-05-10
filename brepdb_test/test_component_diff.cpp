#include <catch2/catch_test_macros.hpp>

#include "ComponentDiff.h"

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

// ============================================================
// Tests
// ============================================================

TEST_CASE("ComponentDiff detects no changes for identical pools", "[component_diff]")
{
    GeometryPool pool;
    pool.headers.push_back(make_header(Type::Face, 10, 0, 3));
    pool.data_pool = {1.0, 2.0, 3.0};

    auto diff = ComponentDiff::Compute(pool, pool);
    CHECK(diff.IsEmpty());
}

TEST_CASE("ComponentDiff detects added entities", "[component_diff]")
{
    GeometryPool old_pool;
    old_pool.headers.push_back(make_header(Type::Face, 10, 0, 2));
    old_pool.data_pool = {1.0, 2.0};

    GeometryPool new_pool = old_pool;
    new_pool.headers.push_back(make_header(Type::Edge, 20, 2, 3));
    new_pool.data_pool.insert(new_pool.data_pool.end(), {3.0, 4.0, 5.0});

    auto diff = ComponentDiff::Compute(old_pool, new_pool);
    REQUIRE(diff.added_entities.size() == 1);
    CHECK(diff.added_entities[0] == 20);
    CHECK(diff.removed_entities.empty());
    CHECK(diff.patches.empty());
}

TEST_CASE("ComponentDiff detects removed entities", "[component_diff]")
{
    GeometryPool old_pool;
    old_pool.headers.push_back(make_header(Type::Face, 10, 0, 2));
    old_pool.headers.push_back(make_header(Type::Edge, 20, 2, 3));
    old_pool.data_pool = {1.0, 2.0, 3.0, 4.0, 5.0};

    GeometryPool new_pool;
    new_pool.headers.push_back(make_header(Type::Face, 10, 0, 2));
    new_pool.data_pool = {1.0, 2.0};

    auto diff = ComponentDiff::Compute(old_pool, new_pool);
    REQUIRE(diff.removed_entities.size() == 1);
    CHECK(diff.removed_entities[0] == 20);
    CHECK(diff.added_entities.empty());
    CHECK(diff.patches.empty());
}

TEST_CASE("ComponentDiff detects AABB-only change", "[component_diff]")
{
    GeometryPool old_pool;
    old_pool.headers.push_back(make_header(Type::Face, 10, 0, 2, 0.0, 1.0));
    old_pool.data_pool = {1.0, 2.0};

    GeometryPool new_pool;
    new_pool.headers.push_back(make_header(Type::Face, 10, 0, 2, 0.0, 5.0));
    new_pool.data_pool = {1.0, 2.0};

    auto diff = ComponentDiff::Compute(old_pool, new_pool);
    CHECK(diff.added_entities.empty());
    CHECK(diff.removed_entities.empty());
    REQUIRE(diff.patches.size() == 1);
    CHECK(diff.patches[0].entity_id == 10);
    CHECK(diff.patches[0].kind == ComponentKind::Aabb);
    CHECK(diff.patches[0].old_data.size() == 48);
    CHECK(diff.patches[0].new_data.size() == 48);
}

TEST_CASE("ComponentDiff detects params-only change", "[component_diff]")
{
    GeometryPool old_pool;
    old_pool.headers.push_back(make_header(Type::Face, 10, 0, 3));
    old_pool.data_pool = {1.0, 2.0, 3.0};

    GeometryPool new_pool;
    new_pool.headers.push_back(make_header(Type::Face, 10, 0, 3));
    new_pool.data_pool = {1.0, 2.0, 99.0};  // only last param changed

    auto diff = ComponentDiff::Compute(old_pool, new_pool);
    REQUIRE(diff.patches.size() == 1);
    CHECK(diff.patches[0].kind == ComponentKind::Params);
    CHECK(diff.patches[0].old_data.size() == 3 * sizeof(double));
    CHECK(diff.patches[0].new_data.size() == 3 * sizeof(double));
}

TEST_CASE("ComponentDiff detects type change", "[component_diff]")
{
    GeometryPool old_pool;
    old_pool.headers.push_back(make_header(Type::Line, 10, 0, 2));
    old_pool.data_pool = {1.0, 2.0};

    GeometryPool new_pool;
    new_pool.headers.push_back(make_header(Type::Circle, 10, 0, 2));
    new_pool.data_pool = {1.0, 2.0};

    auto diff = ComponentDiff::Compute(old_pool, new_pool);
    REQUIRE(diff.patches.size() == 1);
    CHECK(diff.patches[0].kind == ComponentKind::Type);
    CHECK(diff.patches[0].old_data[0] == static_cast<uint8_t>(Type::Line));
    CHECK(diff.patches[0].new_data[0] == static_cast<uint8_t>(Type::Circle));
}

TEST_CASE("ComponentDiff is smaller than full entity diff for AABB-only change", "[component_diff]")
{
    // Entity with large params but only AABB changes
    GeometryPool old_pool;
    auto h = make_header(Type::BSplineSurface, 10, 0, 200, 0.0, 1.0);
    old_pool.headers.push_back(h);
    old_pool.data_pool.resize(200, 1.0);

    GeometryPool new_pool;
    h.max_pt[0] = h.max_pt[1] = h.max_pt[2] = 2.0;  // only AABB changed
    new_pool.headers.push_back(h);
    new_pool.data_pool.resize(200, 1.0);

    auto diff = ComponentDiff::Compute(old_pool, new_pool);
    CHECK(diff.patches.size() == 1);
    CHECK(diff.patches[0].kind == ComponentKind::Aabb);
    // Only 96 bytes (2x48 for old+new AABB) instead of 2x1600 bytes for full params
    CHECK(diff.PatchBytes() == 96);
}

TEST_CASE("ComponentDiff multiple components change on same entity", "[component_diff]")
{
    GeometryPool old_pool;
    old_pool.headers.push_back(make_header(Type::Face, 10, 0, 2, 0.0, 1.0));
    old_pool.data_pool = {1.0, 2.0};

    GeometryPool new_pool;
    new_pool.headers.push_back(make_header(Type::Face, 10, 0, 2, 0.0, 5.0));
    new_pool.data_pool = {1.0, 99.0};  // AABB + params both changed

    auto diff = ComponentDiff::Compute(old_pool, new_pool);
    CHECK(diff.patches.size() == 2);

    bool has_aabb = false, has_params = false;
    for (auto& p : diff.patches)
    {
        if (p.kind == ComponentKind::Aabb)   has_aabb = true;
        if (p.kind == ComponentKind::Params) has_params = true;
    }
    CHECK(has_aabb);
    CHECK(has_params);
}

TEST_CASE("ComponentDiff ApplyForward reconstructs new pool", "[component_diff]")
{
    GeometryPool old_pool;
    old_pool.headers.push_back(make_header(Type::Face, 10, 0, 3, 0.0, 1.0));
    old_pool.headers.push_back(make_header(Type::Edge, 20, 3, 2, 1.0, 2.0));
    old_pool.data_pool = {1.0, 2.0, 3.0, 4.0, 5.0};

    // Change: face10 params change, edge20 AABB changes
    GeometryPool new_pool;
    new_pool.headers.push_back(make_header(Type::Face, 10, 0, 3, 0.0, 1.0));
    new_pool.headers.push_back(make_header(Type::Edge, 20, 3, 2, 1.0, 9.0));
    new_pool.data_pool = {1.0, 2.0, 99.0, 4.0, 5.0};

    auto diff = ComponentDiff::Compute(old_pool, new_pool);
    auto reconstructed = ComponentDiff::ApplyForward(old_pool, diff);

    REQUIRE(reconstructed.headers.size() == 2);
    CHECK(reconstructed.headers[0].persistent_id == 10);
    CHECK(reconstructed.headers[1].persistent_id == 20);

    // Check face10 params updated
    uint32_t off0 = reconstructed.headers[0].param_offset;
    CHECK(reconstructed.data_pool[off0 + 2] == 99.0);

    // Check edge20 AABB updated
    CHECK(reconstructed.headers[1].max_pt[0] == 9.0);
}

TEST_CASE("ComponentDiff ApplyForward handles removal", "[component_diff]")
{
    GeometryPool old_pool;
    old_pool.headers.push_back(make_header(Type::Face, 10, 0, 2));
    old_pool.headers.push_back(make_header(Type::Edge, 20, 2, 3));
    old_pool.data_pool = {1.0, 2.0, 3.0, 4.0, 5.0};

    GeometryPool new_pool;
    new_pool.headers.push_back(make_header(Type::Face, 10, 0, 2));
    new_pool.data_pool = {1.0, 2.0};

    auto diff = ComponentDiff::Compute(old_pool, new_pool);
    auto reconstructed = ComponentDiff::ApplyForward(old_pool, diff);

    REQUIRE(reconstructed.headers.size() == 1);
    CHECK(reconstructed.headers[0].persistent_id == 10);
}

TEST_CASE("ComponentDiff ApplyReverse recovers old pool", "[component_diff]")
{
    GeometryPool old_pool;
    old_pool.headers.push_back(make_header(Type::Face, 10, 0, 3, 0.0, 1.0));
    old_pool.data_pool = {1.0, 2.0, 3.0};

    GeometryPool new_pool;
    new_pool.headers.push_back(make_header(Type::Face, 10, 0, 3, 0.0, 5.0));
    new_pool.data_pool = {1.0, 2.0, 99.0};

    auto diff = ComponentDiff::Compute(old_pool, new_pool);
    auto recovered = ComponentDiff::ApplyReverse(new_pool, diff);

    REQUIRE(recovered.headers.size() == 1);
    CHECK(recovered.headers[0].persistent_id == 10);
    CHECK(recovered.headers[0].max_pt[0] == 1.0);
    uint32_t off = recovered.headers[0].param_offset;
    CHECK(recovered.data_pool[off + 2] == 3.0);
}
