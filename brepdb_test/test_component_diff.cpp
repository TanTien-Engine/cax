#include <catch2/catch_test_macros.hpp>

#include "ComponentDiff.h"

using namespace brepdb;

// ============================================================
// Helpers
// ============================================================

static BRepWorld make_world(uint32_t pid, Type type,
                            const std::vector<double>& params,
                            double min_val = 0.0, double max_val = 1.0)
{
    BRepWorld w;
    w.RegisterEntity(pid);
    w.Types().Set(pid, type);

    AabbComp aabb;
    aabb.min_pt[0] = aabb.min_pt[1] = aabb.min_pt[2] = min_val;
    aabb.max_pt[0] = aabb.max_pt[1] = aabb.max_pt[2] = max_val;
    w.Aabbs().Set(pid, aabb);

    ParamsComp pc;
    pc.data = params;
    w.Params().Set(pid, pc);

    return w;
}

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

// ============================================================
// Tests
// ============================================================

TEST_CASE("ComponentDiff detects no changes for identical worlds", "[component_diff]")
{
    auto w = make_world(10, Type::Face, {1.0, 2.0, 3.0});

    auto diff = ComponentDiff::Compute(w, w);
    CHECK(diff.IsEmpty());
}

TEST_CASE("ComponentDiff detects added entities", "[component_diff]")
{
    auto old_w = make_world(10, Type::Face, {1.0, 2.0});

    auto new_w = old_w;
    add_entity(new_w, 20, Type::Edge, {3.0, 4.0, 5.0});

    auto diff = ComponentDiff::Compute(old_w, new_w);
    REQUIRE(diff.added_entities.size() == 1);
    CHECK(diff.added_entities[0] == 20);
    CHECK(diff.removed_entities.empty());
    CHECK(diff.patches.empty());
}

TEST_CASE("ComponentDiff detects removed entities", "[component_diff]")
{
    BRepWorld old_w;
    add_entity(old_w, 10, Type::Face, {1.0, 2.0});
    add_entity(old_w, 20, Type::Edge, {3.0, 4.0, 5.0});

    auto new_w = make_world(10, Type::Face, {1.0, 2.0});

    auto diff = ComponentDiff::Compute(old_w, new_w);
    REQUIRE(diff.removed_entities.size() == 1);
    CHECK(diff.removed_entities[0] == 20);
    CHECK(diff.added_entities.empty());
    CHECK(diff.patches.empty());
}

TEST_CASE("ComponentDiff detects AABB-only change", "[component_diff]")
{
    auto old_w = make_world(10, Type::Face, {1.0, 2.0}, 0.0, 1.0);
    auto new_w = make_world(10, Type::Face, {1.0, 2.0}, 0.0, 5.0);

    auto diff = ComponentDiff::Compute(old_w, new_w);
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
    auto old_w = make_world(10, Type::Face, {1.0, 2.0, 3.0});
    auto new_w = make_world(10, Type::Face, {1.0, 2.0, 99.0});

    auto diff = ComponentDiff::Compute(old_w, new_w);
    REQUIRE(diff.patches.size() == 1);
    CHECK(diff.patches[0].kind == ComponentKind::Params);
    CHECK(diff.patches[0].old_data.size() == 3 * sizeof(double));
    CHECK(diff.patches[0].new_data.size() == 3 * sizeof(double));
}

TEST_CASE("ComponentDiff detects type change", "[component_diff]")
{
    auto old_w = make_world(10, Type::Line, {1.0, 2.0});
    auto new_w = make_world(10, Type::Circle, {1.0, 2.0});

    auto diff = ComponentDiff::Compute(old_w, new_w);
    REQUIRE(diff.patches.size() == 1);
    CHECK(diff.patches[0].kind == ComponentKind::Type);
    CHECK(diff.patches[0].old_data[0] == static_cast<uint8_t>(Type::Line));
    CHECK(diff.patches[0].new_data[0] == static_cast<uint8_t>(Type::Circle));
}

TEST_CASE("ComponentDiff is smaller than full entity diff for AABB-only change", "[component_diff]")
{
    std::vector<double> big_params(200, 1.0);
    auto old_w = make_world(10, Type::BSplineSurface, big_params, 0.0, 1.0);
    auto new_w = make_world(10, Type::BSplineSurface, big_params, 0.0, 2.0);

    auto diff = ComponentDiff::Compute(old_w, new_w);
    CHECK(diff.patches.size() == 1);
    CHECK(diff.patches[0].kind == ComponentKind::Aabb);
    CHECK(diff.PatchBytes() == 96);
}

TEST_CASE("ComponentDiff multiple components change on same entity", "[component_diff]")
{
    auto old_w = make_world(10, Type::Face, {1.0, 2.0}, 0.0, 1.0);
    auto new_w = make_world(10, Type::Face, {1.0, 99.0}, 0.0, 5.0);

    auto diff = ComponentDiff::Compute(old_w, new_w);
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

TEST_CASE("ComponentDiff ApplyForward reconstructs new world", "[component_diff]")
{
    BRepWorld old_w;
    add_entity(old_w, 10, Type::Face, {1.0, 2.0, 3.0}, 0.0, 1.0);
    add_entity(old_w, 20, Type::Edge, {4.0, 5.0}, 1.0, 2.0);

    BRepWorld new_w;
    add_entity(new_w, 10, Type::Face, {1.0, 2.0, 99.0}, 0.0, 1.0);
    add_entity(new_w, 20, Type::Edge, {4.0, 5.0}, 1.0, 9.0);

    auto diff = ComponentDiff::Compute(old_w, new_w);
    auto reconstructed = ComponentDiff::ApplyForward(old_w, diff);

    REQUIRE(reconstructed.EntityCount() == 2);

    auto* p10 = reconstructed.Params().Get(10);
    REQUIRE(p10);
    CHECK(p10->data[2] == 99.0);

    auto* a20 = reconstructed.Aabbs().Get(20);
    REQUIRE(a20);
    CHECK(a20->max_pt[0] == 9.0);
}

TEST_CASE("ComponentDiff ApplyForward handles removal", "[component_diff]")
{
    BRepWorld old_w;
    add_entity(old_w, 10, Type::Face, {1.0, 2.0});
    add_entity(old_w, 20, Type::Edge, {3.0, 4.0, 5.0});

    auto new_w = make_world(10, Type::Face, {1.0, 2.0});

    auto diff = ComponentDiff::Compute(old_w, new_w);
    auto reconstructed = ComponentDiff::ApplyForward(old_w, diff);

    REQUIRE(reconstructed.EntityCount() == 1);
    CHECK(reconstructed.IsAlive(10));
    CHECK(!reconstructed.IsAlive(20));
}

TEST_CASE("ComponentDiff ApplyReverse recovers old world", "[component_diff]")
{
    auto old_w = make_world(10, Type::Face, {1.0, 2.0, 3.0}, 0.0, 1.0);
    auto new_w = make_world(10, Type::Face, {1.0, 2.0, 99.0}, 0.0, 5.0);

    auto diff = ComponentDiff::Compute(old_w, new_w);
    auto recovered = ComponentDiff::ApplyReverse(new_w, diff);

    REQUIRE(recovered.EntityCount() == 1);
    CHECK(recovered.IsAlive(10));

    auto* aabb = recovered.Aabbs().Get(10);
    REQUIRE(aabb);
    CHECK(aabb->max_pt[0] == 1.0);

    auto* params = recovered.Params().Get(10);
    REQUIRE(params);
    CHECK(params->data[2] == 3.0);
}
