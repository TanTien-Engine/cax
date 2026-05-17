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
    REQUIRE(diff.added.size() == 1);
    CHECK(diff.added[0].id == 20);
    CHECK(diff.removed.empty());
    CHECK(diff.patches.empty());
}

TEST_CASE("ComponentDiff detects removed entities", "[component_diff]")
{
    BRepWorld old_w;
    add_entity(old_w, 10, Type::Face, {1.0, 2.0});
    add_entity(old_w, 20, Type::Edge, {3.0, 4.0, 5.0});

    auto new_w = make_world(10, Type::Face, {1.0, 2.0});

    auto diff = ComponentDiff::Compute(old_w, new_w);
    REQUIRE(diff.removed.size() == 1);
    CHECK(diff.removed[0].id == 20);
    CHECK(diff.added.empty());
    CHECK(diff.patches.empty());
}

TEST_CASE("ComponentDiff detects AABB-only change", "[component_diff]")
{
    auto old_w = make_world(10, Type::Face, {1.0, 2.0}, 0.0, 1.0);
    auto new_w = make_world(10, Type::Face, {1.0, 2.0}, 0.0, 5.0);

    auto diff = ComponentDiff::Compute(old_w, new_w);
    CHECK(diff.added.empty());
    CHECK(diff.removed.empty());
    REQUIRE(diff.patches.size() == 1);
    CHECK(diff.patches[0].entity_id == 10);
    CHECK(diff.patches[0].kind == ComponentKind::Aabb);
    CHECK(diff.patches[0].old_data.size() == 48);
    CHECK(diff.patches[0].new_data.size() == 48);
}

TEST_CASE("ComponentDiff detects params-only change via hunks", "[component_diff]")
{
    auto old_w = make_world(10, Type::Face, {1.0, 2.0, 3.0});
    auto new_w = make_world(10, Type::Face, {1.0, 2.0, 99.0});

    auto diff = ComponentDiff::Compute(old_w, new_w);
    REQUIRE(diff.patches.size() == 1);
    CHECK(diff.patches[0].kind == ComponentKind::Params);
    // Old/new data buffers should be empty -- hunks carry the delta
    CHECK(diff.patches[0].old_data.empty());
    CHECK(diff.patches[0].new_data.empty());
    CHECK(diff.patches[0].old_param_count == 3);
    CHECK(diff.patches[0].new_param_count == 3);
    // Only the changed range (index 2) should appear in the hunk
    REQUIRE(diff.patches[0].forward_hunks.size() == 1);
    CHECK(diff.patches[0].forward_hunks[0].offset == 2);
    CHECK(diff.patches[0].forward_hunks[0].data.size() == 1);
    CHECK(diff.patches[0].forward_hunks[0].data[0] == 99.0);
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

TEST_CASE("ComponentDiff hunk compression: AABB-only on big params", "[component_diff]")
{
    std::vector<double> big_params(200, 1.0);
    auto old_w = make_world(10, Type::BSplineSurface, big_params, 0.0, 1.0);
    auto new_w = make_world(10, Type::BSplineSurface, big_params, 0.0, 2.0);

    auto diff = ComponentDiff::Compute(old_w, new_w);
    REQUIRE(diff.patches.size() == 1);
    CHECK(diff.patches[0].kind == ComponentKind::Aabb);
    CHECK(diff.PatchBytes() == 96); // 48 bytes old + 48 bytes new
}

TEST_CASE("ComponentDiff hunk compression: small params delta on big array", "[component_diff]")
{
    std::vector<double> big(100, 1.0);
    auto old_w = make_world(10, Type::BSplineSurface, big);
    big[42] = 7.0;
    auto new_w = make_world(10, Type::BSplineSurface, big);

    auto diff = ComponentDiff::Compute(old_w, new_w);
    REQUIRE(diff.patches.size() == 1);
    CHECK(diff.patches[0].kind == ComponentKind::Params);
    // Only a tiny hunk, not full 100-double copies
    CHECK(diff.PatchBytes() < 200);
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

    REQUIRE(reconstructed->EntityCount() == 2);

    auto* p10 = reconstructed->Params().Get(10);
    REQUIRE(p10);
    CHECK(p10->data[2] == 99.0);

    auto* a20 = reconstructed->Aabbs().Get(20);
    REQUIRE(a20);
    CHECK(a20->max_pt[0] == 9.0);
}

TEST_CASE("ComponentDiff ApplyForward handles add", "[component_diff]")
{
    auto old_w = make_world(10, Type::Face, {1.0, 2.0});

    BRepWorld new_w;
    add_entity(new_w, 10, Type::Face, {1.0, 2.0});
    add_entity(new_w, 20, Type::Edge, {3.0, 4.0, 5.0});

    auto diff = ComponentDiff::Compute(old_w, new_w);
    auto fwd = ComponentDiff::ApplyForward(old_w, diff);

    REQUIRE(fwd->EntityCount() == 2);
    CHECK(fwd->IsAlive(20));
    auto* p20 = fwd->Params().Get(20);
    REQUIRE(p20);
    CHECK(p20->data.size() == 3);
    CHECK(p20->data[2] == 5.0);
}

TEST_CASE("ComponentDiff ApplyForward handles removal", "[component_diff]")
{
    BRepWorld old_w;
    add_entity(old_w, 10, Type::Face, {1.0, 2.0});
    add_entity(old_w, 20, Type::Edge, {3.0, 4.0, 5.0});

    auto new_w = make_world(10, Type::Face, {1.0, 2.0});

    auto diff = ComponentDiff::Compute(old_w, new_w);
    auto reconstructed = ComponentDiff::ApplyForward(old_w, diff);

    REQUIRE(reconstructed->EntityCount() == 1);
    CHECK(reconstructed->IsAlive(10));
    CHECK(!reconstructed->IsAlive(20));
}

TEST_CASE("ComponentDiff ApplyReverse recovers old world", "[component_diff]")
{
    auto old_w = make_world(10, Type::Face, {1.0, 2.0, 3.0}, 0.0, 1.0);
    auto new_w = make_world(10, Type::Face, {1.0, 2.0, 99.0}, 0.0, 5.0);

    auto diff = ComponentDiff::Compute(old_w, new_w);
    auto recovered = ComponentDiff::ApplyReverse(new_w, diff);

    REQUIRE(recovered->EntityCount() == 1);
    CHECK(recovered->IsAlive(10));

    auto* aabb = recovered->Aabbs().Get(10);
    REQUIRE(aabb);
    CHECK(aabb->max_pt[0] == 1.0);

    auto* params = recovered->Params().Get(10);
    REQUIRE(params);
    CHECK(params->data[2] == 3.0);
}

TEST_CASE("ComponentDiff round-trip with add/remove/modify", "[component_diff]")
{
    BRepWorld old_w;
    add_entity(old_w, 10, Type::Face, {1.0, 2.0, 3.0}, 0.0, 1.0);
    add_entity(old_w, 20, Type::Edge, {4.0, 5.0}, 1.0, 2.0);
    add_entity(old_w, 30, Type::Vertex, {6.0, 7.0}, 2.0, 3.0);

    BRepWorld new_w;
    add_entity(new_w, 10, Type::Face, {1.0, 2.0, 99.0}, 0.0, 1.0); // modified
    add_entity(new_w, 20, Type::Edge, {4.0, 5.0}, 1.0, 2.0);       // unchanged
    add_entity(new_w, 40, Type::Vertex, {8.0, 9.0}, 3.0, 4.0);     // 30 removed, 40 added

    auto diff = ComponentDiff::Compute(old_w, new_w);
    auto fwd = ComponentDiff::ApplyForward(old_w, diff);
    auto rev = ComponentDiff::ApplyReverse(*fwd, diff);

    CHECK(fwd->IsAlive(10));
    CHECK(fwd->IsAlive(20));
    CHECK(!fwd->IsAlive(30));
    CHECK(fwd->IsAlive(40));

    CHECK(rev->IsAlive(10));
    CHECK(rev->IsAlive(20));
    CHECK(rev->IsAlive(30));
    CHECK(!rev->IsAlive(40));
    CHECK(rev->Params().Get(10)->data[2] == 3.0);
}

TEST_CASE("ComponentDiff pid mapping: rename + modify", "[component_diff][pidmap]")
{
    BRepWorld old_w;
    add_entity(old_w, 100, Type::Compound, {1.0, 2.0, 3.0});
    add_entity(old_w, 200, Type::Compound, {4.0, 5.0});

    BRepWorld new_w;
    add_entity(new_w, 300, Type::Compound, {10.0, 20.0, 30.0});
    add_entity(new_w, 400, Type::Compound, {4.0, 5.0});

    ComponentDiff::PidMapping pid_map;
    pid_map[100] = { 300 };
    pid_map[200] = { 400 };

    auto diff = ComponentDiff::ComputeWithPidMapping(old_w, new_w, pid_map);
    CHECK(diff.renamed.size() == 2);

    auto fwd = ComponentDiff::ApplyForward(old_w, diff);
    REQUIRE(fwd->IsAlive(300));
    REQUIRE(fwd->IsAlive(400));
    CHECK(fwd->Params().Get(300)->data[0] == 10.0);

    auto rev = ComponentDiff::ApplyReverse(*fwd, diff);
    REQUIRE(rev->IsAlive(100));
    REQUIRE(rev->IsAlive(200));
    CHECK(rev->Params().Get(100)->data[0] == 1.0);
}

TEST_CASE("ComponentDiff pid mapping: entity deleted", "[component_diff][pidmap]")
{
    BRepWorld old_w;
    add_entity(old_w, 10, Type::Compound, {1.0, 2.0});
    add_entity(old_w, 20, Type::Compound, {3.0, 4.0});

    BRepWorld new_w;
    add_entity(new_w, 110, Type::Compound, {1.0, 2.0});

    ComponentDiff::PidMapping pid_map;
    pid_map[10] = { 110 };
    pid_map[20] = {}; // deleted

    auto diff = ComponentDiff::ComputeWithPidMapping(old_w, new_w, pid_map);
    auto fwd = ComponentDiff::ApplyForward(old_w, diff);
    CHECK(fwd->IsAlive(110));
    CHECK(!fwd->IsAlive(20));

    auto rev = ComponentDiff::ApplyReverse(*fwd, diff);
    CHECK(rev->IsAlive(10));
    CHECK(rev->IsAlive(20));
}

TEST_CASE("ComponentDiff preserves entity order", "[component_diff]")
{
    BRepWorld old_w;
    add_entity(old_w, 10, Type::Face, {1.0});
    add_entity(old_w, 20, Type::Edge, {2.0});
    add_entity(old_w, 30, Type::Vertex, {3.0});

    BRepWorld new_w;
    add_entity(new_w, 30, Type::Vertex, {3.0});
    add_entity(new_w, 10, Type::Face, {1.0});
    add_entity(new_w, 20, Type::Edge, {2.0});

    auto diff = ComponentDiff::Compute(old_w, new_w);
    CHECK(diff.old_order == std::vector<uint32_t>{10, 20, 30});
    CHECK(diff.new_order == std::vector<uint32_t>{30, 10, 20});

    auto fwd = ComponentDiff::ApplyForward(old_w, diff);
    REQUIRE(fwd->AliveEntities() == std::vector<uint32_t>{30, 10, 20});

    auto rev = ComponentDiff::ApplyReverse(*fwd, diff);
    REQUIRE(rev->AliveEntities() == std::vector<uint32_t>{10, 20, 30});
}
