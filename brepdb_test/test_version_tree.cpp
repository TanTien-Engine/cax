#include <catch2/catch_test_macros.hpp>

#include "VersionTree.h"
#include "TypedPool.h"

#include <numeric>
#include <set>
#include <string>

using namespace brepdb;

// ============================================================
// Helpers
// ============================================================

static void add_entity(BRepWorld& w, uint32_t pid, Type type,
                       const std::vector<double>& params)
{
    w.RegisterEntity(pid);
    w.Types().Set(pid, type);
    AabbComp aabb;
    aabb.min_pt[0] = aabb.min_pt[1] = aabb.min_pt[2] = 0.0;
    aabb.max_pt[0] = aabb.max_pt[1] = aabb.max_pt[2] = 1.0;
    w.Aabbs().Set(pid, aabb);
    if (!params.empty()) {
        ParamsComp pc;
        pc.data = params;
        w.Params().Set(pid, pc);
    }
}

// world: [a(pid=10, 3 doubles), b(pid=20, 2 doubles)]
static BRepWorld make_world_ab()
{
    BRepWorld w;
    add_entity(w, 10, Type::Compound, { 1, 2, 3 });
    add_entity(w, 20, Type::Compound, { 4, 5 });
    return w;
}

// world: [c(pid=30), a(pid=10), b(pid=20)]
static BRepWorld make_world_cab()
{
    BRepWorld w;
    add_entity(w, 30, Type::Compound, { 6, 7, 8, 9 });
    add_entity(w, 10, Type::Compound, { 1, 2, 3 });
    add_entity(w, 20, Type::Compound, { 4, 5 });
    return w;
}

static std::vector<double> get_params(const BRepWorld& w, uint32_t pid)
{
    return w.ExportEntityParams(pid);
}

static std::vector<double> get_params(const WorldPtr& w, uint32_t pid)
{
    return get_params(*w, pid);
}

static std::vector<uint32_t> get_alive(const WorldPtr& w)
{
    return w->AliveEntities();
}

static bool worlds_equal(const BRepWorld& a, const BRepWorld& b)
{
    auto aa = a.AliveEntities();
    auto ba = b.AliveEntities();
    std::set<uint32_t> sa(aa.begin(), aa.end());
    std::set<uint32_t> sb(ba.begin(), ba.end());
    if (sa != sb) return false;
    for (uint32_t id : sa) {
        auto pa = get_params(a, id);
        auto pb = get_params(b, id);
        if (pa != pb) return false;
    }
    return true;
}

static EntityEntry make_entity(uint32_t pid, Type type, const std::vector<double>& params)
{
    EntityEntry e;
    e.persistent_id = pid;
    e.type = type;
    e.min_pt[0] = e.min_pt[1] = e.min_pt[2] = 0.0;
    e.max_pt[0] = e.max_pt[1] = e.max_pt[2] = 1.0;
    e.params = params;
    return e;
}

// ============================================================
// ParamHunk tests
// ============================================================

TEST_CASE("ParamHunk: basic change in the middle of a large array", "[hunk]")
{
    std::vector<double> old_p(1000);
    std::iota(old_p.begin(), old_p.end(), 0.0);

    auto new_p  = old_p;
    new_p[100]  = 999.0;
    new_p[101]  = 888.0;

    std::vector<ParamHunk> fwd, rev;
    VersionTree::ComputeParamHunks(old_p, new_p, fwd, rev);

    REQUIRE(fwd.size()     == 1);
    REQUIRE(fwd[0].offset  == 100);
    REQUIRE(fwd[0].data[0] == 999.0);
    REQUIRE(fwd[0].data[1] == 888.0);

    auto rebuilt  = VersionTree::ApplyParamHunks(old_p, fwd, static_cast<uint32_t>(new_p.size()));
    auto reverted = VersionTree::ApplyParamHunks(new_p, rev, static_cast<uint32_t>(old_p.size()));
    REQUIRE(rebuilt  == new_p);
    REQUIRE(reverted == old_p);
}

TEST_CASE("ParamHunk: array grows", "[hunk]")
{
    std::vector<double> old_p = { 1, 2, 3, 4, 5 };
    std::vector<double> new_p = { 1, 2, 99, 4, 5, 6, 7, 8 };

    std::vector<ParamHunk> fwd, rev;
    VersionTree::ComputeParamHunks(old_p, new_p, fwd, rev);

    auto rebuilt  = VersionTree::ApplyParamHunks(old_p, fwd, static_cast<uint32_t>(new_p.size()));
    auto reverted = VersionTree::ApplyParamHunks(new_p, rev, static_cast<uint32_t>(old_p.size()));
    REQUIRE(rebuilt  == new_p);
    REQUIRE(reverted == old_p);
}

TEST_CASE("ParamHunk: no change produces empty hunks", "[hunk]")
{
    std::vector<double> p = { 1, 2, 3, 4, 5 };
    std::vector<ParamHunk> fwd, rev;
    VersionTree::ComputeParamHunks(p, p, fwd, rev);
    REQUIRE(fwd.empty());
    REQUIRE(rev.empty());
}

TEST_CASE("ParamHunk: nearby changes are coalesced", "[hunk]")
{
    std::vector<double> old_p(100, 0.0);
    auto new_p  = old_p;
    new_p[10]   = 1.0;
    new_p[13]   = 2.0;  // gap = 2, within COALESCE_GAP = 4

    std::vector<ParamHunk> fwd, rev;
    VersionTree::ComputeParamHunks(old_p, new_p, fwd, rev);

    REQUIRE(fwd.size()       == 1);
    REQUIRE(fwd[0].offset    == 10);
    REQUIRE(fwd[0].data.size() == 4);

    auto rebuilt = VersionTree::ApplyParamHunks(old_p, fwd, static_cast<uint32_t>(new_p.size()));
    REQUIRE(rebuilt == new_p);
}

TEST_CASE("ParamHunk: distant changes stay separate", "[hunk]")
{
    std::vector<double> old_p(100, 0.0);
    auto new_p = old_p;
    new_p[5]   = 1.0;
    new_p[50]  = 2.0;
    new_p[90]  = 3.0;

    std::vector<ParamHunk> fwd, rev;
    VersionTree::ComputeParamHunks(old_p, new_p, fwd, rev);

    REQUIRE(fwd.size() == 3);

    auto rebuilt  = VersionTree::ApplyParamHunks(old_p, fwd, static_cast<uint32_t>(new_p.size()));
    auto reverted = VersionTree::ApplyParamHunks(new_p, rev, static_cast<uint32_t>(old_p.size()));
    REQUIRE(rebuilt  == new_p);
    REQUIRE(reverted == old_p);
}

TEST_CASE("ParamHunk: large entity with tiny change stores very few doubles", "[hunk]")
{
    std::vector<double> old_p(5000);
    std::iota(old_p.begin(), old_p.end(), 0.0);

    auto new_p   = old_p;
    new_p[500]   = -1.0;
    new_p[501]   = -2.0;
    new_p[502]   = -3.0;
    new_p[2000]  = -4.0;
    new_p[4000]  = -5.0;
    new_p[4001]  = -6.0;

    auto old_e = make_entity(10, Type::Compound, old_p);
    auto new_e = make_entity(50, Type::Compound, new_p);
    auto mod   = VersionTree::BuildModifiedEntry(10, 50, old_e, new_e);

    size_t fwd_doubles = 0;
    for (const auto& h : mod.forward_hunks) { fwd_doubles += h.data.size(); }
    REQUIRE(fwd_doubles < 50);

    auto rebuilt  = VersionTree::ApplyParamHunks(old_p, mod.forward_hunks, mod.new_param_count);
    auto reverted = VersionTree::ApplyParamHunks(new_p, mod.reverse_hunks, mod.old_param_count);
    REQUIRE(rebuilt  == new_p);
    REQUIRE(reverted == old_p);
}

// ============================================================
// ComputeDiff tests
// ============================================================

TEST_CASE("ComputeDiff: insert at front", "[diff]")
{
    auto old_w = make_world_ab();
    auto new_w = make_world_cab();
    auto diff  = VersionTree::ComputeDiff(old_w, new_w);

    REQUIRE(diff.added.size()          == 1);
    REQUIRE(diff.added[0].PersistentId() == 30);
    REQUIRE(diff.removed.empty());
    REQUIRE(diff.modified.empty());

    auto fwd = VersionTree::ApplyForward(old_w, diff);
    REQUIRE(get_alive(fwd).size() == 3);

    auto rev = VersionTree::ApplyReverse(new_w, diff);
    REQUIRE(get_alive(rev).size() == 2);
    REQUIRE(rev->IsAlive(10));
    REQUIRE(rev->IsAlive(20));
}

TEST_CASE("ComputeDiff: empty diff when pools are identical", "[diff]")
{
    auto w = make_world_ab();
    auto diff = VersionTree::ComputeDiff(w, w);
    REQUIRE(diff.IsEmpty());
    auto fwd = VersionTree::ApplyForward(w, diff);
    REQUIRE(worlds_equal(*fwd, w));
}

TEST_CASE("ComputeDiff: reorder only — no entities changed", "[diff]")
{
    auto old_w = make_world_ab();

    BRepWorld new_w;
    add_entity(new_w, 20, Type::Compound, { 4, 5 });
    add_entity(new_w, 10, Type::Compound, { 1, 2, 3 });

    auto diff = VersionTree::ComputeDiff(old_w, new_w);
    REQUIRE(diff.IsEmpty());
    REQUIRE(diff.new_order[0] == 20);
    REQUIRE(diff.new_order[1] == 10);

    auto fwd = VersionTree::ApplyForward(old_w, diff);
    auto alive = get_alive(fwd);
    REQUIRE(alive[0] == 20);
    REQUIRE(alive[1] == 10);
}

TEST_CASE("ComputeDiff: complex op — add + remove + modify", "[diff]")
{
    BRepWorld old_w;
    add_entity(old_w, 1, Type::Compound, { 0,0,0 });
    add_entity(old_w, 2, Type::Compound, { 1,1,1,1,1 });
    add_entity(old_w, 3, Type::Compound, { 2,2,2,2 });
    add_entity(old_w, 4, Type::Compound, { 3,3,3,3 });

    BRepWorld new_w;
    add_entity(new_w, 1, Type::Compound, { 0,0,0 });
    add_entity(new_w, 3, Type::Compound, { 9,9,9,9 });
    add_entity(new_w, 5, Type::Compound, { 7,7,7,7,7,7 });

    auto diff = VersionTree::ComputeDiff(old_w, new_w);
    REQUIRE(diff.removed.size()  == 2);  // edge(2), face(4)
    REQUIRE(diff.added.size()    == 1);  // face(5)
    REQUIRE(diff.modified.size() == 1);  // face(3)
    REQUIRE(diff.modified[0].old_persistent_id == 3);

    auto fwd = VersionTree::ApplyForward(old_w, diff);
    REQUIRE(get_alive(fwd).size() == 3);
    REQUIRE(get_params(fwd, 3)[0] == 9.0);

    auto rev = VersionTree::ApplyReverse(*fwd, diff);
    REQUIRE(get_alive(rev).size() == 4);
    REQUIRE(get_params(rev, 3)[0] == 2.0);
}

// ============================================================
// Pre-built diff with pid change
// ============================================================

TEST_CASE("Pre-built diff: pid changes after CalcUID re-assigns uids", "[diff][pidmap]")
{
    BRepWorld old_w;
    add_entity(old_w, 100, Type::Compound, { 1, 2, 3 });
    add_entity(old_w, 200, Type::Compound, { 4, 5 });

    BRepWorld new_w;
    add_entity(new_w, 300, Type::Compound, { 10, 20, 30 });
    add_entity(new_w, 400, Type::Compound, { 4, 5 });

    PoolDiff diff;
    diff.old_order = { 100, 200 };
    diff.new_order = { 300, 400 };
    diff.modified.push_back(VersionTree::BuildModifiedEntry(
        100, 300,
        VersionTree::ExtractEntity(old_w, 100),
        VersionTree::ExtractEntity(new_w, 300)));
    diff.modified.push_back(VersionTree::BuildModifiedEntry(
        200, 400,
        VersionTree::ExtractEntity(old_w, 200),
        VersionTree::ExtractEntity(new_w, 400)));

    auto fwd = VersionTree::ApplyForward(old_w, diff);
    REQUIRE(fwd->IsAlive(300));
    REQUIRE(get_params(fwd, 300)[0] == 10.0);

    auto rev = VersionTree::ApplyReverse(new_w, diff);
    REQUIRE(rev->IsAlive(100));
    REQUIRE(get_params(rev, 100)[0] == 1.0);
}

// ============================================================
// PidMapping tests
// ============================================================

TEST_CASE("PidMapping: simple modify — all pids change", "[pidmap]")
{
    BRepWorld old_w;
    add_entity(old_w, 10, Type::Compound, { 1, 2, 3 });
    add_entity(old_w, 20, Type::Compound, { 4, 5 });

    BRepWorld new_w;
    add_entity(new_w, 110, Type::Compound, { 10, 20, 30 });
    add_entity(new_w, 120, Type::Compound, { 4, 5 });

    VersionTree::PidMapping pid_map;
    pid_map[10] = { 110 };
    pid_map[20] = { 120 };

    auto diff = VersionTree::BuildDiffFromPidMapping(old_w, new_w, pid_map);
    REQUIRE(diff.modified.size() == 2);
    REQUIRE(diff.added.empty());
    REQUIRE(diff.removed.empty());

    auto fwd = VersionTree::ApplyForward(old_w, diff);
    REQUIRE(fwd->IsAlive(110));

    auto rev = VersionTree::ApplyReverse(*fwd, diff);
    REQUIRE(rev->IsAlive(10));
}

TEST_CASE("PidMapping: entity deleted", "[pidmap]")
{
    BRepWorld old_w;
    add_entity(old_w, 10, Type::Compound, { 1, 2 });
    add_entity(old_w, 20, Type::Compound, { 3, 4 });
    add_entity(old_w, 30, Type::Compound, { 5, 6 });

    BRepWorld new_w;
    add_entity(new_w, 110, Type::Compound, { 1, 2 });
    add_entity(new_w, 130, Type::Compound, { 5, 6 });

    VersionTree::PidMapping pid_map;
    pid_map[10] = { 110 };
    pid_map[20] = {};       // deleted
    pid_map[30] = { 130 };

    auto diff = VersionTree::BuildDiffFromPidMapping(old_w, new_w, pid_map);
    REQUIRE(diff.removed.size()              == 1);
    REQUIRE(diff.removed[0].PersistentId()   == 20);

    auto fwd = VersionTree::ApplyForward(old_w, diff);
    REQUIRE(get_alive(fwd).size() == 2);

    auto rev = VersionTree::ApplyReverse(*fwd, diff);
    REQUIRE(get_alive(rev).size() == 3);
    REQUIRE(rev->IsAlive(20));
}

TEST_CASE("PidMapping: entity split into two", "[pidmap]")
{
    BRepWorld old_w;
    add_entity(old_w, 10, Type::Compound, { 1, 2, 3, 4 });

    BRepWorld new_w;
    add_entity(new_w, 50, Type::Compound, { 10, 20, 30 });
    add_entity(new_w, 51, Type::Compound, { 40, 50, 60 });

    VersionTree::PidMapping pid_map;
    pid_map[10] = { 50, 51 };

    auto diff = VersionTree::BuildDiffFromPidMapping(old_w, new_w, pid_map);
    REQUIRE(diff.modified.size()         == 1);
    REQUIRE(diff.added.size()            == 1);
    REQUIRE(diff.added[0].PersistentId() == 51);

    auto fwd = VersionTree::ApplyForward(old_w, diff);
    REQUIRE(get_alive(fwd).size() == 2);

    auto rev = VersionTree::ApplyReverse(*fwd, diff);
    REQUIRE(get_alive(rev).size() == 1);
    REQUIRE(rev->IsAlive(10));
}

TEST_CASE("PidMapping: unmapped new entity detected as added", "[pidmap]")
{
    BRepWorld old_w;
    add_entity(old_w, 10, Type::Compound, { 1, 2 });

    BRepWorld new_w;
    add_entity(new_w, 110, Type::Compound, { 1, 2 });
    add_entity(new_w, 111, Type::Compound, { 5, 6, 7 });

    VersionTree::PidMapping pid_map;
    pid_map[10] = { 110 };

    auto diff = VersionTree::BuildDiffFromPidMapping(old_w, new_w, pid_map);
    REQUIRE(diff.added.size()            == 1);
    REQUIRE(diff.added[0].PersistentId() == 111);

    auto fwd = VersionTree::ApplyForward(old_w, diff);
    REQUIRE(get_alive(fwd).size() == 2);
    REQUIRE(fwd->IsAlive(111));

    auto rev = VersionTree::ApplyReverse(*fwd, diff);
    REQUIRE(get_alive(rev).size() == 1);
    REQUIRE(rev->IsAlive(10));
}

TEST_CASE("PidMapping: unlisted entity falls back to pid match", "[pidmap]")
{
    BRepWorld old_w;
    add_entity(old_w, 10, Type::Compound, { 1, 2, 3 });
    add_entity(old_w, 50, Type::Compound, { 4, 5 });

    BRepWorld new_w;
    add_entity(new_w, 110, Type::Compound, { 10, 20, 30 });
    add_entity(new_w, 50,  Type::Compound, { 40, 50 });

    VersionTree::PidMapping pid_map;
    pid_map[10] = { 110 };

    auto diff = VersionTree::BuildDiffFromPidMapping(old_w, new_w, pid_map);
    REQUIRE(diff.modified.size() == 2);  // face 10->110 + curve 50->50

    bool curve_modified = false;
    for (const auto& m : diff.modified) {
        if (m.old_persistent_id == 50 && m.new_persistent_id == 50) {
            curve_modified = true;
        }
    }
    REQUIRE(curve_modified);
}

TEST_CASE("PidMapping: empty old_pool — initial commit through pid_map", "[pidmap]")
{
    BRepWorld old_w;  // empty

    BRepWorld new_w;
    add_entity(new_w, 110, Type::Compound, { 1, 2 });
    add_entity(new_w, 120, Type::Compound, { 3, 4, 5 });
    add_entity(new_w, 121, Type::Compound, { 6 });

    VersionTree::PidMapping pid_map;
    pid_map[10] = { 110 };
    pid_map[20] = { 120, 121 };

    auto diff = VersionTree::BuildDiffFromPidMapping(old_w, new_w, pid_map);

    REQUIRE(diff.modified.empty());
    REQUIRE(diff.removed.empty());
    REQUIRE(diff.added.size() == 3);

    std::set<uint32_t> added_pids;
    for (const auto& e : diff.added) { added_pids.insert(e.PersistentId()); }
    REQUIRE(added_pids == std::set<uint32_t>{ 110, 120, 121 });

    auto fwd = VersionTree::ApplyForward(old_w, diff);
    REQUIRE(get_alive(fwd).size() == 3);

    auto rev = VersionTree::ApplyReverse(*fwd, diff);
    REQUIRE(get_alive(rev).size() == 0);
}

TEST_CASE("PidMapping: stale old_pid not in old_pool falls back to added", "[pidmap]")
{
    BRepWorld old_w;
    add_entity(old_w, 10, Type::Compound, { 1, 2 });

    BRepWorld new_w;
    add_entity(new_w, 110, Type::Compound, { 1, 2 });
    add_entity(new_w, 210, Type::Compound, { 7, 8 });

    VersionTree::PidMapping pid_map;
    pid_map[10] = { 110 };
    pid_map[99] = { 210 };

    auto diff = VersionTree::BuildDiffFromPidMapping(old_w, new_w, pid_map);

    REQUIRE(diff.modified.size() == 1);
    REQUIRE(diff.modified[0].old_persistent_id == 10);
    REQUIRE(diff.modified[0].new_persistent_id == 110);

    REQUIRE(diff.added.size()            == 1);
    REQUIRE(diff.added[0].PersistentId() == 210);

    REQUIRE(diff.removed.empty());
}

// ============================================================
// VersionTree navigation tests
// ============================================================

TEST_CASE("VersionTree: Commit(pid_map) auto-installs root on first call", "[tree][pidmap]")
{
    VersionTree tree;
    REQUIRE(tree.GetCurrentWorld() == nullptr);
    REQUIRE(tree.GetRootId() == UINT32_MAX);

    auto w0 = make_world_ab();

    VersionTree::PidMapping junk_map;
    junk_map[999] = { 12345 };

    uint32_t root_id = tree.Commit(w0, junk_map, "create box", /*op_type=*/7);

    REQUIRE(tree.GetRootId()    == root_id);
    REQUIRE(tree.GetCurrentId() == root_id);
    REQUIRE(tree.GetNodeCount() == 1);

    auto current = tree.GetCurrentWorld();
    REQUIRE(get_alive(current).size() == 2);
    REQUIRE(current->IsAlive(10));
    REQUIRE(current->IsAlive(20));

    const auto* node = tree.GetNode(root_id);
    REQUIRE(node);
    REQUIRE(node->op_desc == "create box");
    REQUIRE(node->op_type == 7);
}

TEST_CASE("VersionTree: Commit(pid_map) builds diff for non-first call", "[tree][pidmap]")
{
    VersionTree tree;

    BRepWorld w0;
    add_entity(w0, 10, Type::Compound, { 1, 2 });
    tree.Commit(w0, VersionTree::PidMapping{}, "init", 0);

    BRepWorld w1;
    add_entity(w1, 110, Type::Compound, { 1, 2 });
    add_entity(w1, 111, Type::Compound, { 7, 8, 9 });

    VersionTree::PidMapping pid_map;
    pid_map[10] = { 110, 111 };

    uint32_t commit_id = tree.Commit(w1, pid_map, "split face", 1);
    REQUIRE(commit_id != tree.GetRootId());
    REQUIRE(tree.GetNodeCount() == 2);

    auto cur = tree.GetCurrentWorld();
    REQUIRE(get_alive(cur).size() == 2);
    REQUIRE(cur->IsAlive(110));
    REQUIRE(cur->IsAlive(111));

    auto undone = tree.Undo();
    REQUIRE(get_alive(undone).size() == 1);
    REQUIRE(undone->IsAlive(10));

    auto redone = tree.Redo();
    REQUIRE(get_alive(redone).size() == 2);
    REQUIRE(redone->IsAlive(111));
}

TEST_CASE("VersionTree: linear undo / redo", "[tree]")
{
    VersionTree tree;
    auto w0 = make_world_ab();
    tree.Commit(w0, "create box");

    auto w1 = make_world_cab();
    tree.Commit(w1, "fillet");

    BRepWorld w2 = w1;
    w2.Params().Get(20)->data[0] = 99.0;
    tree.Commit(w2, "chamfer");

    REQUIRE(tree.GetNodeCount() == 3);

    auto u1 = tree.Undo();
    REQUIRE(get_alive(u1).size() == 3);
    REQUIRE(get_params(u1, 20)[0] == 4.0);

    auto u0 = tree.Undo();
    REQUIRE(get_alive(u0).size() == 2);
    REQUIRE(u0->IsAlive(10));
    REQUIRE_FALSE(tree.CanUndo());

    auto r1 = tree.Redo();
    REQUIRE(r1->IsAlive(30));

    auto r2 = tree.Redo();
    REQUIRE(get_params(r2, 20)[0] == 99.0);
    REQUIRE_FALSE(tree.CanRedo());
}

TEST_CASE("VersionTree: branching and cross-branch checkout", "[tree]")
{
    VersionTree tree;
    auto w0 = make_world_ab();
    tree.Commit(w0, "init");

    BRepWorld wA;
    add_entity(wA, 50, Type::Compound, { 10, 20 });

    VersionTree::PidMapping mapA;
    mapA[10] = { 50 };
    mapA[20] = {};
    auto dA  = VersionTree::BuildDiffFromPidMapping(w0, wA, mapA);
    uint32_t idA = tree.Commit(wA, std::move(dA), "branch A");

    BRepWorld wB;
    add_entity(wB, 60, Type::Compound, { 30, 40 });

    VersionTree::PidMapping mapB;
    mapB[10] = { 60 };
    mapB[20] = {};
    auto dB  = VersionTree::BuildDiffFromPidMapping(w0, wB, mapB);
    uint32_t idB = tree.Branch(0, wB, std::move(dB), "branch B");

    SECTION("checkout branch A") {
        auto rA = tree.Checkout(idA);
        REQUIRE(rA->IsAlive(50));
        REQUIRE(get_params(rA, 50)[0] == 10.0);
    }

    SECTION("checkout branch B") {
        auto rB = tree.Checkout(idB);
        REQUIRE(rB->IsAlive(60));
        REQUIRE(get_params(rB, 60)[0] == 30.0);
    }

    SECTION("checkout root") {
        auto r0 = tree.Checkout(0);
        REQUIRE(r0->IsAlive(10));
    }

    SECTION("root has 2 children") {
        REQUIRE(tree.GetNode(0)->children.size() == 2);
    }
}

TEST_CASE("VersionTree: Redo selects correct child by index", "[tree]")
{
    VersionTree tree;
    auto ab_w = make_world_ab();
    tree.Commit(ab_w, "root");

    auto cab_w = make_world_cab();
    tree.Commit(cab_w, "branch A");

    BRepWorld wB = make_world_ab();
    wB.Params().Get(10)->data[0] = 77.0;
    tree.Branch(0, wB, VersionTree::ComputeDiff(ab_w, wB), "branch B");

    tree.Checkout(0);

    auto rA = tree.Redo(0);
    REQUIRE(tree.GetCurrentId() == 1);
    REQUIRE(get_alive(rA).size() == 3);

    tree.Checkout(0);

    auto rB = tree.Redo(1);
    REQUIRE(tree.GetCurrentId() == 2);
    REQUIRE(get_params(rB, 10)[0] == 77.0);
}

TEST_CASE("VersionTree: deep chain of 20 commits — undo all then redo all", "[tree]")
{
    VersionTree tree;

    BRepWorld w0;
    add_entity(w0, 1, Type::Compound, std::vector<double>(10, 0.0));
    tree.Commit(w0, "init");

    constexpr int DEPTH = 20;
    for (int i = 1; i <= DEPTH; ++i)
    {
        BRepWorld wi;
        std::vector<double> p(10, 0.0);
        p[0] = static_cast<double>(i);
        add_entity(wi, 1, Type::Compound, p);
        tree.Commit(wi, "step " + std::to_string(i));
    }

    REQUIRE(tree.GetNodeCount() == DEPTH + 1);

    for (int i = 0; i < DEPTH; ++i) { tree.Undo(); }
    REQUIRE(tree.GetCurrentId() == 0);
    REQUIRE(get_params(tree.GetCurrentWorld(), 1)[0] == 0.0);

    for (int i = 0; i < DEPTH; ++i) { tree.Redo(); }
    REQUIRE(get_params(tree.GetCurrentWorld(), 1)[0] == static_cast<double>(DEPTH));
}

// ============================================================
// Query tests
// ============================================================

TEST_CASE("VersionTree: GetPathFromRoot and GetLeaves", "[tree][query]")
{
    VersionTree tree;
    auto ab_w = make_world_ab();
    tree.Commit(ab_w, "root");
    tree.Commit(make_world_cab(), "op1");

    BRepWorld w2 = make_world_cab();
    w2.Params().Get(30)->data[0] = 42.0;
    tree.Commit(w2, "op2");

    BRepWorld w3 = make_world_ab();
    w3.Params().Get(10)->data[0] = 99.0;
    tree.Branch(0, w3, VersionTree::ComputeDiff(ab_w, w3), "op3");

    auto path = tree.GetPathFromRoot(2);
    REQUIRE(path.size() == 3);
    REQUIRE(path[0] == 0);
    REQUIRE(path[1] == 1);
    REQUIRE(path[2] == 2);

    auto leaves = tree.GetLeaves();
    REQUIRE(leaves.size() == 2);
}

TEST_CASE("VersionTree: TraverseAll visits every node", "[tree][query]")
{
    VersionTree tree;
    tree.Commit(make_world_ab(), "root");
    tree.Commit(make_world_cab(), "op1");

    int count = 0;
    tree.TraverseAll([&](const VersionNode&) { ++count; });
    REQUIRE(count == 2);
}

// ============================================================
// Persistence tests
// ============================================================

TEST_CASE("Persistence: SaveToFile / LoadFromFile round-trip", "[persist]")
{
    const std::string fp = "/tmp/vt_brepdb_test.vtbd";

    VersionTree t1;
    auto ab_w = make_world_ab();
    t1.Commit(ab_w, "create box");

    auto cab_w = make_world_cab();
    t1.Commit(cab_w, "fillet");

    BRepWorld w2 = make_world_cab();
    w2.Params().Get(10)->data[0] = 77.0;
    t1.Commit(w2, "chamfer");

    BRepWorld wb = make_world_cab();
    wb.Params().Get(30)->data[0] = 55.0;
    t1.Branch(1, wb, VersionTree::ComputeDiff(cab_w, wb), "branch");

    REQUIRE(t1.SaveToFile(fp));

    VersionTree t2;
    REQUIRE(t2.LoadFromFile(fp));
    REQUIRE(t2.GetNodeCount() == t1.GetNodeCount());

    auto r0 = t2.Checkout(0);
    REQUIRE(get_alive(r0).size() == 2);
    REQUIRE(r0->IsAlive(10));

    auto r2 = t2.Checkout(2);
    REQUIRE(get_params(r2, 10)[0] == 77.0);

    auto rb = t2.Checkout(3);
    REQUIRE(get_params(rb, 30)[0] == 55.0);
}

TEST_CASE("Persistence: StoreToByteArray / LoadFromByteArray (BrepDB meta page path)", "[persist]")
{
    VersionTree t1;
    auto w0 = make_world_ab();
    t1.Commit(w0, "create box");

    auto w1 = make_world_cab();
    t1.Commit(w1, "fillet");

    BRepWorld w2 = w1;
    w2.Params().Get(30)->data[0] = 42.0;
    t1.Commit(w2, "chamfer");

    uint8_t* buf = nullptr;
    uint32_t len = 0;
    t1.StoreToByteArray(&buf, len);
    REQUIRE(buf != nullptr);
    REQUIRE(len > 0);

    VersionTree t2;
    t2.LoadFromByteArray(buf, len, w2);
    delete[] buf;

    REQUIRE(t2.GetNodeCount()  == 3);
    REQUIRE(t2.GetCurrentId()  == t1.GetCurrentId());

    auto u1 = t2.Undo();
    REQUIRE(get_alive(u1).size() == 3);
    REQUIRE(get_params(u1, 30)[0] != 42.0);

    auto u0 = t2.Undo();
    REQUIRE(get_alive(u0).size() == 2);
    REQUIRE(u0->IsAlive(10));

    t2.Redo();
    auto r2 = t2.Redo();
    REQUIRE(get_params(r2, 30)[0] == 42.0);
}

TEST_CASE("Persistence: byte-array with branches", "[persist]")
{
    VersionTree t1;
    auto w0 = make_world_ab();
    t1.Commit(w0, "init");

    BRepWorld wA;
    add_entity(wA, 50, Type::Compound, { 10, 20, 30 });

    VersionTree::PidMapping mapA;
    mapA[10] = { 50 };
    t1.Commit(wA, VersionTree::BuildDiffFromPidMapping(w0, wA, mapA), "branch A");

    BRepWorld wB;
    add_entity(wB, 60, Type::Compound, { 100, 200, 300 });

    VersionTree::PidMapping mapB;
    mapB[10] = { 60 };
    t1.Branch(0, wB, VersionTree::BuildDiffFromPidMapping(w0, wB, mapB), "branch B");

    uint8_t* buf = nullptr;
    uint32_t len = 0;
    t1.StoreToByteArray(&buf, len);

    VersionTree t2;
    t2.LoadFromByteArray(buf, len, wB);
    delete[] buf;

    REQUIRE(t2.GetNodeCount() == 3);

    auto rA = t2.Checkout(1);
    REQUIRE(rA->IsAlive(50));

    auto r0 = t2.Checkout(0);
    REQUIRE(r0->IsAlive(10));

    auto rB = t2.Checkout(2);
    REQUIRE(get_params(rB, 60)[0] == 100.0);
}

// ============================================================
// BrepDB integration flow
// ============================================================

TEST_CASE("Integration: BrepDB open/save/undo/redo cycle", "[integration]")
{
    auto w0 = make_world_ab();

    VersionTree tree;
    tree.Commit(w0, "create box");

    auto w1 = make_world_cab();
    VersionTree::PidMapping map1;
    map1[10] = { 10 };
    map1[20] = { 20 };
    tree.Commit(w1, VersionTree::BuildDiffFromPidMapping(w0, w1, map1), "fillet");

    uint8_t* save_buf = nullptr;
    uint32_t save_len = 0;
    tree.StoreToByteArray(&save_buf, save_len);

    VersionTree loaded;
    loaded.LoadFromByteArray(save_buf, save_len, w1);
    delete[] save_buf;

    REQUIRE(loaded.GetNodeCount() == 2);
    REQUIRE(worlds_equal(*loaded.GetCurrentWorld(), w1));

    auto undo_w = loaded.Undo();
    REQUIRE(get_alive(undo_w).size() == 2);
    REQUIRE(undo_w->IsAlive(10));

    auto redo_w = loaded.Redo();
    REQUIRE(worlds_equal(*redo_w, w1));
}
