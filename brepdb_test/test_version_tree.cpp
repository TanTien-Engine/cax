#include <catch2/catch_test_macros.hpp>

#include "VersionTree.h"

#include <numeric>
#include <string>

using namespace brepdb;

// ============================================================
// Helpers
// ============================================================

static GeomHeader make_header(Type type, uint32_t pid,
                               uint32_t offset, uint32_t count)
{
    GeomHeader h{};
    h.type          = type;
    h.persistent_id = pid;
    h.param_offset  = offset;
    h.param_count   = count;
    h.min_pt[0] = h.min_pt[1] = h.min_pt[2] = 0.0;
    h.max_pt[0] = h.max_pt[1] = h.max_pt[2] = 1.0;
    return h;
}

// pool: [face_a(pid=10, 3 doubles), face_b(pid=20, 2 doubles)]
static GeometryPool make_pool_ab()
{
    GeometryPool pool;
    pool.headers.push_back(make_header(Type::Face, 10, 0, 3));
    pool.headers.push_back(make_header(Type::Face, 20, 3, 2));
    pool.data_pool = { 1, 2, 3, 4, 5 };
    return pool;
}

// pool: [face_c(pid=30), face_a(pid=10), face_b(pid=20)]
static GeometryPool make_pool_cab()
{
    GeometryPool pool;
    pool.headers.push_back(make_header(Type::Face, 30, 0, 4));
    pool.headers.push_back(make_header(Type::Face, 10, 4, 3));
    pool.headers.push_back(make_header(Type::Face, 20, 7, 2));
    pool.data_pool = { 6, 7, 8, 9, 1, 2, 3, 4, 5 };
    return pool;
}

static bool pools_equal(const GeometryPool& a, const GeometryPool& b)
{
    if (a.headers.size() != b.headers.size()) { return false; }
    for (size_t i = 0; i < a.headers.size(); ++i)
    {
        if (a.headers[i].persistent_id != b.headers[i].persistent_id) { return false; }
        if (a.headers[i].param_count   != b.headers[i].param_count)   { return false; }
    }
    return a.data_pool == b.data_pool;
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

    auto old_e = EntityEntry{ make_header(Type::Face, 10, 0, 5000), old_p };
    auto new_e = EntityEntry{ make_header(Type::Face, 50, 0, 5000), new_p };
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
    auto old_pool = make_pool_ab();
    auto new_pool = make_pool_cab();
    auto diff     = VersionTree::ComputeDiff(old_pool, new_pool);

    REQUIRE(diff.added.size()          == 1);
    REQUIRE(diff.added[0].PersistentId() == 30);
    REQUIRE(diff.removed.empty());
    REQUIRE(diff.modified.empty());

    auto fwd = VersionTree::ApplyForward(old_pool, diff);
    REQUIRE(fwd.headers.size()            == 3);
    REQUIRE(fwd.headers[0].persistent_id == 30);

    auto rev = VersionTree::ApplyReverse(new_pool, diff);
    REQUIRE(rev.headers.size()            == 2);
    REQUIRE(rev.headers[0].persistent_id == 10);
}

TEST_CASE("ComputeDiff: empty diff when pools are identical", "[diff]")
{
    auto pool = make_pool_ab();
    auto diff = VersionTree::ComputeDiff(pool, pool);
    REQUIRE(diff.IsEmpty());
    REQUIRE(pools_equal(VersionTree::ApplyForward(pool, diff), pool));
}

TEST_CASE("ComputeDiff: reorder only — no entities changed", "[diff]")
{
    auto old_pool = make_pool_ab();

    GeometryPool new_pool;
    new_pool.headers.push_back(make_header(Type::Face, 20, 0, 2));
    new_pool.headers.push_back(make_header(Type::Face, 10, 2, 3));
    new_pool.data_pool = { 4, 5, 1, 2, 3 };

    auto diff = VersionTree::ComputeDiff(old_pool, new_pool);
    REQUIRE(diff.IsEmpty());
    REQUIRE(diff.new_order[0] == 20);
    REQUIRE(diff.new_order[1] == 10);

    auto fwd = VersionTree::ApplyForward(old_pool, diff);
    REQUIRE(fwd.headers[0].persistent_id == 20);
    REQUIRE(fwd.headers[1].persistent_id == 10);
}

TEST_CASE("ComputeDiff: complex op — add + remove + modify", "[diff]")
{
    GeometryPool old_p;
    old_p.headers.push_back(make_header(Type::Vertex, 1, 0,  3));
    old_p.headers.push_back(make_header(Type::Edge,   2, 3,  5));
    old_p.headers.push_back(make_header(Type::Face,   3, 8,  4));
    old_p.headers.push_back(make_header(Type::Face,   4, 12, 4));
    old_p.data_pool = { 0,0,0, 1,1,1,1,1, 2,2,2,2, 3,3,3,3 };

    GeometryPool new_p;
    new_p.headers.push_back(make_header(Type::Vertex, 1, 0, 3));
    new_p.headers.push_back(make_header(Type::Face,   3, 3, 4));
    new_p.headers.push_back(make_header(Type::Face,   5, 7, 6));
    new_p.data_pool = { 0,0,0, 9,9,9,9, 7,7,7,7,7,7 };

    auto diff = VersionTree::ComputeDiff(old_p, new_p);
    REQUIRE(diff.removed.size()  == 2);  // edge(2), face(4)
    REQUIRE(diff.added.size()    == 1);  // face(5)
    REQUIRE(diff.modified.size() == 1);  // face(3)
    REQUIRE(diff.modified[0].old_persistent_id == 3);

    auto fwd = VersionTree::ApplyForward(old_p, diff);
    REQUIRE(fwd.headers.size()            == 3);
    REQUIRE(fwd.headers[1].persistent_id == 3);
    REQUIRE(fwd.data_pool[fwd.headers[1].param_offset] == 9.0);

    auto rev = VersionTree::ApplyReverse(fwd, diff);
    REQUIRE(rev.headers.size() == 4);
    REQUIRE(rev.data_pool[rev.headers[2].param_offset] == 2.0);
}

// ============================================================
// Pre-built diff with pid change
// ============================================================

TEST_CASE("Pre-built diff: pid changes after CalcUID re-assigns uids", "[diff][pidmap]")
{
    GeometryPool old_p;
    old_p.headers.push_back(make_header(Type::Face, 100, 0, 3));
    old_p.headers.push_back(make_header(Type::Face, 200, 3, 2));
    old_p.data_pool = { 1, 2, 3, 4, 5 };

    GeometryPool new_p;
    new_p.headers.push_back(make_header(Type::Face, 300, 0, 3));
    new_p.headers.push_back(make_header(Type::Face, 400, 3, 2));
    new_p.data_pool = { 10, 20, 30, 4, 5 };

    PoolDiff diff;
    diff.old_order = { 100, 200 };
    diff.new_order = { 300, 400 };
    diff.modified.push_back(VersionTree::BuildModifiedEntry(
        100, 300,
        VersionTree::ExtractEntity(old_p, 0),
        VersionTree::ExtractEntity(new_p, 0)));
    diff.modified.push_back(VersionTree::BuildModifiedEntry(
        200, 400,
        VersionTree::ExtractEntity(old_p, 1),
        VersionTree::ExtractEntity(new_p, 1)));

    auto fwd = VersionTree::ApplyForward(old_p, diff);
    REQUIRE(fwd.headers[0].persistent_id == 300);
    REQUIRE(fwd.data_pool[fwd.headers[0].param_offset] == 10.0);

    auto rev = VersionTree::ApplyReverse(new_p, diff);
    REQUIRE(rev.headers[0].persistent_id == 100);
    REQUIRE(rev.data_pool[rev.headers[0].param_offset] == 1.0);
}

// ============================================================
// PidMapping tests
// ============================================================

TEST_CASE("PidMapping: simple modify — all pids change", "[pidmap]")
{
    GeometryPool old_p;
    old_p.headers.push_back(make_header(Type::Face, 10, 0, 3));
    old_p.headers.push_back(make_header(Type::Face, 20, 3, 2));
    old_p.data_pool = { 1, 2, 3, 4, 5 };

    GeometryPool new_p;
    new_p.headers.push_back(make_header(Type::Face, 110, 0, 3));
    new_p.headers.push_back(make_header(Type::Face, 120, 3, 2));
    new_p.data_pool = { 10, 20, 30, 4, 5 };

    VersionTree::PidMapping pid_map;
    pid_map[10] = { 110 };
    pid_map[20] = { 120 };

    auto diff = VersionTree::BuildDiffFromPidMapping(old_p, new_p, pid_map);
    REQUIRE(diff.modified.size() == 2);
    REQUIRE(diff.added.empty());
    REQUIRE(diff.removed.empty());

    auto fwd = VersionTree::ApplyForward(old_p, diff);
    REQUIRE(fwd.headers[0].persistent_id == 110);

    auto rev = VersionTree::ApplyReverse(fwd, diff);
    REQUIRE(rev.headers[0].persistent_id == 10);
}

TEST_CASE("PidMapping: entity deleted", "[pidmap]")
{
    GeometryPool old_p;
    old_p.headers.push_back(make_header(Type::Face, 10, 0, 2));
    old_p.headers.push_back(make_header(Type::Face, 20, 2, 2));
    old_p.headers.push_back(make_header(Type::Face, 30, 4, 2));
    old_p.data_pool = { 1, 2, 3, 4, 5, 6 };

    GeometryPool new_p;
    new_p.headers.push_back(make_header(Type::Face, 110, 0, 2));
    new_p.headers.push_back(make_header(Type::Face, 130, 2, 2));
    new_p.data_pool = { 1, 2, 5, 6 };

    VersionTree::PidMapping pid_map;
    pid_map[10] = { 110 };
    pid_map[20] = {};       // deleted
    pid_map[30] = { 130 };

    auto diff = VersionTree::BuildDiffFromPidMapping(old_p, new_p, pid_map);
    REQUIRE(diff.removed.size()              == 1);
    REQUIRE(diff.removed[0].PersistentId()   == 20);

    auto fwd = VersionTree::ApplyForward(old_p, diff);
    REQUIRE(fwd.headers.size() == 2);

    auto rev = VersionTree::ApplyReverse(fwd, diff);
    REQUIRE(rev.headers.size()            == 3);
    REQUIRE(rev.headers[1].persistent_id == 20);
}

TEST_CASE("PidMapping: entity split into two", "[pidmap]")
{
    GeometryPool old_p;
    old_p.headers.push_back(make_header(Type::Face, 10, 0, 4));
    old_p.data_pool = { 1, 2, 3, 4 };

    GeometryPool new_p;
    new_p.headers.push_back(make_header(Type::Face, 50, 0, 3));
    new_p.headers.push_back(make_header(Type::Face, 51, 3, 3));
    new_p.data_pool = { 10, 20, 30, 40, 50, 60 };

    VersionTree::PidMapping pid_map;
    pid_map[10] = { 50, 51 };  // 50 = modified, 51 = split result (added)

    auto diff = VersionTree::BuildDiffFromPidMapping(old_p, new_p, pid_map);
    REQUIRE(diff.modified.size()         == 1);
    REQUIRE(diff.added.size()            == 1);
    REQUIRE(diff.added[0].PersistentId() == 51);

    auto fwd = VersionTree::ApplyForward(old_p, diff);
    REQUIRE(fwd.headers.size() == 2);

    auto rev = VersionTree::ApplyReverse(fwd, diff);
    REQUIRE(rev.headers.size()            == 1);
    REQUIRE(rev.headers[0].persistent_id == 10);
}

TEST_CASE("PidMapping: unmapped new entity detected as added", "[pidmap]")
{
    GeometryPool old_p;
    old_p.headers.push_back(make_header(Type::Face, 10, 0, 2));
    old_p.data_pool = { 1, 2 };

    GeometryPool new_p;
    new_p.headers.push_back(make_header(Type::Face, 110, 0, 2));
    new_p.headers.push_back(make_header(Type::Face, 111, 2, 3));
    new_p.data_pool = { 1, 2, 5, 6, 7 };

    VersionTree::PidMapping pid_map;
    pid_map[10] = { 110 };
    // 111 not in pid_map -> should be auto-detected as ADDED

    auto diff = VersionTree::BuildDiffFromPidMapping(old_p, new_p, pid_map);
    REQUIRE(diff.added.size()            == 1);
    REQUIRE(diff.added[0].PersistentId() == 111);

    auto fwd = VersionTree::ApplyForward(old_p, diff);
    REQUIRE(fwd.headers.size()            == 2);
    REQUIRE(fwd.headers[1].persistent_id == 111);

    auto rev = VersionTree::ApplyReverse(fwd, diff);
    REQUIRE(rev.headers.size()            == 1);
    REQUIRE(rev.headers[0].persistent_id == 10);
}

TEST_CASE("PidMapping: unlisted entity falls back to pid match", "[pidmap]")
{
    // Face-type BRepHistory does not track curves.
    // curve(pid=50) changes data but is not in pid_map -> fallback comparison.
    GeometryPool old_p;
    old_p.headers.push_back(make_header(Type::Face, 10, 0, 3));
    old_p.headers.push_back(make_header(Type::Line, 50, 3, 2));
    old_p.data_pool = { 1, 2, 3, 4, 5 };

    GeometryPool new_p;
    new_p.headers.push_back(make_header(Type::Face, 110, 0, 3));
    new_p.headers.push_back(make_header(Type::Line, 50,  3, 2));
    new_p.data_pool = { 10, 20, 30, 40, 50 };

    VersionTree::PidMapping pid_map;
    pid_map[10] = { 110 };

    auto diff = VersionTree::BuildDiffFromPidMapping(old_p, new_p, pid_map);
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
    // Regression: when old_pool is empty (or old_pid is missing), the primary
    // new_pid was being marked accounted but never pushed into diff.added,
    // causing entities to silently disappear.
    GeometryPool old_p;  // empty

    GeometryPool new_p;
    new_p.headers.push_back(make_header(Type::Face, 110, 0, 2));
    new_p.headers.push_back(make_header(Type::Face, 120, 2, 3));
    new_p.headers.push_back(make_header(Type::Face, 121, 5, 1));
    new_p.data_pool = { 1, 2, 3, 4, 5, 6 };

    VersionTree::PidMapping pid_map;
    pid_map[10] = { 110 };       // old_pid not in (empty) old_pool
    pid_map[20] = { 120, 121 };  // split-style entry, also rooted at missing old_pid

    auto diff = VersionTree::BuildDiffFromPidMapping(old_p, new_p, pid_map);

    REQUIRE(diff.modified.empty());
    REQUIRE(diff.removed.empty());
    REQUIRE(diff.added.size() == 3);

    std::set<uint32_t> added_pids;
    for (const auto& e : diff.added) { added_pids.insert(e.PersistentId()); }
    REQUIRE(added_pids == std::set<uint32_t>{ 110, 120, 121 });

    auto fwd = VersionTree::ApplyForward(old_p, diff);
    REQUIRE(fwd.headers.size() == 3);

    auto rev = VersionTree::ApplyReverse(fwd, diff);
    REQUIRE(rev.headers.empty());
}

TEST_CASE("PidMapping: stale old_pid not in old_pool falls back to added", "[pidmap]")
{
    // pid_map references an old_pid that no longer exists in old_pool —
    // the primary new_pid should still surface as ADDED rather than vanishing.
    GeometryPool old_p;
    old_p.headers.push_back(make_header(Type::Face, 10, 0, 2));
    old_p.data_pool = { 1, 2 };

    GeometryPool new_p;
    new_p.headers.push_back(make_header(Type::Face, 110, 0, 2));
    new_p.headers.push_back(make_header(Type::Face, 210, 2, 2));
    new_p.data_pool = { 1, 2, 7, 8 };

    VersionTree::PidMapping pid_map;
    pid_map[10] = { 110 };
    pid_map[99] = { 210 };  // 99 is not in old_pool

    auto diff = VersionTree::BuildDiffFromPidMapping(old_p, new_p, pid_map);

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

TEST_CASE("VersionTree: linear undo / redo", "[tree]")
{
    VersionTree tree;
    auto p0 = make_pool_ab();
    tree.Init(p0, "create box");

    auto p1 = make_pool_cab();
    tree.Commit(p1, "fillet");

    GeometryPool p2 = p1;
    p2.data_pool[p2.headers[2].param_offset] = 99.0;
    tree.Commit(p2, "chamfer");

    REQUIRE(tree.GetNodeCount() == 3);

    auto u1 = tree.Undo();
    REQUIRE(u1->headers.size()                    == 3);
    REQUIRE(u1->data_pool[u1->headers[2].param_offset] == 4.0);  // original value

    auto u0 = tree.Undo();
    REQUIRE(u0->headers.size()            == 2);
    REQUIRE(u0->headers[0].persistent_id == 10);
    REQUIRE_FALSE(tree.CanUndo());

    auto r1 = tree.Redo();
    REQUIRE(r1->headers[0].persistent_id == 30);

    auto r2 = tree.Redo();
    REQUIRE(r2->data_pool[r2->headers[2].param_offset] == 99.0);
    REQUIRE_FALSE(tree.CanRedo());
}

TEST_CASE("VersionTree: branching and cross-branch checkout", "[tree]")
{
    VersionTree tree;
    auto p0 = make_pool_ab();
    tree.Init(p0, "init");

    // Branch A: pid 10->50
    GeometryPool pA;
    pA.headers.push_back(make_header(Type::Face, 50, 0, 2));
    pA.data_pool = { 10, 20 };

    VersionTree::PidMapping mapA;
    mapA[10] = { 50 };
    mapA[20] = {};  // deleted
    auto dA  = VersionTree::BuildDiffFromPidMapping(p0, pA, mapA);
    uint32_t idA = tree.Commit(pA, std::move(dA), "branch A");

    // Branch B from root: pid 10->60
    GeometryPool pB;
    pB.headers.push_back(make_header(Type::Face, 60, 0, 2));
    pB.data_pool = { 30, 40 };

    VersionTree::PidMapping mapB;
    mapB[10] = { 60 };
    mapB[20] = {};
    auto dB  = VersionTree::BuildDiffFromPidMapping(p0, pB, mapB);
    uint32_t idB = tree.Branch(0, pB, std::move(dB), "branch B");

    SECTION("checkout branch A") {
        auto rA = tree.Checkout(idA);
        REQUIRE(rA->headers[0].persistent_id == 50);
        REQUIRE(rA->data_pool[0] == 10.0);
    }

    SECTION("checkout branch B") {
        auto rB = tree.Checkout(idB);
        REQUIRE(rB->headers[0].persistent_id == 60);
        REQUIRE(rB->data_pool[0] == 30.0);
    }

    SECTION("checkout root") {
        auto r0 = tree.Checkout(0);
        REQUIRE(r0->headers[0].persistent_id == 10);
    }

    SECTION("root has 2 children") {
        REQUIRE(tree.GetNode(0)->children.size() == 2);
    }
}

TEST_CASE("VersionTree: Redo selects correct child by index", "[tree]")
{
    VersionTree tree;
    tree.Init(make_pool_ab(), "root");

    tree.Commit(make_pool_cab(), "branch A");

    GeometryPool pB = make_pool_ab();
    pB.data_pool[0] = 77.0;
    tree.Branch(0, pB, VersionTree::ComputeDiff(make_pool_ab(), pB), "branch B");

    tree.Checkout(0);

    auto rA = tree.Redo(0);
    REQUIRE(tree.GetCurrentId() == 1);
    REQUIRE(rA->headers.size()   == 3);

    tree.Checkout(0);

    auto rB = tree.Redo(1);
    REQUIRE(tree.GetCurrentId() == 2);
    REQUIRE(rB->data_pool[0]     == 77.0);
}

TEST_CASE("VersionTree: deep chain of 20 commits — undo all then redo all", "[tree]")
{
    VersionTree tree;

    GeometryPool p;
    p.headers.push_back(make_header(Type::Face, 1, 0, 10));
    p.data_pool.resize(10, 0.0);
    tree.Init(p, "init");

    constexpr int DEPTH = 20;
    for (int i = 1; i <= DEPTH; ++i)
    {
        GeometryPool np;
        np.headers.push_back(make_header(Type::Face, 1, 0, 10));
        np.data_pool.resize(10, 0.0);
        np.data_pool[0] = static_cast<double>(i);
        tree.Commit(np, "step " + std::to_string(i));
    }

    REQUIRE(tree.GetNodeCount() == DEPTH + 1);

    for (int i = 0; i < DEPTH; ++i) { tree.Undo(); }
    REQUIRE(tree.GetCurrentId()             == 0);
    REQUIRE(tree.GetCurrentPool()->data_pool[0] == 0.0);

    for (int i = 0; i < DEPTH; ++i) { tree.Redo(); }
    REQUIRE(tree.GetCurrentPool()->data_pool[0] == static_cast<double>(DEPTH));
}

// ============================================================
// Query tests
// ============================================================

TEST_CASE("VersionTree: GetPathFromRoot and GetLeaves", "[tree][query]")
{
    VersionTree tree;
    tree.Init(make_pool_ab(), "root");
    tree.Commit(make_pool_cab(), "op1");

    GeometryPool p2 = make_pool_cab();
    p2.data_pool[0] = 42.0;
    tree.Commit(p2, "op2");

    GeometryPool p3 = make_pool_ab();
    p3.data_pool[0] = 99.0;
    tree.Branch(0, p3, VersionTree::ComputeDiff(make_pool_ab(), p3), "op3");

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
    tree.Init(make_pool_ab(), "root");
    tree.Commit(make_pool_cab(), "op1");

    int count = 0;
    tree.TraverseAll([&](const VersionNode&) { ++count; });
    REQUIRE(count == 2);
}

// ============================================================
// Persistence tests
// ============================================================

TEST_CASE("Persistence: SaveToFile / LoadFromFile round-trip", "[persist]")
{
    const std::string fp = "/vt_brepdb_test.vtbd";

    VersionTree t1;
    t1.Init(make_pool_ab(), "create box");
    t1.Commit(make_pool_cab(), "fillet");

    GeometryPool p2 = make_pool_cab();
    p2.data_pool[p2.headers[1].param_offset] = 77.0;
    t1.Commit(p2, "chamfer");

    GeometryPool pb = make_pool_cab();
    pb.data_pool[pb.headers[0].param_offset] = 55.0;
    t1.Branch(1, pb, VersionTree::ComputeDiff(make_pool_cab(), pb), "branch");

    REQUIRE(t1.SaveToFile(fp));

    VersionTree t2;
    REQUIRE(t2.LoadFromFile(fp));
    REQUIRE(t2.GetNodeCount() == t1.GetNodeCount());

    auto r0 = t2.Checkout(0);
    REQUIRE(r0->headers.size()            == 2);
    REQUIRE(r0->headers[0].persistent_id == 10);

    auto r2 = t2.Checkout(2);
    REQUIRE(r2->data_pool[r2->headers[1].param_offset] == 77.0);

    auto rb = t2.Checkout(3);
    REQUIRE(rb->data_pool[rb->headers[0].param_offset] == 55.0);
}

TEST_CASE("Persistence: StoreToByteArray / LoadFromByteArray (BrepDB meta page path)", "[persist]")
{
    VersionTree t1;
    auto p0 = make_pool_ab();
    t1.Init(p0, "create box");

    auto p1 = make_pool_cab();
    t1.Commit(p1, "fillet");

    GeometryPool p2 = p1;
    p2.data_pool[0] = 42.0;
    t1.Commit(p2, "chamfer");

    uint8_t* buf = nullptr;
    uint32_t len = 0;
    t1.StoreToByteArray(&buf, len);
    REQUIRE(buf != nullptr);
    REQUIRE(len > 0);

    // Reload anchored at the current pool (what the RTree would provide)
    VersionTree t2;
    t2.LoadFromByteArray(buf, len, p2);
    delete[] buf;

    REQUIRE(t2.GetNodeCount()  == 3);
    REQUIRE(t2.GetCurrentId()  == t1.GetCurrentId());

    auto u1 = t2.Undo();
    REQUIRE(u1->headers.size()            == 3);
    REQUIRE(u1->data_pool[0]              != 42.0);

    auto u0 = t2.Undo();
    REQUIRE(u0->headers.size()            == 2);
    REQUIRE(u0->headers[0].persistent_id == 10);

    t2.Redo();
    auto r2 = t2.Redo();
    REQUIRE(r2->data_pool[0] == 42.0);
}

TEST_CASE("Persistence: byte-array with branches", "[persist]")
{
    VersionTree t1;
    auto p0 = make_pool_ab();
    t1.Init(p0, "init");

    GeometryPool pA;
    pA.headers.push_back(make_header(Type::Face, 50, 0, 3));
    pA.data_pool = { 10, 20, 30 };

    VersionTree::PidMapping mapA;
    mapA[10] = { 50 };
    t1.Commit(pA, VersionTree::BuildDiffFromPidMapping(p0, pA, mapA), "branch A");

    GeometryPool pB;
    pB.headers.push_back(make_header(Type::Face, 60, 0, 3));
    pB.data_pool = { 100, 200, 300 };

    VersionTree::PidMapping mapB;
    mapB[10] = { 60 };
    t1.Branch(0, pB, VersionTree::BuildDiffFromPidMapping(p0, pB, mapB), "branch B");

    uint8_t* buf = nullptr;
    uint32_t len = 0;
    t1.StoreToByteArray(&buf, len);

    VersionTree t2;
    t2.LoadFromByteArray(buf, len, pB);
    delete[] buf;

    REQUIRE(t2.GetNodeCount() == 3);

    auto rA = t2.Checkout(1);
    REQUIRE(rA->headers[0].persistent_id == 50);

    auto r0 = t2.Checkout(0);
    REQUIRE(r0->headers[0].persistent_id == 10);

    auto rB = t2.Checkout(2);
    REQUIRE(rB->data_pool[0] == 100.0);
}

// ============================================================
// BrepDB integration flow
// ============================================================

TEST_CASE("Integration: BrepDB open/save/undo/redo cycle", "[integration]")
{
    auto p0 = make_pool_ab();

    VersionTree tree;
    tree.Init(p0, "create box");

    // Modeling op: add face_c
    auto p1 = make_pool_cab();
    VersionTree::PidMapping map1;
    map1[10] = { 10 };
    map1[20] = { 20 };
    tree.Commit(p1, VersionTree::BuildDiffFromPidMapping(p0, p1, map1), "fillet");

    // Simulate save: serialize history without pool data
    uint8_t* save_buf = nullptr;
    uint32_t save_len = 0;
    tree.StoreToByteArray(&save_buf, save_len);

    // Simulate load: current pool comes from the RTree
    VersionTree loaded;
    loaded.LoadFromByteArray(save_buf, save_len, p1);
    delete[] save_buf;

    REQUIRE(loaded.GetNodeCount() == 2);
    REQUIRE(pools_equal(*loaded.GetCurrentPool(), p1));

    auto undo_pool = loaded.Undo();
    REQUIRE(undo_pool->headers.size()            == 2);
    REQUIRE(undo_pool->headers[0].persistent_id == 10);

    auto redo_pool = loaded.Redo();
    REQUIRE(pools_equal(*redo_pool, p1));
}
