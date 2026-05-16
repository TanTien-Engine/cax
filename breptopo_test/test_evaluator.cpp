#include <catch2/catch_test_macros.hpp>

#include "CompGraph.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <set>
#include <thread>

using namespace breptopo;

// Dummy ops that don't depend on OCCT.
// "add_nums" : (double, double) -> double
// "make_shape": (double) -> ShapeVal  (tag = int(input))
// "transform" : (ShapeVal, double) -> ShapeVal (tag = input_tag + int(offset))
static void RegisterTestOps(OpRegistry& reg)
{
    reg.Define("add_nums", {"a", "b"}, {},
        [](EvalCtx& ctx) -> Val {
            return ctx.Num(0) + ctx.Num(1);
        });

    reg.Define("make_shape", {"size"}, {},
        [](EvalCtx& ctx) -> Val {
            ShapeVal sv;
            sv.shape = nullptr;
            sv.tag = static_cast<uint32_t>(ctx.Num(0));
            return sv;
        });

    reg.Define("transform", {"shape", "offset"}, {},
        [](EvalCtx& ctx) -> Val {
            auto sv = ctx.GetShape(0);
            ShapeVal out;
            out.shape = nullptr;
            out.tag = sv.tag + static_cast<uint32_t>(ctx.Num(1));
            return out;
        });
}

TEST_CASE("Evaluator: non-shape values stay in IRNode::cached", "[evaluator]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    auto c1 = g.Const(3.0);
    auto c2 = g.Const(7.0);
    auto add = g.Add("add_nums", {c1, c2});

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;  // null -- test ops don't use it
    Val result = eval.Run(g, add, tn);

    REQUIRE(std::holds_alternative<double>(result));
    REQUIRE(std::get<double>(result) == 10.0);

    // non-shape result should be in IRNode::cached
    auto* nd = g.Get(add);
    REQUIRE(std::holds_alternative<double>(nd->cached));

    // shape cache should be empty (no shapes in this graph)
    REQUIRE(eval.GetShapeCache().Size() == 0);
}

TEST_CASE("Evaluator: shape values go to ShapeCache, not IRNode::cached", "[evaluator]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    auto sz = g.Const(42.0);
    auto mk = g.Add("make_shape", {sz});

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;
    Val result = eval.Run(g, mk, tn);

    REQUIRE(std::holds_alternative<ShapeVal>(result));
    REQUIRE(std::get<ShapeVal>(result).tag == 42);

    // shape should NOT be in IRNode::cached
    auto* nd = g.Get(mk);
    REQUIRE(std::holds_alternative<std::monostate>(nd->cached));

    // shape should be in ShapeCache
    auto* cached = eval.GetShapeCache().Get(mk.id);
    REQUIRE(cached != nullptr);
    REQUIRE(std::holds_alternative<ShapeVal>(*cached));
    REQUIRE(std::get<ShapeVal>(*cached).tag == 42);
}

TEST_CASE("Evaluator: shape input resolution via ShapeCache", "[evaluator]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    auto sz  = g.Const(10.0);
    auto mk  = g.Add("make_shape", {sz});
    auto off = g.Const(5.0);
    auto tr  = g.Add("transform", {mk, off});

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;
    Val result = eval.Run(g, tr, tn);

    REQUIRE(std::holds_alternative<ShapeVal>(result));
    REQUIRE(std::get<ShapeVal>(result).tag == 15);

    // both shape nodes should be in cache
    REQUIRE(eval.GetShapeCache().Get(mk.id) != nullptr);
    REQUIRE(eval.GetShapeCache().Get(tr.id) != nullptr);
}

TEST_CASE("Evaluator: cache hit on second run", "[evaluator]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    auto sz = g.Const(7.0);
    auto mk = g.Add("make_shape", {sz});

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;

    eval.Run(g, mk, tn);
    REQUIRE(eval.CacheMisses() == 1);

    eval.ResetStats();
    Val result = eval.Run(g, mk, tn);

    REQUIRE(eval.CacheHits() == 1);
    REQUIRE(eval.CacheMisses() == 0);
    REQUIRE(std::get<ShapeVal>(result).tag == 7);
}

TEST_CASE("Evaluator: invalidate clears shape from cache", "[evaluator]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    auto sz  = g.Const(10.0);
    auto mk  = g.Add("make_shape", {sz});
    auto off = g.Const(5.0);
    auto tr  = g.Add("transform", {mk, off});

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;
    eval.Run(g, tr, tn);

    REQUIRE(eval.GetShapeCache().Get(mk.id) != nullptr);
    REQUIRE(eval.GetShapeCache().Get(tr.id) != nullptr);

    // Invalidate(mk) is O(1): it clears mk's own cache entry only.
    // Downstream tr's cache is left alone -- the demand-driven EvalNode
    // catches the staleness lazily on the next Run via per-input result_rev
    // comparison and refreshes tr then.
    eval.Invalidate(g, mk);
    REQUIRE(eval.GetShapeCache().Get(mk.id) == nullptr);
    REQUIRE(eval.GetShapeCache().Get(tr.id) != nullptr);  // still cached, will refresh on next Run

    eval.Run(g, tr, tn);
    // After re-run both are fresh again.
    REQUIRE(eval.GetShapeCache().Get(mk.id) != nullptr);
    REQUIRE(eval.GetShapeCache().Get(tr.id) != nullptr);
}

TEST_CASE("Evaluator: re-eval after invalidation produces correct result", "[evaluator]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    auto sz  = g.Const(10.0);
    auto mk  = g.Add("make_shape", {sz});
    auto off = g.Const(5.0);
    auto tr  = g.Add("transform", {mk, off});

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;
    eval.Run(g, tr, tn);

    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, tr)).tag == 15);

    // change size from 10 -> 20
    g.UpdateImmediate(sz, Val(20.0));
    eval.Invalidate(g, sz);

    eval.Run(g, tr, tn);
    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, tr)).tag == 25);
}

TEST_CASE("Evaluator: constant shape goes to ShapeCache", "[evaluator]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    ShapeVal sv;
    sv.shape = nullptr;
    sv.tag = 99;
    auto cshp = g.Const(sv);

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;
    Val result = eval.Run(g, cshp, tn);

    REQUIRE(std::holds_alternative<ShapeVal>(result));
    REQUIRE(std::get<ShapeVal>(result).tag == 99);

    // constant shape should be in ShapeCache, not in cached
    auto* nd = g.Get(cshp);
    REQUIRE(std::holds_alternative<std::monostate>(nd->cached));
    REQUIRE(eval.GetShapeCache().Get(cshp.id) != nullptr);
}

TEST_CASE("Evaluator: ResolveVal returns non-shape from cached", "[evaluator]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    auto c = g.Const(42.0);

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;
    eval.Run(g, c, tn);

    Val resolved = eval.ResolveVal(g, c);
    REQUIRE(std::holds_alternative<double>(resolved));
    REQUIRE(std::get<double>(resolved) == 42.0);
}

TEST_CASE("Evaluator: LRU eviction triggers re-eval on next Run", "[evaluator]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    // build 3 independent shape nodes (each evaluated by its own Run)
    auto s1 = g.Const(1.0);
    auto m1 = g.Add("make_shape", {s1});
    auto s2 = g.Const(2.0);
    auto m2 = g.Add("make_shape", {s2});
    auto s3 = g.Const(3.0);
    auto m3 = g.Add("make_shape", {s3});

    Evaluator eval(reg);
    // capacity=2 so one shape gets evicted per cycle
    eval.GetShapeCache().SetCapacity(2);

    std::shared_ptr<TopoNaming> tn;
    // Demand-driven Run only visits the root's subtree, so evaluate each
    // independently to populate the cache and trigger LRU eviction.
    eval.Run(g, m1, tn);
    eval.Run(g, m2, tn);
    eval.Run(g, m3, tn);

    // m1 was the first insert; capacity=2 evicts it when m3 lands.
    REQUIRE(eval.GetShapeCache().Get(m1.id) == nullptr);
    REQUIRE(eval.GetShapeCache().Get(m2.id) != nullptr);
    REQUIRE(eval.GetShapeCache().Get(m3.id) != nullptr);

    // ResolveVal for evicted node (without restore_fn) returns monostate.
    Val lost = eval.ResolveVal(g, m1);
    REQUIRE(std::holds_alternative<std::monostate>(lost));

    // Re-running m1 re-evaluates it (cache miss) and re-populates the cache.
    eval.ResetStats();
    eval.Run(g, m1, tn);
    REQUIRE(eval.CacheMisses() >= 1);
    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, m1)).tag == 1);

    // widen capacity so all shapes fit, then re-run each and verify values
    eval.GetShapeCache().SetCapacity(4);
    eval.Run(g, m1, tn);
    eval.Run(g, m2, tn);
    eval.Run(g, m3, tn);
    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, m1)).tag == 1);
    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, m2)).tag == 2);
    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, m3)).tag == 3);
}

TEST_CASE("Evaluator: evicted shape input still produces correct downstream result", "[evaluator]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    // chain: make_shape(10) -> transform(+5) -> transform(+3)
    auto sz  = g.Const(10.0);
    auto mk  = g.Add("make_shape", {sz});
    auto o1  = g.Const(5.0);
    auto tr1 = g.Add("transform", {mk, o1});
    auto o2  = g.Const(3.0);
    auto tr2 = g.Add("transform", {tr1, o2});

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;
    eval.Run(g, tr2, tn);

    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, tr2)).tag == 18);

    // shrink cache to evict earlier nodes
    eval.GetShapeCache().SetCapacity(1);
    // only tr2 (last accessed) should remain
    REQUIRE(eval.GetShapeCache().Get(mk.id) == nullptr);

    // re-run: should re-evaluate evicted nodes and still get correct answer
    eval.Run(g, tr2, tn);
    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, tr2)).tag == 18);
}

// ---------------------------------------------------------------
//  Mock version store for testing commit/restore callbacks
// ---------------------------------------------------------------

struct MockVersionStore
{
    std::unordered_map<uint32_t, Val> store;
    uint32_t next_id = 0;

    uint32_t commit(uint32_t /*nref_id*/, const Val& val)
    {
        uint32_t id = next_id++;
        store[id] = val;
        return id;
    }

    Val restore(uint32_t vt_node_id) const
    {
        auto it = store.find(vt_node_id);
        return it != store.end() ? it->second : Val{};
    }
};

TEST_CASE("Evaluator: commit callback stores shapes to version store", "[restore]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    auto sz = g.Const(10.0);
    auto mk = g.Add("make_shape", {sz});

    MockVersionStore vs;
    Evaluator eval(reg);
    eval.SetCommitFn([&](uint32_t nref_id, const Val& v, const std::shared_ptr<TopoNaming>&) {
        return vs.commit(nref_id, v);
    });

    std::shared_ptr<TopoNaming> tn;
    eval.Run(g, mk, tn);

    // shape should be committed to version store
    REQUIRE(vs.store.size() == 1);

    // IRNode should have a valid vt_node_id
    auto* nd = g.Get(mk);
    REQUIRE(nd->vt_node_id != UINT32_MAX);

    // version store holds the correct shape
    auto& stored = vs.store[nd->vt_node_id];
    REQUIRE(std::holds_alternative<ShapeVal>(stored));
    REQUIRE(std::get<ShapeVal>(stored).tag == 10);
}

TEST_CASE("Evaluator: restore callback recovers evicted shapes", "[restore]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    auto sz  = g.Const(10.0);
    auto mk  = g.Add("make_shape", {sz});
    auto off = g.Const(5.0);
    auto tr  = g.Add("transform", {mk, off});

    MockVersionStore vs;
    Evaluator eval(reg);
    eval.SetCommitFn([&](uint32_t nref_id, const Val& v, const std::shared_ptr<TopoNaming>&) {
        return vs.commit(nref_id, v);
    });
    eval.SetRestoreFn([&](uint32_t vt_id, const std::shared_ptr<TopoNaming>&) {
        return vs.restore(vt_id);
    });

    std::shared_ptr<TopoNaming> tn;
    eval.Run(g, tr, tn);

    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, tr)).tag == 15);
    REQUIRE(vs.store.size() == 2);  // mk and tr both committed

    // evict all shapes from LRU
    eval.GetShapeCache().Clear();
    REQUIRE(eval.GetShapeCache().Size() == 0);

    // ResolveVal should restore from version store via callback
    eval.ResetStats();
    Val restored_mk = eval.ResolveVal(g, mk);
    REQUIRE(std::holds_alternative<ShapeVal>(restored_mk));
    REQUIRE(std::get<ShapeVal>(restored_mk).tag == 10);

    Val restored_tr = eval.ResolveVal(g, tr);
    REQUIRE(std::holds_alternative<ShapeVal>(restored_tr));
    REQUIRE(std::get<ShapeVal>(restored_tr).tag == 15);

    REQUIRE(eval.CacheRestores() == 2);

    // restored shapes should now be back in LRU
    REQUIRE(eval.GetShapeCache().Get(mk.id) != nullptr);
    REQUIRE(eval.GetShapeCache().Get(tr.id) != nullptr);
}

TEST_CASE("Evaluator: restore avoids re-eval when LRU evicts", "[restore]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    // 3 shape nodes, capacity=2. Each is evaluated by its own Run since
    // demand-driven Run(root) only walks the root's subtree.
    auto s1 = g.Const(1.0);
    auto m1 = g.Add("make_shape", {s1});
    auto s2 = g.Const(2.0);
    auto m2 = g.Add("make_shape", {s2});
    auto s3 = g.Const(3.0);
    auto m3 = g.Add("make_shape", {s3});

    MockVersionStore vs;
    Evaluator eval(reg);
    eval.GetShapeCache().SetCapacity(2);
    eval.SetCommitFn([&](uint32_t nref_id, const Val& v, const std::shared_ptr<TopoNaming>&) {
        return vs.commit(nref_id, v);
    });
    eval.SetRestoreFn([&](uint32_t vt_id, const std::shared_ptr<TopoNaming>&) {
        return vs.restore(vt_id);
    });

    std::shared_ptr<TopoNaming> tn;
    eval.Run(g, m1, tn);
    eval.Run(g, m2, tn);
    eval.Run(g, m3, tn);

    // m1 evicted from LRU but committed to version store
    REQUIRE(eval.GetShapeCache().Get(m1.id) == nullptr);
    REQUIRE(vs.store.size() == 3);

    // re-asking for m1: should restore from VT, not re-evaluate.
    eval.ResetStats();
    eval.Run(g, m1, tn);

    REQUIRE(eval.CacheMisses() == 0);
    REQUIRE(eval.CacheRestores() >= 1);
    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, m1)).tag == 1);
}

TEST_CASE("Evaluator: constant shape committed to version store", "[restore]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    ShapeVal sv;
    sv.shape = nullptr;
    sv.tag = 77;
    auto cshp = g.Const(sv);

    MockVersionStore vs;
    Evaluator eval(reg);
    eval.SetCommitFn([&](uint32_t nref_id, const Val& v, const std::shared_ptr<TopoNaming>&) {
        return vs.commit(nref_id, v);
    });
    eval.SetRestoreFn([&](uint32_t vt_id, const std::shared_ptr<TopoNaming>&) {
        return vs.restore(vt_id);
    });

    std::shared_ptr<TopoNaming> tn;
    eval.Run(g, cshp, tn);

    auto* nd = g.Get(cshp);
    REQUIRE(nd->vt_node_id != UINT32_MAX);
    REQUIRE(vs.store.size() == 1);

    // evict and restore
    eval.GetShapeCache().Clear();
    Val restored = eval.ResolveVal(g, cshp);
    REQUIRE(std::get<ShapeVal>(restored).tag == 77);
}

TEST_CASE("Evaluator: invalidation clears vt_node_id for dirty nodes", "[restore]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    auto sz  = g.Const(10.0);
    auto mk  = g.Add("make_shape", {sz});

    MockVersionStore vs;
    Evaluator eval(reg);
    eval.SetCommitFn([&](uint32_t nref_id, const Val& v, const std::shared_ptr<TopoNaming>&) {
        return vs.commit(nref_id, v);
    });
    eval.SetRestoreFn([&](uint32_t vt_id, const std::shared_ptr<TopoNaming>&) {
        return vs.restore(vt_id);
    });

    std::shared_ptr<TopoNaming> tn;
    eval.Run(g, mk, tn);

    auto* nd = g.Get(mk);
    REQUIRE(nd->vt_node_id != UINT32_MAX);

    // change the parameter -> invalidate. The new O(1) Invalidate only
    // clears the invalidated node's own state; downstream `mk` keeps its
    // old vt_node_id until the next Run picks up sz's bumped result_rev
    // and re-eval's mk (which assigns a fresh vt_node_id then).
    g.UpdateImmediate(sz, Val(20.0));
    eval.Invalidate(g, sz);

    // re-eval re-commits mk with a new vt_node_id
    uint32_t old_vt_id = nd->vt_node_id;
    eval.Run(g, mk, tn);
    REQUIRE(nd->vt_node_id != UINT32_MAX);
    REQUIRE(nd->vt_node_id != old_vt_id);
    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, mk)).tag == 20);
}

// ---------------------------------------------------------------
//  CollectDeps / AreIndependent / TopoLevels tests
// ---------------------------------------------------------------

TEST_CASE("IRGraph: CollectDeps returns full transitive closure", "[graph]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    //  c1 -> add_nums -> transform
    //  c2 ----^             ^
    //  c3 (offset) ---------+
    auto c1 = g.Const(1.0);
    auto c2 = g.Const(2.0);
    auto add = g.Add("add_nums", {c1, c2});
    auto c3 = g.Const(3.0);
    auto mk = g.Add("make_shape", {add});
    auto tr = g.Add("transform", {mk, c3});

    auto deps = g.CollectDeps(tr);
    REQUIRE(deps.size() == 6);  // tr, mk, add, c1, c2, c3
    REQUIRE(deps.count(tr.id));
    REQUIRE(deps.count(mk.id));
    REQUIRE(deps.count(add.id));
    REQUIRE(deps.count(c1.id));
    REQUIRE(deps.count(c2.id));
    REQUIRE(deps.count(c3.id));

    auto deps_add = g.CollectDeps(add);
    REQUIRE(deps_add.size() == 3);  // add, c1, c2
    REQUIRE(deps_add.count(add.id));
    REQUIRE(deps_add.count(c1.id));
    REQUIRE(deps_add.count(c2.id));

    auto deps_c1 = g.CollectDeps(c1);
    REQUIRE(deps_c1.size() == 1);
    REQUIRE(deps_c1.count(c1.id));
}

TEST_CASE("IRGraph: AreIndependent detects independent subtrees", "[graph]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    // subtree A: c1 -> make_shape -> transform(c2)
    auto c1 = g.Const(10.0);
    auto mkA = g.Add("make_shape", {c1});
    auto offA = g.Const(5.0);
    auto trA = g.Add("transform", {mkA, offA});

    // subtree B: c3 -> make_shape -> transform(c4)
    auto c3 = g.Const(20.0);
    auto mkB = g.Add("make_shape", {c3});
    auto offB = g.Const(7.0);
    auto trB = g.Add("transform", {mkB, offB});

    REQUIRE(g.AreIndependent(trA, trB));
    REQUIRE(g.AreIndependent(mkA, mkB));
    REQUIRE(g.AreIndependent(trA, mkB));
}

TEST_CASE("IRGraph: AreIndependent detects shared nodes", "[graph]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    // shared root
    auto shared = g.Const(10.0);
    auto mkA = g.Add("make_shape", {shared});
    auto mkB = g.Add("make_shape", {shared});

    REQUIRE_FALSE(g.AreIndependent(mkA, mkB));

    // deeper sharing
    auto offA = g.Const(1.0);
    auto trA = g.Add("transform", {mkA, offA});
    auto offB = g.Const(2.0);
    auto trB = g.Add("transform", {mkB, offB});

    REQUIRE_FALSE(g.AreIndependent(trA, trB));
}

TEST_CASE("DepIndex: O(1) independence check matches brute-force", "[graph]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    // independent subtrees
    auto c1 = g.Const(10.0);
    auto mkA = g.Add("make_shape", {c1});
    auto offA = g.Const(5.0);
    auto trA = g.Add("transform", {mkA, offA});

    auto c3 = g.Const(20.0);
    auto mkB = g.Add("make_shape", {c3});
    auto offB = g.Const(7.0);
    auto trB = g.Add("transform", {mkB, offB});

    auto idx = g.BuildDepIndex();
    REQUIRE(idx.AreIndependent(trA, trB));
    REQUIRE(idx.AreIndependent(mkA, mkB));
    REQUIRE(g.AreIndependent(trA, trB));  // cross-check

    // shared root -> not independent
    auto shared = g.Const(99.0);
    auto mkC = g.Add("make_shape", {shared});
    auto mkD = g.Add("make_shape", {shared});
    auto idx2 = g.BuildDepIndex();
    REQUIRE_FALSE(idx2.AreIndependent(mkC, mkD));
    REQUIRE_FALSE(g.AreIndependent(mkC, mkD));  // cross-check
}

TEST_CASE("DepIndex: wide graph with many leaves", "[graph]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    // 100 independent branches -- tests bitset > 64 bits
    std::vector<NRef> branches;
    for (int i = 0; i < 100; ++i)
    {
        auto c = g.Const(static_cast<double>(i));
        branches.push_back(g.Add("make_shape", {c}));
    }

    auto idx = g.BuildDepIndex();
    for (int i = 0; i < 100; ++i)
        for (int j = i + 1; j < 100; j += 17)
            REQUIRE(idx.AreIndependent(branches[i], branches[j]));
}

TEST_CASE("IRGraph: TopoLevels groups nodes by level", "[graph]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    // level 0: c1, c2, c3
    // level 1: mkA(c1), mkB(c2)
    // level 2: trA(mkA, c3)
    auto c1 = g.Const(10.0);
    auto c2 = g.Const(20.0);
    auto c3 = g.Const(5.0);
    auto mkA = g.Add("make_shape", {c1});
    auto mkB = g.Add("make_shape", {c2});
    auto trA = g.Add("transform", {mkA, c3});

    auto levels = g.TopoLevels();
    REQUIRE(levels.size() == 3);

    // level 0: all constants
    std::unordered_set<uint32_t> lv0;
    for (auto& r : levels[0]) lv0.insert(r.id);
    REQUIRE(lv0.count(c1.id));
    REQUIRE(lv0.count(c2.id));
    REQUIRE(lv0.count(c3.id));

    // level 1: mkA and mkB
    std::unordered_set<uint32_t> lv1;
    for (auto& r : levels[1]) lv1.insert(r.id);
    REQUIRE(lv1.count(mkA.id));
    REQUIRE(lv1.count(mkB.id));

    // level 2: trA
    REQUIRE(levels[2].size() == 1);
    REQUIRE(levels[2][0].id == trA.id);
}

// ---------------------------------------------------------------
//  RunParallel tests
// ---------------------------------------------------------------

static void RegisterSlowTestOps(OpRegistry& reg)
{
    reg.Define("slow_shape", {"size"}, {},
        [](EvalCtx& ctx) -> Val {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            ShapeVal sv;
            sv.shape = nullptr;
            sv.tag = static_cast<uint32_t>(ctx.Num(0));
            return sv;
        });

    reg.Define("slow_combine", {"shape1", "shape2"}, {},
        [](EvalCtx& ctx) -> Val {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            auto s1 = ctx.GetShape(0);
            auto s2 = ctx.GetShape(1);
            ShapeVal out;
            out.shape = nullptr;
            out.tag = s1.tag + s2.tag;
            return out;
        },
        {false, false, true, false});  // is_boolean
}

TEST_CASE("RunParallel: produces correct results for independent subtrees", "[parallel]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    // two independent subtrees joined by add_nums
    auto c1 = g.Const(10.0);
    auto c2 = g.Const(20.0);
    auto mkA = g.Add("make_shape", {c1});
    auto mkB = g.Add("make_shape", {c2});
    auto offA = g.Const(3.0);
    auto offB = g.Const(7.0);
    auto trA = g.Add("transform", {mkA, offA});
    auto trB = g.Add("transform", {mkB, offB});

    // verify independence
    REQUIRE(g.AreIndependent(trA, trB));

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;
    Val rA = eval.RunParallel(g,trA, tn);
    REQUIRE(std::holds_alternative<ShapeVal>(rA));
    REQUIRE(std::get<ShapeVal>(rA).tag == 13);

    Val rB = eval.ResolveVal(g, trB);
    REQUIRE(std::holds_alternative<ShapeVal>(rB));
    REQUIRE(std::get<ShapeVal>(rB).tag == 27);
}

TEST_CASE("RunParallel: matches sequential Run results", "[parallel]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    auto c1 = g.Const(10.0);
    auto mk = g.Add("make_shape", {c1});
    auto off = g.Const(5.0);
    auto tr = g.Add("transform", {mk, off});

    Evaluator eval_seq(reg);
    Evaluator eval_par(reg);
    std::shared_ptr<TopoNaming> tn;

    Val seq_result = eval_seq.Run(g, tr, tn);
    Val par_result = eval_par.RunParallel(g,tr, tn);

    REQUIRE(std::get<ShapeVal>(seq_result).tag == std::get<ShapeVal>(par_result).tag);
}

TEST_CASE("RunParallel: actually runs in parallel (timing)", "[parallel]")
{
    OpRegistry reg;
    RegisterSlowTestOps(reg);
    IRGraph g(reg);

    // two independent slow_shape nodes at the same topo level
    auto c1 = g.Const(10.0);
    auto c2 = g.Const(20.0);
    auto mkA = g.Add("slow_shape", {c1});
    auto mkB = g.Add("slow_shape", {c2});
    auto combine = g.Add("slow_combine", {mkA, mkB});

    REQUIRE(g.AreIndependent(mkA, mkB));

    // sequential: 3 * 100ms = ~300ms
    {
        Evaluator eval(reg);
        std::shared_ptr<TopoNaming> tn;
        auto t0 = std::chrono::steady_clock::now();
        eval.Run(g, combine, tn);
        auto t1 = std::chrono::steady_clock::now();
        auto seq_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        REQUIRE(seq_ms >= 280);
    }

    // parallel: max(100,100) + 100 = ~200ms
    {
        Evaluator eval(reg);
        std::shared_ptr<TopoNaming> tn;
        auto t0 = std::chrono::steady_clock::now();
        eval.RunParallel(g,combine, tn);
        auto t1 = std::chrono::steady_clock::now();
        auto par_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        REQUIRE(par_ms < 280);
    }

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;
    Val result = eval.RunParallel(g,combine, tn);
    REQUIRE(std::holds_alternative<ShapeVal>(result));
    REQUIRE(std::get<ShapeVal>(result).tag == 30);
}

TEST_CASE("RunParallel: handles cache hits correctly", "[parallel]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    auto c1 = g.Const(10.0);
    auto c2 = g.Const(20.0);
    auto mkA = g.Add("make_shape", {c1});
    auto mkB = g.Add("make_shape", {c2});

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;

    // first run populates caches
    eval.RunParallel(g,mkB, tn);
    REQUIRE(eval.CacheMisses() == 2);

    // second run should hit caches
    eval.ResetStats();
    eval.RunParallel(g,mkB, tn);
    REQUIRE(eval.CacheHits() == 2);
    REQUIRE(eval.CacheMisses() == 0);
}

TEST_CASE("RunParallel: invalidation then re-eval", "[parallel]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    auto c1 = g.Const(10.0);
    auto mk = g.Add("make_shape", {c1});
    auto off = g.Const(5.0);
    auto tr = g.Add("transform", {mk, off});

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;
    eval.RunParallel(g,tr, tn);
    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, tr)).tag == 15);

    g.UpdateImmediate(c1, Val(20.0));
    eval.Invalidate(g, c1);

    eval.RunParallel(g,tr, tn);
    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, tr)).tag == 25);
}

TEST_CASE("RunParallel: wide graph with many independent branches", "[parallel]")
{
    OpRegistry reg;
    RegisterTestOps(reg);
    IRGraph g(reg);

    std::vector<NRef> branches;
    for (int i = 1; i <= 8; ++i)
    {
        auto c = g.Const(static_cast<double>(i));
        auto mk = g.Add("make_shape", {c});
        branches.push_back(mk);
    }

    // all branches should be pairwise independent
    for (size_t i = 0; i < branches.size(); ++i)
        for (size_t j = i + 1; j < branches.size(); ++j)
            REQUIRE(g.AreIndependent(branches[i], branches[j]));

    auto levels = g.TopoLevels();
    REQUIRE(levels.size() == 2);
    REQUIRE(levels[0].size() == 8);  // all constants
    REQUIRE(levels[1].size() == 8);  // all make_shape

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;
    eval.RunParallel(g,branches.back(), tn);

    for (int i = 0; i < 8; ++i)
    {
        Val v = eval.ResolveVal(g, branches[i]);
        REQUIRE(std::holds_alternative<ShapeVal>(v));
        REQUIRE(std::get<ShapeVal>(v).tag == static_cast<uint32_t>(i + 1));
    }
}

// ---------------------------------------------------------------
//  CompGraph facade integration tests (real modeling scenarios)
// ---------------------------------------------------------------

// Register CAD-like ops that simulate real boolean modeling workloads
static void RegisterCADTestOps(OpRegistry& reg)
{
    reg.Define("box", {"length", "width", "height"}, {},
        [](EvalCtx& ctx) -> Val {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            ShapeVal sv;
            sv.shape = nullptr;
            sv.tag = static_cast<uint32_t>(ctx.Num(0) * 100 + ctx.Num(1) * 10 + ctx.Num(2));
            return sv;
        });

    reg.Define("translate", {"shape", "offset"}, {},
        [](EvalCtx& ctx) -> Val {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            auto sv = ctx.GetShape(0);
            auto off = ctx.GetVec3(1);
            ShapeVal out;
            out.shape = nullptr;
            out.tag = sv.tag + static_cast<uint32_t>(off[0]);
            return out;
        });

    reg.Define("cut", {"shape1", "shape2"}, {},
        [](EvalCtx& ctx) -> Val {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            auto s1 = ctx.GetShape(0);
            auto s2 = ctx.GetShape(1);
            ShapeVal out;
            out.shape = nullptr;
            out.tag = s1.tag * 1000 + s2.tag;
            return out;
        },
        {false, false, true, false});

    reg.Define("fuse", {"shape1", "shape2"}, {},
        [](EvalCtx& ctx) -> Val {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            auto s1 = ctx.GetShape(0);
            auto s2 = ctx.GetShape(1);
            ShapeVal out;
            out.shape = nullptr;
            out.tag = s1.tag + s2.tag;
            return out;
        },
        {false, false, true, false});

    reg.Define("fillet", {"shape", "radius"}, {"edges"},
        [](EvalCtx& ctx) -> Val {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            auto sv = ctx.GetShape(0);
            ShapeVal out;
            out.shape = nullptr;
            out.tag = sv.tag + static_cast<uint32_t>(ctx.Num(1));
            return out;
        },
        {true, false, false, false});
}

TEST_CASE("CAD: boolean cut with two independent box subtrees", "[integration]")
{
    // Mirrors test/breptopo/nodes/rollback.ves:
    //   boxA -> cut(_, boxB->translate) via CompGraph
    OpRegistry reg;
    RegisterCADTestOps(reg);
    IRGraph g(reg);

    // subtree A: box(1,2,3) -> translate(offset=(10,0,0))
    auto lenA = g.Const(1.0);
    auto widA = g.Const(2.0);
    auto htA  = g.Const(3.0);
    auto boxA = g.Add("box", {lenA, widA, htA});
    auto offA = g.Const(Vec3{10, 0, 0});
    auto trA  = g.Add("translate", {boxA, offA});

    // subtree B: box(4,5,6)
    auto lenB = g.Const(4.0);
    auto widB = g.Const(5.0);
    auto htB  = g.Const(6.0);
    auto boxB = g.Add("box", {lenB, widB, htB});

    auto cut  = g.Add("cut", {trA, boxB});

    REQUIRE(g.AreIndependent(trA, boxB));

    // sequential
    Evaluator eval_seq(reg);
    std::shared_ptr<TopoNaming> tn;
    Val seq_result = eval_seq.Run(g, cut, tn);
    REQUIRE(std::holds_alternative<ShapeVal>(seq_result));
    uint32_t seq_tag = std::get<ShapeVal>(seq_result).tag;

    // parallel
    Evaluator eval_par(reg);
    Val par_result = eval_par.RunParallel(g,cut, tn);
    REQUIRE(std::holds_alternative<ShapeVal>(par_result));
    REQUIRE(std::get<ShapeVal>(par_result).tag == seq_tag);
}

TEST_CASE("CAD: parallel boolean is faster than sequential", "[integration]")
{
    // boxA(30ms) + translate(20ms) -> cut(30ms) <- boxB(30ms)
    // Sequential: 30+20+30+30 = 110ms
    // Parallel:   max(30+20, 30) + 30 = 80ms
    OpRegistry reg;
    RegisterCADTestOps(reg);

    auto build_graph = [&](IRGraph& g) -> NRef {
        auto lenA = g.Const(1.0);
        auto widA = g.Const(2.0);
        auto htA  = g.Const(3.0);
        auto boxA = g.Add("box", {lenA, widA, htA});
        auto offA = g.Const(Vec3{5, 0, 0});
        auto trA  = g.Add("translate", {boxA, offA});

        auto lenB = g.Const(4.0);
        auto widB = g.Const(5.0);
        auto htB  = g.Const(6.0);
        auto boxB = g.Add("box", {lenB, widB, htB});

        return g.Add("cut", {trA, boxB});
    };

    long seq_ms, par_ms;

    {
        IRGraph g(reg);
        auto root = build_graph(g);
        Evaluator eval(reg);
        std::shared_ptr<TopoNaming> tn;
        auto t0 = std::chrono::steady_clock::now();
        eval.Run(g, root, tn);
        auto t1 = std::chrono::steady_clock::now();
        seq_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    }

    {
        IRGraph g(reg);
        auto root = build_graph(g);
        Evaluator eval(reg);
        std::shared_ptr<TopoNaming> tn;
        auto t0 = std::chrono::steady_clock::now();
        eval.RunParallel(g,root, tn);
        auto t1 = std::chrono::steady_clock::now();
        par_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    }

    REQUIRE(seq_ms >= 100);
    REQUIRE(par_ms < seq_ms);
}

TEST_CASE("CAD: parallel param update + re-eval (fuse)", "[integration]")
{
    // fuse(boxA, boxB), then change boxA param and re-eval in parallel
    OpRegistry reg;
    RegisterCADTestOps(reg);
    IRGraph g(reg);

    auto lenA = g.Const(1.0);
    auto widA = g.Const(2.0);
    auto htA  = g.Const(3.0);
    auto boxA = g.Add("box", {lenA, widA, htA});

    auto lenB = g.Const(4.0);
    auto widB = g.Const(5.0);
    auto htB  = g.Const(6.0);
    auto boxB = g.Add("box", {lenB, widB, htB});

    auto fuse = g.Add("fuse", {boxA, boxB});

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;
    Val r1 = eval.RunParallel(g,fuse, tn);
    REQUIRE(std::holds_alternative<ShapeVal>(r1));
    uint32_t tag1 = std::get<ShapeVal>(r1).tag;

    // change lenA: 1 -> 9
    g.UpdateImmediate(lenA, Val(9.0));
    eval.Invalidate(g, lenA);

    Val r2 = eval.RunParallel(g,fuse, tn);
    REQUIRE(std::holds_alternative<ShapeVal>(r2));
    uint32_t tag2 = std::get<ShapeVal>(r2).tag;

    REQUIRE(tag2 != tag1);
    // boxA: 9*100+2*10+3=923, boxB: 4*100+5*10+6=456, fuse = 923+456 = 1379
    REQUIRE(tag2 == 1379);
}

TEST_CASE("CAD: complex multi-boolean pipeline (cut + fuse + fillet)", "[integration]")
{
    // Mirrors test/breptopo/nodes/compute.ves:
    //   boxA -> fillet -> cut(_, boxB->translate) via CompGraph
    // Extended with fuse:
    //
    //   boxA -> trA --+
    //                 +--> cut --+
    //   boxB ---------+         +--> fuse --> fillet
    //                           |
    //   boxC -> trC ------------+

    OpRegistry reg;
    RegisterCADTestOps(reg);
    IRGraph g(reg);

    auto la = g.Const(1.0); auto wa = g.Const(1.0); auto ha = g.Const(1.0);
    auto boxA = g.Add("box", {la, wa, ha});
    auto offA = g.Const(Vec3{5, 0, 0});
    auto trA  = g.Add("translate", {boxA, offA});

    auto lb = g.Const(2.0); auto wb = g.Const(2.0); auto hb = g.Const(2.0);
    auto boxB = g.Add("box", {lb, wb, hb});

    auto cut = g.Add("cut", {trA, boxB});

    auto lc = g.Const(3.0); auto wc = g.Const(3.0); auto hc = g.Const(3.0);
    auto boxC = g.Add("box", {lc, wc, hc});
    auto offC = g.Const(Vec3{1, 0, 0});
    auto trC  = g.Add("translate", {boxC, offC});

    auto fuse = g.Add("fuse", {cut, trC});

    auto rad = g.Const(2.0);
    auto fillet = g.Add("fillet", {fuse, rad});

    // verify independence
    REQUIRE(g.AreIndependent(trA, boxB));
    REQUIRE(g.AreIndependent(cut, trC));

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;
    Val result = eval.RunParallel(g,fillet, tn);
    REQUIRE(std::holds_alternative<ShapeVal>(result));

    // boxA: 1*100+1*10+1=111, trA: 111+5=116
    // boxB: 2*100+2*10+2=222
    // cut: 116*1000+222=116222
    // boxC: 3*100+3*10+3=333, trC: 333+1=334
    // fuse: 116222+334=116556
    // fillet: 116556+2=116558
    REQUIRE(std::get<ShapeVal>(result).tag == 116558);
}

TEST_CASE("RunParallel: clean nodes in fork subtree are not re-evaluated", "[parallel]")
{
    // Mirrors the compute.ves scenario:
    //   box0 -> fillet -> cut <- translate(box1)
    // When only translate's offset changes, fillet must NOT be re-evaluated.
    OpRegistry reg;

    std::atomic<int> fillet_eval_count{0};

    reg.Define("box", {"size"}, {},
        [](EvalCtx& ctx) -> Val {
            ShapeVal sv;
            sv.shape = nullptr;
            sv.tag = static_cast<uint32_t>(ctx.Num(0));
            return sv;
        });

    reg.Define("fillet_counted", {"shape", "radius"}, {},
        [&fillet_eval_count](EvalCtx& ctx) -> Val {
            fillet_eval_count++;
            auto sv = ctx.GetShape(0);
            ShapeVal out;
            out.shape = nullptr;
            out.tag = sv.tag + static_cast<uint32_t>(ctx.Num(1));
            return out;
        },
        {true, false, false, false});  // is_dressup

    reg.Define("translate", {"shape", "offset"}, {},
        [](EvalCtx& ctx) -> Val {
            auto sv = ctx.GetShape(0);
            ShapeVal out;
            out.shape = nullptr;
            out.tag = sv.tag + static_cast<uint32_t>(ctx.Num(1));
            return out;
        });

    reg.Define("cut", {"a", "b"}, {},
        [](EvalCtx& ctx) -> Val {
            auto a = ctx.GetShape(0);
            auto b = ctx.GetShape(1);
            ShapeVal out;
            out.shape = nullptr;
            out.tag = a.tag * 1000 + b.tag;
            return out;
        },
        {false, false, true, false});  // is_boolean

    IRGraph g(reg);

    // branch A: box0 -> fillet
    auto sz0 = g.Const(10.0);
    auto box0 = g.Add("box", {sz0});
    auto rad = g.Const(2.0);
    auto fillet = g.Add("fillet_counted", {box0, rad});

    // branch B: box1 -> translate
    auto sz1 = g.Const(20.0);
    auto box1 = g.Add("box", {sz1});
    auto offset = g.Const(5.0);
    auto tr = g.Add("translate", {box1, offset});

    auto cut = g.Add("cut", {fillet, tr});

    REQUIRE(g.AreIndependent(fillet, tr));

    // first eval: everything computed
    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;
    eval.RunParallel(g, cut, tn);
    REQUIRE(fillet_eval_count == 1);

    // fillet: 10+2=12, translate: 20+5=25, cut: 12*1000+25=12025
    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, cut)).tag == 12025);

    // change only the translate offset: 5 -> 8
    g.UpdateImmediate(offset, Val(8.0));
    eval.Invalidate(g, offset);

    // re-eval: fillet should NOT be re-evaluated
    fillet_eval_count = 0;
    eval.RunParallel(g, cut, tn);
    REQUIRE(fillet_eval_count == 0);

    // fillet: 12 (unchanged), translate: 20+8=28, cut: 12*1000+28=12028
    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, cut)).tag == 12028);
}

// ---------------------------------------------------------------
//  op_id stability tests
// ---------------------------------------------------------------

TEST_CASE("op_id: serial and parallel assign identical op_ids", "[op_id]")
{
    // Record which op_id each node gets via a custom op that captures it.
    // Build a diamond graph with two independent branches joined by a boolean.
    OpRegistry reg;

    reg.Define("record_shape", {"size"}, {},
        [](EvalCtx& ctx) -> Val {
            ShapeVal sv;
            sv.shape = nullptr;
            sv.tag = ctx.op_id;
            return sv;
        });

    reg.Define("record_transform", {"shape", "offset"}, {},
        [](EvalCtx& ctx) -> Val {
            auto sv = ctx.GetShape(0);
            ShapeVal out;
            out.shape = nullptr;
            out.tag = ctx.op_id;
            return out;
        });

    reg.Define("record_combine", {"shape1", "shape2"}, {},
        [](EvalCtx& ctx) -> Val {
            auto s1 = ctx.GetShape(0);
            auto s2 = ctx.GetShape(1);
            ShapeVal out;
            out.shape = nullptr;
            out.tag = ctx.op_id;
            return out;
        },
        {false, false, true, false});  // is_boolean

    // Graph:
    //   c1 -> mkA -> trA --+
    //                       +--> combine
    //   c2 -> mkB -> trB --+
    IRGraph g(reg);
    auto c1 = g.Const(10.0);
    auto mkA = g.Add("record_shape", {c1});
    auto offA = g.Const(3.0);
    auto trA = g.Add("record_transform", {mkA, offA});

    auto c2 = g.Const(20.0);
    auto mkB = g.Add("record_shape", {c2});
    auto offB = g.Const(7.0);
    auto trB = g.Add("record_transform", {mkB, offB});

    auto combine = g.Add("record_combine", {trA, trB});

    REQUIRE(g.AreIndependent(trA, trB));

    // serial run
    Evaluator eval_seq(reg);
    std::shared_ptr<TopoNaming> tn;
    eval_seq.Run(g, combine, tn);

    uint32_t seq_mkA = std::get<ShapeVal>(eval_seq.ResolveVal(g, mkA)).tag;
    uint32_t seq_trA = std::get<ShapeVal>(eval_seq.ResolveVal(g, trA)).tag;
    uint32_t seq_mkB = std::get<ShapeVal>(eval_seq.ResolveVal(g, mkB)).tag;
    uint32_t seq_trB = std::get<ShapeVal>(eval_seq.ResolveVal(g, trB)).tag;
    uint32_t seq_cmb = std::get<ShapeVal>(eval_seq.ResolveVal(g, combine)).tag;

    // all op_ids should be distinct
    std::set<uint32_t> seq_ids{seq_mkA, seq_trA, seq_mkB, seq_trB, seq_cmb};
    REQUIRE(seq_ids.size() == 5);

    // parallel run -- invalidate first so nodes re-eval
    for (auto ref : g.TopoSort()) {
        auto* nd = g.Get(ref);
        if (nd) { nd->eval_version = 0; nd->cached = {}; }
    }

    Evaluator eval_par(reg);
    eval_par.RunParallel(g, combine, tn);

    uint32_t par_mkA = std::get<ShapeVal>(eval_par.ResolveVal(g, mkA)).tag;
    uint32_t par_trA = std::get<ShapeVal>(eval_par.ResolveVal(g, trA)).tag;
    uint32_t par_mkB = std::get<ShapeVal>(eval_par.ResolveVal(g, mkB)).tag;
    uint32_t par_trB = std::get<ShapeVal>(eval_par.ResolveVal(g, trB)).tag;
    uint32_t par_cmb = std::get<ShapeVal>(eval_par.ResolveVal(g, combine)).tag;

    // serial and parallel must produce identical op_ids
    REQUIRE(par_mkA == seq_mkA);
    REQUIRE(par_trA == seq_trA);
    REQUIRE(par_mkB == seq_mkB);
    REQUIRE(par_trB == seq_trB);
    REQUIRE(par_cmb == seq_cmb);
}

TEST_CASE("op_id: parallel branches never collide", "[op_id]")
{
    OpRegistry reg;

    reg.Define("id_shape", {"size"}, {},
        [](EvalCtx& ctx) -> Val {
            ShapeVal sv;
            sv.shape = nullptr;
            sv.tag = ctx.op_id;
            return sv;
        });

    reg.Define("id_combine", {"shape1", "shape2"}, {},
        [](EvalCtx& ctx) -> Val {
            ShapeVal out;
            out.shape = nullptr;
            out.tag = ctx.op_id;
            return out;
        },
        {false, false, true, false});

    // 4 independent branches feeding into a boolean
    IRGraph g(reg);
    auto c1 = g.Const(1.0);  auto m1 = g.Add("id_shape", {c1});
    auto c2 = g.Const(2.0);  auto m2 = g.Add("id_shape", {c2});
    auto c3 = g.Const(3.0);  auto m3 = g.Add("id_shape", {c3});
    auto c4 = g.Const(4.0);  auto m4 = g.Add("id_shape", {c4});

    // combine pairs, then combine results
    auto comb1 = g.Add("id_combine", {m1, m2});
    auto comb2 = g.Add("id_combine", {m3, m4});
    auto root  = g.Add("id_combine", {comb1, comb2});

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;
    eval.RunParallel(g, root, tn);

    std::set<uint32_t> ids;
    for (auto ref : {m1, m2, m3, m4, comb1, comb2, root}) {
        auto val = eval.ResolveVal(g, ref);
        REQUIRE(std::holds_alternative<ShapeVal>(val));
        uint32_t id = std::get<ShapeVal>(val).tag;
        REQUIRE(ids.count(id) == 0);
        ids.insert(id);
    }
    REQUIRE(ids.size() == 7);
}

TEST_CASE("op_id: stable across re-evaluation after invalidation", "[op_id]")
{
    OpRegistry reg;

    reg.Define("id_shape", {"size"}, {},
        [](EvalCtx& ctx) -> Val {
            ShapeVal sv;
            sv.shape = nullptr;
            sv.tag = ctx.op_id;
            return sv;
        });

    reg.Define("id_transform", {"shape", "offset"}, {},
        [](EvalCtx& ctx) -> Val {
            ShapeVal out;
            out.shape = nullptr;
            out.tag = ctx.op_id;
            return out;
        });

    IRGraph g(reg);
    auto c1 = g.Const(10.0);
    auto mk = g.Add("id_shape", {c1});
    auto off = g.Const(5.0);
    auto tr = g.Add("id_transform", {mk, off});

    Evaluator eval(reg);
    std::shared_ptr<TopoNaming> tn;
    eval.Run(g, tr, tn);

    uint32_t first_mk = std::get<ShapeVal>(eval.ResolveVal(g, mk)).tag;
    uint32_t first_tr = std::get<ShapeVal>(eval.ResolveVal(g, tr)).tag;

    // change param and re-eval
    g.UpdateImmediate(c1, Val(20.0));
    eval.Invalidate(g, c1);
    eval.Run(g, tr, tn);

    uint32_t second_mk = std::get<ShapeVal>(eval.ResolveVal(g, mk)).tag;
    uint32_t second_tr = std::get<ShapeVal>(eval.ResolveVal(g, tr)).tag;

    // op_ids are deterministic from graph structure, not param values
    REQUIRE(second_mk == first_mk);
    REQUIRE(second_tr == first_tr);
}
