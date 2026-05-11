#include <catch2/catch_test_macros.hpp>

#include "CompGraph.h"

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

    // invalidate mk -> should also invalidate downstream tr
    eval.Invalidate(g, mk);

    REQUIRE(eval.GetShapeCache().Get(mk.id) == nullptr);
    REQUIRE(eval.GetShapeCache().Get(tr.id) == nullptr);
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

    // build 3 independent shape nodes
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
    eval.Run(g, m3, tn);

    // m1 was inserted first, should be evicted (capacity 2, m2 & m3 remain)
    REQUIRE(eval.GetShapeCache().Get(m1.id) == nullptr);
    REQUIRE(eval.GetShapeCache().Get(m2.id) != nullptr);
    REQUIRE(eval.GetShapeCache().Get(m3.id) != nullptr);

    // ResolveVal for evicted node returns monostate
    Val lost = eval.ResolveVal(g, m1);
    REQUIRE(std::holds_alternative<std::monostate>(lost));

    // re-run: all evicted shapes must be re-evaluated (cache miss)
    eval.ResetStats();
    eval.Run(g, m3, tn);

    // with capacity=2 and 3 shape nodes, topo-order re-eval causes cascading
    // evictions (m1 Put evicts old LRU, m2 Put evicts another, ...).
    // The final result is that the LAST two nodes in topo order stay in cache.
    // What matters is that all re-evals produced correct results during Run.
    REQUIRE(eval.CacheMisses() >= 1);

    // final value of the root (m3) must still be correct
    Val root_val = eval.ResolveVal(g, m3);
    REQUIRE(std::holds_alternative<ShapeVal>(root_val));
    REQUIRE(std::get<ShapeVal>(root_val).tag == 3);

    // widen capacity so all shapes fit, then re-run and verify all values
    eval.GetShapeCache().SetCapacity(4);
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
    eval.SetCommitFn([&](uint32_t nref_id, const Val& v) {
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
    eval.SetCommitFn([&](uint32_t nref_id, const Val& v) {
        return vs.commit(nref_id, v);
    });
    eval.SetRestoreFn([&](uint32_t vt_id) {
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

    // 3 shape nodes, capacity=2
    auto s1 = g.Const(1.0);
    auto m1 = g.Add("make_shape", {s1});
    auto s2 = g.Const(2.0);
    auto m2 = g.Add("make_shape", {s2});
    auto s3 = g.Const(3.0);
    auto m3 = g.Add("make_shape", {s3});

    MockVersionStore vs;
    Evaluator eval(reg);
    eval.GetShapeCache().SetCapacity(2);
    eval.SetCommitFn([&](uint32_t nref_id, const Val& v) {
        return vs.commit(nref_id, v);
    });
    eval.SetRestoreFn([&](uint32_t vt_id) {
        return vs.restore(vt_id);
    });

    std::shared_ptr<TopoNaming> tn;
    eval.Run(g, m3, tn);

    // m1 evicted from LRU but committed to version store
    REQUIRE(eval.GetShapeCache().Get(m1.id) == nullptr);
    REQUIRE(vs.store.size() == 3);

    // second run: m1 should be restored from version store, not re-evaluated
    eval.ResetStats();
    eval.Run(g, m3, tn);

    // m1 was restored (not re-evaluated), so cache misses should be 0
    // All 3 shape nodes hit the cache-hit path; m1 falls through to restore
    REQUIRE(eval.CacheMisses() == 0);
    REQUIRE(eval.CacheRestores() >= 1);

    // all results still correct
    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, m3)).tag == 3);
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
    eval.SetCommitFn([&](uint32_t nref_id, const Val& v) {
        return vs.commit(nref_id, v);
    });
    eval.SetRestoreFn([&](uint32_t vt_id) {
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
    eval.SetCommitFn([&](uint32_t nref_id, const Val& v) {
        return vs.commit(nref_id, v);
    });
    eval.SetRestoreFn([&](uint32_t vt_id) {
        return vs.restore(vt_id);
    });

    std::shared_ptr<TopoNaming> tn;
    eval.Run(g, mk, tn);

    auto* nd = g.Get(mk);
    REQUIRE(nd->vt_node_id != UINT32_MAX);

    // change the parameter -> invalidate
    g.UpdateImmediate(sz, Val(20.0));
    eval.Invalidate(g, sz);

    // mk's vt_node_id should be cleared (old shape is stale)
    REQUIRE(nd->vt_node_id == UINT32_MAX);

    // re-eval produces new shape with new vt_node_id
    eval.Run(g, mk, tn);
    REQUIRE(nd->vt_node_id != UINT32_MAX);
    REQUIRE(std::get<ShapeVal>(eval.ResolveVal(g, mk)).tag == 20);
}
