#include <catch2/catch_test_macros.hpp>

#include "CompGraph.h"

#include <memory>

using namespace breptopo;

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

    reg.Define("fuse", {"shape1", "shape2"}, {},
        [](EvalCtx& ctx) -> Val {
            auto s1 = ctx.GetShape(0);
            auto s2 = ctx.GetShape(1);
            ShapeVal out;
            out.shape = nullptr;
            out.tag = s1.tag + s2.tag;
            return out;
        },
        {false, false, true, false});
}

// Simulate CompGraph::RebuildIR + AppendNewSteps at the raw IR level.
// This lets us test incrementality without OCCT dependencies.

struct TestGraph
{
    OpRegistry reg;
    OpHistory history;
    IRGraph ir;
    Evaluator eval;

    struct NodeMeta { NRef ref; std::string desc; };
    std::vector<NodeMeta> nodes;
    std::unordered_map<uint32_t, int> ref2ext;

    bool lowered = false;
    size_t lowered_count = 0;

    TestGraph() : ir(reg), eval(reg)
    {
        RegisterTestOps(reg);
    }

    int Register(NRef ref, const std::string& desc)
    {
        int ext = static_cast<int>(nodes.size());
        nodes.push_back({ref, desc});
        ref2ext[ref.id] = ext;
        return ext;
    }

    int AddConst(double v, const std::string& desc)
    {
        lowered = false;
        return history.AddConst(v, desc);
    }

    int AddOp(const std::string& op,
              const std::vector<int>& inputs,
              const std::vector<int>& var_inputs = {},
              const std::string& desc = "")
    {
        lowered = false;
        return history.AddOp(op, inputs, var_inputs, desc);
    }

    void UpdateConst(int ext_id, Val v)
    {
        history.UpdateConst(ext_id, v);
        if (lowered && ext_id >= 0 && ext_id < (int)nodes.size())
        {
            NRef ref = nodes[ext_id].ref;
            ir.UpdateImmediate(ref, v);
            eval.Invalidate(ir, ref);
        }
        else
        {
            lowered = false;
        }
    }

    void AppendNewSteps()
    {
        auto& steps = history.Steps();
        for (size_t i = lowered_count; i < steps.size(); ++i)
        {
            auto& step = steps[i];
            if (!step.op_name.empty() && step.op_name[0] == '$')
            {
                NRef ref = NREF_NULL;
                if (step.op_name == "$num") ref = ir.Const(std::get<double>(step.imm));
                Register(ref, step.desc);
            }
            else
            {
                std::vector<NRef> refs;
                for (int j : step.inputs)
                    refs.push_back((j >= 0 && j < (int)nodes.size()) ? nodes[j].ref : NREF_NULL);
                std::vector<NRef> var_refs;
                for (int j : step.var_inputs)
                    var_refs.push_back((j >= 0 && j < (int)nodes.size()) ? nodes[j].ref : NREF_NULL);
                Register(ir.Add(step.op_name, refs, var_refs), step.desc);
            }
        }
        lowered_count = steps.size();
    }

    void RebuildIR()
    {
        ir.Clear();
        nodes.clear();
        ref2ext.clear();
        lowered_count = 0;
        AppendNewSteps();
    }

    void Lower()
    {
        if (lowered) return;
        if (lowered_count > 0)
            AppendNewSteps();
        else
            RebuildIR();
        lowered = true;
    }

    void Truncate(size_t keep)
    {
        if (keep >= history.Size()) return;

        if (lowered && keep <= lowered_count)
        {
            for (size_t i = keep; i < nodes.size(); ++i)
            {
                NRef ref = nodes[i].ref;
                eval.GetShapeCache().Remove(ref.id);
                ir.Erase(ref);
                ref2ext.erase(ref.id);
            }
            nodes.resize(keep);
            lowered_count = keep;
        }
        else
        {
            lowered_count = 0;
        }

        history.Truncate(keep);
        lowered = false;
    }

    Val Eval(int ext_id)
    {
        Lower();
        if (ext_id < 0 || ext_id >= (int)nodes.size()) return {};
        NRef ref = nodes[ext_id].ref;
        std::shared_ptr<TopoNaming> tn;
        eval.Run(ir, ref, tn);
        return eval.ResolveVal(ir, ref);
    }
};

// ---------------------------------------------------------------
//  Incremental lower tests
// ---------------------------------------------------------------

TEST_CASE("IncrementalLower: append preserves existing NRef IDs", "[incremental]")
{
    TestGraph g;

    int c1 = g.AddConst(10.0, "c1");
    int c2 = g.AddConst(20.0, "c2");
    int add = g.AddOp("add_nums", {c1, c2}, {}, "add");

    g.Lower();
    NRef ref_c1 = g.nodes[c1].ref;
    NRef ref_c2 = g.nodes[c2].ref;
    NRef ref_add = g.nodes[add].ref;
    REQUIRE(ref_c1.valid());
    REQUIRE(ref_c2.valid());
    REQUIRE(ref_add.valid());

    // append a new node
    int c3 = g.AddConst(30.0, "c3");
    int add2 = g.AddOp("add_nums", {add, c3}, {}, "add2");

    g.Lower();

    // old NRef IDs must be unchanged
    REQUIRE(g.nodes[c1].ref == ref_c1);
    REQUIRE(g.nodes[c2].ref == ref_c2);
    REQUIRE(g.nodes[add].ref == ref_add);

    // new nodes have valid refs
    REQUIRE(g.nodes[c3].ref.valid());
    REQUIRE(g.nodes[add2].ref.valid());
}

TEST_CASE("IncrementalLower: cache survives incremental append", "[incremental]")
{
    TestGraph g;

    int sz = g.AddConst(10.0, "sz");
    int mk = g.AddOp("make_shape", {sz}, {}, "mk");

    Val v1 = g.Eval(mk);
    REQUIRE(std::holds_alternative<ShapeVal>(v1));
    REQUIRE(std::get<ShapeVal>(v1).tag == 10);
    REQUIRE(g.eval.CacheMisses() == 1);

    // append a downstream node
    int off = g.AddConst(5.0, "off");
    int tr = g.AddOp("transform", {mk, off}, {}, "tr");

    g.eval.ResetStats();
    Val v2 = g.Eval(tr);

    // mk should be a cache hit (not re-evaluated), tr is new (miss)
    REQUIRE(std::holds_alternative<ShapeVal>(v2));
    REQUIRE(std::get<ShapeVal>(v2).tag == 15);
    REQUIRE(g.eval.CacheHits() >= 1);
}

TEST_CASE("IncrementalLower: full rebuild matches incremental result", "[incremental]")
{
    // Build the same graph two ways: full rebuild vs incremental, compare results.
    auto build_full = []() -> Val {
        TestGraph g;
        int c1 = g.AddConst(10.0, "c1");
        int mk = g.AddOp("make_shape", {c1}, {}, "mk");
        int c2 = g.AddConst(5.0, "c2");
        int tr = g.AddOp("transform", {mk, c2}, {}, "tr");
        int c3 = g.AddConst(3.0, "c3");
        int mk2 = g.AddOp("make_shape", {c3}, {}, "mk2");
        int fuse = g.AddOp("fuse", {tr, mk2}, {}, "fuse");
        return g.Eval(fuse);
    };

    auto build_incremental = []() -> Val {
        TestGraph g;
        int c1 = g.AddConst(10.0, "c1");
        int mk = g.AddOp("make_shape", {c1}, {}, "mk");
        g.Eval(mk);  // force first lower

        int c2 = g.AddConst(5.0, "c2");
        int tr = g.AddOp("transform", {mk, c2}, {}, "tr");
        g.Eval(tr);  // incremental lower

        int c3 = g.AddConst(3.0, "c3");
        int mk2 = g.AddOp("make_shape", {c3}, {}, "mk2");
        int fuse = g.AddOp("fuse", {tr, mk2}, {}, "fuse");
        return g.Eval(fuse);  // another incremental lower
    };

    Val full = build_full();
    Val incr = build_incremental();

    REQUIRE(std::holds_alternative<ShapeVal>(full));
    REQUIRE(std::holds_alternative<ShapeVal>(incr));
    REQUIRE(std::get<ShapeVal>(full).tag == std::get<ShapeVal>(incr).tag);
}

TEST_CASE("IncrementalLower: UpdateConst still works after incremental append", "[incremental]")
{
    TestGraph g;

    int sz = g.AddConst(10.0, "sz");
    int mk = g.AddOp("make_shape", {sz}, {}, "mk");
    g.Eval(mk);

    // incremental append
    int off = g.AddConst(5.0, "off");
    int tr = g.AddOp("transform", {mk, off}, {}, "tr");
    Val v1 = g.Eval(tr);
    REQUIRE(std::get<ShapeVal>(v1).tag == 15);

    // update the original constant
    g.UpdateConst(sz, Val(20.0));
    Val v2 = g.Eval(tr);
    REQUIRE(std::get<ShapeVal>(v2).tag == 25);
}

TEST_CASE("IncrementalLower: multiple incremental rounds", "[incremental]")
{
    TestGraph g;

    // round 1
    int c1 = g.AddConst(1.0, "c1");
    int mk1 = g.AddOp("make_shape", {c1}, {}, "mk1");
    g.Eval(mk1);

    NRef ref_mk1 = g.nodes[mk1].ref;

    // round 2
    int c2 = g.AddConst(2.0, "c2");
    int mk2 = g.AddOp("make_shape", {c2}, {}, "mk2");
    g.Eval(mk2);

    NRef ref_mk2 = g.nodes[mk2].ref;

    // round 3
    int fuse = g.AddOp("fuse", {mk1, mk2}, {}, "fuse");
    Val v = g.Eval(fuse);

    // all old refs preserved
    REQUIRE(g.nodes[mk1].ref == ref_mk1);
    REQUIRE(g.nodes[mk2].ref == ref_mk2);

    REQUIRE(std::holds_alternative<ShapeVal>(v));
    REQUIRE(std::get<ShapeVal>(v).tag == 3);  // 1 + 2
}

TEST_CASE("IncrementalLower: lowered_count tracks correctly", "[incremental]")
{
    TestGraph g;

    REQUIRE(g.lowered_count == 0);

    int c1 = g.AddConst(10.0, "c1");
    int c2 = g.AddConst(20.0, "c2");
    g.Lower();
    REQUIRE(g.lowered_count == 2);

    int add = g.AddOp("add_nums", {c1, c2}, {}, "add");
    g.Lower();
    REQUIRE(g.lowered_count == 3);

    int c3 = g.AddConst(30.0, "c3");
    int c4 = g.AddConst(40.0, "c4");
    int add2 = g.AddOp("add_nums", {c3, c4}, {}, "add2");
    g.Lower();
    REQUIRE(g.lowered_count == 6);
}

TEST_CASE("IncrementalLower: new node referencing old node evaluates correctly", "[incremental]")
{
    TestGraph g;

    // phase 1: build and eval a shape
    int sz = g.AddConst(42.0, "sz");
    int mk = g.AddOp("make_shape", {sz}, {}, "mk");
    Val v1 = g.Eval(mk);
    REQUIRE(std::get<ShapeVal>(v1).tag == 42);

    // phase 2: add transform referencing the existing shape
    int off1 = g.AddConst(10.0, "off1");
    int tr1 = g.AddOp("transform", {mk, off1}, {}, "tr1");

    // phase 3: add another transform chained to the first new one
    int off2 = g.AddConst(3.0, "off2");
    int tr2 = g.AddOp("transform", {tr1, off2}, {}, "tr2");

    Val v2 = g.Eval(tr2);
    REQUIRE(std::get<ShapeVal>(v2).tag == 55);  // 42 + 10 + 3
}

TEST_CASE("IncrementalLower: IR node count grows incrementally", "[incremental]")
{
    TestGraph g;

    int c1 = g.AddConst(1.0, "c1");
    int mk = g.AddOp("make_shape", {c1}, {}, "mk");
    g.Lower();
    size_t count1 = g.ir.LiveCount();
    REQUIRE(count1 == 2);

    int c2 = g.AddConst(5.0, "off");
    int tr = g.AddOp("transform", {mk, c2}, {}, "tr");
    g.Lower();
    size_t count2 = g.ir.LiveCount();
    REQUIRE(count2 == 4);  // 2 old + 2 new
}

// ---------------------------------------------------------------
//  Truncate + replace tests
// ---------------------------------------------------------------

TEST_CASE("Truncate: removes tail nodes from IR", "[truncate]")
{
    TestGraph g;

    int c1 = g.AddConst(10.0, "c1");
    int mk = g.AddOp("make_shape", {c1}, {}, "mk");
    int off = g.AddConst(5.0, "off");
    int tr = g.AddOp("transform", {mk, off}, {}, "tr");

    g.Eval(tr);
    REQUIRE(g.ir.LiveCount() == 4);

    // truncate: keep only c1 and mk
    g.Truncate(2);
    REQUIRE(g.ir.LiveCount() == 2);
    REQUIRE(g.nodes.size() == 2);
    REQUIRE(g.lowered_count == 2);

    // old refs still valid
    REQUIRE(g.ir.Get(g.nodes[c1].ref) != nullptr);
    REQUIRE(g.ir.Get(g.nodes[mk].ref) != nullptr);
}

TEST_CASE("Truncate: preserves upstream cache", "[truncate]")
{
    TestGraph g;

    int c1 = g.AddConst(10.0, "c1");
    int mk = g.AddOp("make_shape", {c1}, {}, "mk");
    int off = g.AddConst(5.0, "off");
    int tr = g.AddOp("transform", {mk, off}, {}, "tr");

    g.Eval(tr);
    REQUIRE(std::get<ShapeVal>(g.eval.ResolveVal(g.ir, g.nodes[mk].ref)).tag == 10);

    // truncate removes tr and off, but mk's cache stays
    g.Truncate(2);
    REQUIRE(g.eval.GetShapeCache().Get(g.nodes[mk].ref.id) != nullptr);

    // re-eval mk should hit cache
    g.eval.ResetStats();
    Val v = g.Eval(mk);
    REQUIRE(std::get<ShapeVal>(v).tag == 10);
    REQUIRE(g.eval.CacheHits() >= 1);
    REQUIRE(g.eval.CacheMisses() == 0);
}

TEST_CASE("Truncate: replace tail with different ops", "[truncate]")
{
    TestGraph g;

    // original: c1(10) -> make_shape -> transform(+5) = 15
    int c1 = g.AddConst(10.0, "c1");
    int mk = g.AddOp("make_shape", {c1}, {}, "mk");
    int off = g.AddConst(5.0, "off");
    int tr = g.AddOp("transform", {mk, off}, {}, "tr");

    Val v1 = g.Eval(tr);
    REQUIRE(std::get<ShapeVal>(v1).tag == 15);

    // truncate after mk, replace with transform(+100) = 110
    g.Truncate(2);

    int off2 = g.AddConst(100.0, "off2");
    int tr2 = g.AddOp("transform", {mk, off2}, {}, "tr2");

    Val v2 = g.Eval(tr2);
    REQUIRE(std::get<ShapeVal>(v2).tag == 110);

    // mk should not have been re-evaluated
    g.eval.ResetStats();
    g.Eval(tr2);
    REQUIRE(g.eval.CacheHits() >= 1);
}

TEST_CASE("Truncate: replace middle of pipeline", "[truncate]")
{
    TestGraph g;

    // c1(10) -> mk -> tr(+5) -> tr2(+3) = 18
    int c1 = g.AddConst(10.0, "c1");
    int mk = g.AddOp("make_shape", {c1}, {}, "mk");
    int off1 = g.AddConst(5.0, "off1");
    int tr1 = g.AddOp("transform", {mk, off1}, {}, "tr1");
    int off2 = g.AddConst(3.0, "off2");
    int tr2 = g.AddOp("transform", {tr1, off2}, {}, "tr2");

    Val v1 = g.Eval(tr2);
    REQUIRE(std::get<ShapeVal>(v1).tag == 18);

    NRef ref_mk = g.nodes[mk].ref;

    // truncate after mk, rebuild differently: mk -> fuse(mk, new_shape(20))
    g.Truncate(2);

    int c2 = g.AddConst(20.0, "c2");
    int mk2 = g.AddOp("make_shape", {c2}, {}, "mk2");
    int fuse = g.AddOp("fuse", {mk, mk2}, {}, "fuse");

    Val v2 = g.Eval(fuse);
    REQUIRE(std::get<ShapeVal>(v2).tag == 30);  // 10 + 20

    // original mk ref preserved
    REQUIRE(g.nodes[mk].ref == ref_mk);
}

TEST_CASE("Truncate: truncate to zero then rebuild", "[truncate]")
{
    TestGraph g;

    int c1 = g.AddConst(10.0, "c1");
    int mk = g.AddOp("make_shape", {c1}, {}, "mk");
    g.Eval(mk);

    g.Truncate(0);
    REQUIRE(g.ir.LiveCount() == 0);
    REQUIRE(g.nodes.empty());
    REQUIRE(g.lowered_count == 0);

    // rebuild from scratch
    int c2 = g.AddConst(42.0, "c2");
    int mk2 = g.AddOp("make_shape", {c2}, {}, "mk2");
    Val v = g.Eval(mk2);
    REQUIRE(std::get<ShapeVal>(v).tag == 42);
}

TEST_CASE("Truncate: multiple truncate-and-replace cycles", "[truncate]")
{
    TestGraph g;

    // cycle 1: c1(10) -> mk
    int c1 = g.AddConst(10.0, "c1");
    int mk = g.AddOp("make_shape", {c1}, {}, "mk");
    g.Eval(mk);
    NRef ref_c1 = g.nodes[c1].ref;

    // cycle 2: truncate after c1, replace mk with transform
    g.Truncate(1);
    // c1 still valid, but we need a shape to transform
    // so add a make_shape then transform
    int mk2 = g.AddOp("make_shape", {c1}, {}, "mk2");
    int off = g.AddConst(7.0, "off");
    int tr = g.AddOp("transform", {mk2, off}, {}, "tr");
    Val v1 = g.Eval(tr);
    REQUIRE(std::get<ShapeVal>(v1).tag == 17);
    REQUIRE(g.nodes[c1].ref == ref_c1);

    // cycle 3: truncate after mk2, replace tr with fuse
    g.Truncate(2);
    int c3 = g.AddConst(3.0, "c3");
    int mk3 = g.AddOp("make_shape", {c3}, {}, "mk3");
    int fuse = g.AddOp("fuse", {mk2, mk3}, {}, "fuse");
    Val v2 = g.Eval(fuse);
    REQUIRE(std::get<ShapeVal>(v2).tag == 13);  // 10 + 3
}

TEST_CASE("Truncate: removed nodes' cache entries are cleaned up", "[truncate]")
{
    TestGraph g;

    int c1 = g.AddConst(10.0, "c1");
    int mk = g.AddOp("make_shape", {c1}, {}, "mk");
    int off = g.AddConst(5.0, "off");
    int tr = g.AddOp("transform", {mk, off}, {}, "tr");

    g.Eval(tr);

    NRef ref_tr = g.nodes[tr].ref;
    REQUIRE(g.eval.GetShapeCache().Get(ref_tr.id) != nullptr);

    g.Truncate(2);

    // tr's cache entry should be gone
    REQUIRE(g.eval.GetShapeCache().Get(ref_tr.id) == nullptr);
    // tr's IR node should be gone
    REQUIRE(g.ir.Get(ref_tr) == nullptr);
}

TEST_CASE("Truncate: noop when keep >= history size", "[truncate]")
{
    TestGraph g;

    int c1 = g.AddConst(10.0, "c1");
    int mk = g.AddOp("make_shape", {c1}, {}, "mk");
    g.Eval(mk);

    size_t before = g.ir.LiveCount();
    g.Truncate(5);  // keep > size, should be noop
    REQUIRE(g.ir.LiveCount() == before);
    g.Truncate(2);  // keep == size, should be noop
    REQUIRE(g.ir.LiveCount() == before);
}
