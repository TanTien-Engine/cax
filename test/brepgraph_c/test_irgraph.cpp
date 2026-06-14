#include <catch2/catch_test_macros.hpp>

#include "brepgraph_c/computation/CalcGraph.h"

using namespace brepgraph;

static void RegisterOps(OpRegistry& reg)
{
    reg.Define("add", {"a", "b"}, {},
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

// ---------------------------------------------------------------
//  IRGraph basic operations
// ---------------------------------------------------------------

TEST_CASE("IRGraph: Const creates nodes with correct types", "[irgraph]")
{
    OpRegistry reg;
    IRGraph g(reg);

    auto ci = g.Const(42);
    auto cd = g.Const(3.14);
    auto cb = g.Const(true);
    auto cv = g.Const(Vec3{1.0, 2.0, 3.0});

    REQUIRE(g.Get(ci) != nullptr);
    CHECK(g.Get(ci)->op_name == "$int");
    CHECK(std::get<int>(g.Get(ci)->imm) == 42);

    CHECK(g.Get(cd)->op_name == "$num");
    CHECK(std::get<double>(g.Get(cd)->imm) == 3.14);

    CHECK(g.Get(cb)->op_name == "$bool");
    CHECK(std::get<bool>(g.Get(cb)->imm) == true);

    CHECK(g.Get(cv)->op_name == "$vec3");
    auto v = std::get<Vec3>(g.Get(cv)->imm);
    CHECK(v[0] == 1.0);
    CHECK(v[1] == 2.0);
    CHECK(v[2] == 3.0);
}

TEST_CASE("IRGraph: Add creates operation nodes with inputs", "[irgraph]")
{
    OpRegistry reg;
    RegisterOps(reg);
    IRGraph g(reg);

    auto c1 = g.Const(1.0);
    auto c2 = g.Const(2.0);
    auto add = g.Add("add", {c1, c2});

    auto* nd = g.Get(add);
    REQUIRE(nd != nullptr);
    CHECK(nd->op_name == "add");
    CHECK(nd->inputs.size() == 2);
    CHECK(nd->inputs[0] == c1);
    CHECK(nd->inputs[1] == c2);
}

TEST_CASE("IRGraph: Kill marks node as dead", "[irgraph]")
{
    OpRegistry reg;
    IRGraph g(reg);

    auto c = g.Const(1);
    REQUIRE(g.Get(c) != nullptr);
    CHECK(g.LiveCount() == 1);

    g.Kill(c);
    CHECK(g.Get(c) == nullptr);
    CHECK(g.LiveCount() == 0);
}

TEST_CASE("IRGraph: Erase removes node entirely", "[irgraph]")
{
    OpRegistry reg;
    IRGraph g(reg);

    auto c = g.Const(1);
    g.Erase(c);
    CHECK(g.Get(c) == nullptr);
    CHECK(g.LiveCount() == 0);
}

TEST_CASE("IRGraph: Compact removes dead nodes", "[irgraph]")
{
    OpRegistry reg;
    IRGraph g(reg);

    auto c1 = g.Const(1);
    auto c2 = g.Const(2);
    auto c3 = g.Const(3);

    g.Kill(c2);
    CHECK(g.LiveCount() == 2);

    g.Compact();
    CHECK(g.LiveCount() == 2);
    CHECK(g.Get(c1) != nullptr);
    CHECK(g.Get(c2) == nullptr);
    CHECK(g.Get(c3) != nullptr);
}

TEST_CASE("IRGraph: UpdateImmediate changes value and marks node dirty", "[irgraph]")
{
    OpRegistry reg;
    IRGraph g(reg);

    auto c = g.Const(10.0);
    auto* nd = g.Get(c);
    nd->dirty = false;  // simulate post-eval clean state

    g.UpdateImmediate(c, Val(20.0));
    CHECK(std::get<double>(nd->imm) == 20.0);
    CHECK(nd->dirty);
}

TEST_CASE("IRGraph: ReplaceAllUses substitutes references", "[irgraph]")
{
    OpRegistry reg;
    RegisterOps(reg);
    IRGraph g(reg);

    auto c1 = g.Const(1.0);
    auto c2 = g.Const(2.0);
    auto c3 = g.Const(99.0);
    auto add = g.Add("add", {c1, c2});

    g.ReplaceAllUses(c1, c3);

    auto* nd = g.Get(add);
    CHECK(nd->inputs[0] == c3);
    CHECK(nd->inputs[1] == c2);
}

TEST_CASE("IRGraph: UsersOf returns nodes that reference the given node", "[irgraph]")
{
    OpRegistry reg;
    RegisterOps(reg);
    IRGraph g(reg);

    auto c = g.Const(1.0);
    auto mk1 = g.Add("make_shape", {c});
    auto mk2 = g.Add("make_shape", {c});

    auto users = g.UsersOf(c);
    CHECK(users.size() == 2);

    std::unordered_set<uint32_t> user_ids;
    for (auto& u : users) user_ids.insert(u.id);
    CHECK(user_ids.count(mk1.id));
    CHECK(user_ids.count(mk2.id));
}

TEST_CASE("IRGraph: TopoSort returns valid topological ordering", "[irgraph]")
{
    OpRegistry reg;
    RegisterOps(reg);
    IRGraph g(reg);

    auto c1 = g.Const(1.0);
    auto c2 = g.Const(2.0);
    auto add = g.Add("add", {c1, c2});
    auto mk = g.Add("make_shape", {add});

    auto order = g.TopoSort();
    REQUIRE(order.size() == 4);

    std::unordered_map<uint32_t, size_t> pos;
    for (size_t i = 0; i < order.size(); ++i)
        pos[order[i].id] = i;

    CHECK(pos[c1.id] < pos[add.id]);
    CHECK(pos[c2.id] < pos[add.id]);
    CHECK(pos[add.id] < pos[mk.id]);
}

TEST_CASE("IRGraph: empty graph operations", "[irgraph]")
{
    OpRegistry reg;
    IRGraph g(reg);

    CHECK(g.LiveCount() == 0);
    CHECK(g.TopoSort().empty());
    CHECK(g.TopoLevels().empty());
}

TEST_CASE("IRGraph: Clear removes all nodes", "[irgraph]")
{
    OpRegistry reg;
    IRGraph g(reg);

    g.Const(1);
    g.Const(2);
    CHECK(g.LiveCount() == 2);

    g.Clear();
    CHECK(g.LiveCount() == 0);
}

// ---------------------------------------------------------------
//  IRGraph: AssignOpIds
// ---------------------------------------------------------------

TEST_CASE("IRGraph: AssignOpIds assigns sequential ids to registered ops", "[irgraph]")
{
    OpRegistry reg;
    RegisterOps(reg);
    IRGraph g(reg);

    auto c1 = g.Const(10.0);
    auto c2 = g.Const(20.0);
    auto mk = g.Add("make_shape", {c1});
    auto tr = g.Add("transform", {mk, c2});

    g.AssignOpIds();

    CHECK(g.Get(c1)->op_id == UINT32_MAX);
    CHECK(g.Get(c2)->op_id == UINT32_MAX);
    CHECK(g.Get(mk)->op_id != UINT32_MAX);
    CHECK(g.Get(tr)->op_id != UINT32_MAX);
    CHECK(g.Get(mk)->op_id != g.Get(tr)->op_id);
}

TEST_CASE("IRGraph: AssignOpIds is deterministic", "[irgraph]")
{
    OpRegistry reg;
    RegisterOps(reg);

    auto build = [&]() {
        IRGraph g(reg);
        auto c = g.Const(1.0);
        auto mk = g.Add("make_shape", {c});
        auto off = g.Const(5.0);
        auto tr = g.Add("transform", {mk, off});
        g.AssignOpIds();
        return std::make_pair(g.Get(mk)->op_id, g.Get(tr)->op_id);
    };

    auto [mk1, tr1] = build();
    auto [mk2, tr2] = build();
    CHECK(mk1 == mk2);
    CHECK(tr1 == tr2);
}

// ---------------------------------------------------------------
//  OpRegistry
// ---------------------------------------------------------------

TEST_CASE("OpRegistry: Define and Find", "[registry]")
{
    OpRegistry reg;
    reg.Define("my_op", {"a", "b"}, {"c"},
        [](EvalCtx& ctx) -> Val { return 0; },
        {true, false, false, false});

    auto* desc = reg.Find("my_op");
    REQUIRE(desc != nullptr);
    CHECK(desc->name == "my_op");
    CHECK(desc->input_names.size() == 2);
    CHECK(desc->var_input_names.size() == 1);
    CHECK(desc->flags.is_dressup == true);
    CHECK(desc->flags.is_pattern == false);

    CHECK(reg.Find("nonexistent") == nullptr);
}

// ---------------------------------------------------------------
//  Optimizer: DCE
// ---------------------------------------------------------------

TEST_CASE("Optimizer: DCE removes unreferenced nodes", "[optimizer]")
{
    OpRegistry reg;
    RegisterOps(reg);
    IRGraph g(reg);

    auto c1 = g.Const(1.0);
    auto c2 = g.Const(2.0);
    auto c_dead = g.Const(99.0);
    auto add = g.Add("add", {c1, c2});

    CHECK(g.LiveCount() == 4);

    Optimizer::DCE(g);
    g.Compact();

    CHECK(g.LiveCount() == 3);
    CHECK(g.Get(c_dead) == nullptr);
    CHECK(g.Get(add) != nullptr);
}

TEST_CASE("Optimizer: DCE preserves root even with no users", "[optimizer]")
{
    OpRegistry reg;
    RegisterOps(reg);
    IRGraph g(reg);

    auto c = g.Const(1.0);
    auto mk = g.Add("make_shape", {c});

    Optimizer::DCE(g);
    g.Compact();

    CHECK(g.Get(mk) != nullptr);
    CHECK(g.LiveCount() == 2);
}

// ---------------------------------------------------------------
//  Optimizer: CSE
// ---------------------------------------------------------------

TEST_CASE("Optimizer: CSE eliminates common subexpressions", "[optimizer]")
{
    OpRegistry reg;
    RegisterOps(reg);
    IRGraph g(reg);

    auto c1 = g.Const(1.0);
    auto c2 = g.Const(2.0);
    auto add1 = g.Add("add", {c1, c2});
    auto add2 = g.Add("add", {c1, c2});

    size_t before = g.LiveCount();

    Optimizer::CSE(g);
    g.Compact();

    // One of the duplicate adds is eliminated
    CHECK(g.LiveCount() < before);
    // At least one add survives
    CHECK((g.Get(add1) != nullptr || g.Get(add2) != nullptr));
}

TEST_CASE("Optimizer: CSE does not merge consts of equal type but different value", "[optimizer]")
{
    OpRegistry reg;
    RegisterOps(reg);
    IRGraph g(reg);

    auto c1 = g.Const(1.0);
    auto c2 = g.Const(2.0);        // same type, different value -> must NOT merge
    auto a1 = g.Add("make_shape", {c1});
    auto a2 = g.Add("make_shape", {c2});
    auto root = g.Add("add", {a1, a2});  // keep both live

    Optimizer::CSE(g);
    g.Compact();

    CHECK(g.Get(c1) != nullptr);
    CHECK(g.Get(c2) != nullptr);
    CHECK(std::get<double>(g.Get(c1)->imm) == 1.0);
    CHECK(std::get<double>(g.Get(c2)->imm) == 2.0);
    CHECK(g.Get(a1) != nullptr);
    CHECK(g.Get(a2) != nullptr);
    (void)root;
}

TEST_CASE("Optimizer: CSE does not merge different ops", "[optimizer]")
{
    OpRegistry reg;
    RegisterOps(reg);
    IRGraph g(reg);

    auto c1 = g.Const(1.0);
    auto c2 = g.Const(2.0);
    auto add = g.Add("add", {c1, c2});
    auto mk = g.Add("make_shape", {c1});

    Optimizer::CSE(g);
    g.Compact();

    // add and mk have different ops, so both survive
    CHECK(g.Get(add) != nullptr);
    CHECK(g.Get(mk) != nullptr);
}

// ---------------------------------------------------------------
//  Optimizer: full Run
// ---------------------------------------------------------------

TEST_CASE("Optimizer: Run applies DCE + CSE together", "[optimizer]")
{
    OpRegistry reg;
    RegisterOps(reg);
    IRGraph g(reg);

    auto c1 = g.Const(1.0);
    auto c2 = g.Const(2.0);
    auto add1 = g.Add("add", {c1, c2});
    auto add2 = g.Add("add", {c1, c2});
    auto c_dead = g.Const(999.0);
    auto mk = g.Add("make_shape", {add1});

    size_t before = g.LiveCount();

    Optimizer opt;
    opt.Run(g);

    // c_dead and add2 should be removed (DCE + CSE)
    CHECK(g.Get(c_dead) == nullptr);
    CHECK(g.LiveCount() < before);
    CHECK(g.Get(mk) != nullptr);
}

// ---------------------------------------------------------------
//  MatchPred helpers
// ---------------------------------------------------------------

TEST_CASE("MatchPred: ByName matches op_name", "[optimizer]")
{
    IRNode nd;
    nd.op_name = "fillet";
    CHECK(ByName("fillet")(nd, nullptr));
    CHECK_FALSE(ByName("chamfer")(nd, nullptr));
}

TEST_CASE("MatchPred: ByFlag checks OpFlags", "[optimizer]")
{
    IRNode nd;
    OpDesc desc;
    desc.flags.is_dressup = true;

    CHECK(ByFlag([](const OpFlags& f) { return f.is_dressup; })(nd, &desc));
    CHECK_FALSE(ByFlag([](const OpFlags& f) { return f.is_pattern; })(nd, &desc));
}

TEST_CASE("MatchPred: Any always matches", "[optimizer]")
{
    IRNode nd;
    CHECK(Any()(nd, nullptr));
}

// ---------------------------------------------------------------
//  Optimizer: boolean_reassoc_cluster + pattern_boolean_fold
// ---------------------------------------------------------------

static void RegisterBoolPatternOps(OpRegistry& reg)
{
    auto noop = [](EvalCtx&) -> Val { return {}; };
    reg.Define("make_shape", {"size"},  {}, noop);
    reg.Define("selector",   {"shape"}, {}, noop);
    reg.Define("fuse", {"a", "b"}, {}, noop, {false, false, true, false});  // is_boolean
    reg.Define("cut",  {"a", "b"}, {}, noop, {false, false, true, false});  // is_boolean
    reg.Define("linear_pattern",
        {"shape", "dir1", "count1", "spacing1", "dir2", "count2", "spacing2"}, {},
        noop, {false, true, false, false});  // is_pattern
    reg.Define("feature_pattern",
        {"base", "tool", "op_kind", "dir1", "count1", "spacing1", "dir2", "count2", "spacing2"},
        {}, noop, {});
}

static NRef BuildLinearPattern(IRGraph& g, NRef seed,
                               const std::vector<NRef>& var = {})
{
    auto d1 = g.Const(Vec3{1, 0, 0});
    auto c1 = g.Const(2);
    auto s1 = g.Const(2.0);
    auto d2 = g.Const(Vec3{0, 1, 0});
    auto c2 = g.Const(1);
    auto s2 = g.Const(0.0);
    return g.Add("linear_pattern", {seed, d1, c1, s1, d2, c2, s2}, var);
}

TEST_CASE("Optimizer: cluster re-associates a fuse chain onto one boolean", "[optimizer][cluster]")
{
    OpRegistry reg; RegisterBoolPatternOps(reg);
    IRGraph g(reg);
    auto base = g.Add("make_shape", {g.Const(0.0)});
    auto t1   = g.Add("make_shape", {g.Const(1.0)});
    auto t2   = g.Add("make_shape", {g.Const(2.0)});
    auto t3   = g.Add("make_shape", {g.Const(3.0)});
    auto f1 = g.Add("fuse", {base, t1});
    auto f2 = g.Add("fuse", {f1, t2});
    auto f3 = g.Add("fuse", {f2, t3});

    Optimizer opt; opt.AddDefaultRules();
    opt.Run(g);

    // Root identity reused, still a fuse, base on the left.
    auto* root = g.Get(f3);
    REQUIRE(root != nullptr);
    CHECK(root->op_name == "fuse");
    REQUIRE(root->inputs.size() == 2);
    CHECK(root->inputs[0] == base);

    // The whole point: base now meets exactly ONE boolean.
    CHECK(g.UsersOf(base).size() == 1);

    // The other operand is a tool cluster independent of base.
    auto deps = g.CollectDeps(root->inputs[1]);
    CHECK(deps.count(base.id) == 0);
    CHECK(deps.count(t1.id) == 1);
    CHECK(deps.count(t2.id) == 1);
    CHECK(deps.count(t3.id) == 1);

    // Inner originals were reclaimed.
    CHECK(g.Get(f1) == nullptr);
    CHECK(g.Get(f2) == nullptr);
}

TEST_CASE("Optimizer: cluster re-associates a cut chain (tools unioned)", "[optimizer][cluster]")
{
    OpRegistry reg; RegisterBoolPatternOps(reg);
    IRGraph g(reg);
    auto base = g.Add("make_shape", {g.Const(0.0)});
    auto t1   = g.Add("make_shape", {g.Const(1.0)});
    auto t2   = g.Add("make_shape", {g.Const(2.0)});
    auto c1 = g.Add("cut", {base, t1});
    auto c2 = g.Add("cut", {c1, t2});

    Optimizer opt; opt.AddDefaultRules();
    opt.Run(g);

    auto* root = g.Get(c2);
    REQUIRE(root != nullptr);
    CHECK(root->op_name == "cut");
    REQUIRE(root->inputs.size() == 2);
    CHECK(root->inputs[0] == base);
    CHECK(g.UsersOf(base).size() == 1);

    // Cut tools are combined with a FUSE (cut by many == cut by their union).
    auto* cluster = g.Get(root->inputs[1]);
    REQUIRE(cluster != nullptr);
    CHECK(cluster->op_name == "fuse");
    auto deps = g.CollectDeps(root->inputs[1]);
    CHECK(deps.count(base.id) == 0);
    CHECK(deps.count(t1.id) == 1);
    CHECK(deps.count(t2.id) == 1);
}

TEST_CASE("Optimizer: cluster does NOT cross fuse/cut boundaries", "[optimizer][cluster]")
{
    OpRegistry reg; RegisterBoolPatternOps(reg);
    IRGraph g(reg);
    auto base = g.Add("make_shape", {g.Const(0.0)});
    auto t1   = g.Add("make_shape", {g.Const(1.0)});
    auto t2   = g.Add("make_shape", {g.Const(2.0)});
    auto inner = g.Add("cut",  {base, t1});
    auto outer = g.Add("fuse", {inner, t2});

    Optimizer opt; opt.AddDefaultRules();
    opt.Run(g);

    // Mixed-kind run is order-sensitive -> left untouched.
    auto* o = g.Get(outer);
    REQUIRE(o != nullptr);
    CHECK(o->op_name == "fuse");
    REQUIRE(o->inputs.size() == 2);
    CHECK(o->inputs[0] == inner);
    CHECK(o->inputs[1] == t2);
    auto* i = g.Get(inner);
    REQUIRE(i != nullptr);
    CHECK(i->op_name == "cut");
    CHECK(i->inputs[0] == base);
    CHECK(i->inputs[1] == t1);
}

TEST_CASE("Optimizer: cluster bails on a shared intermediate", "[optimizer][cluster]")
{
    OpRegistry reg; RegisterBoolPatternOps(reg);
    IRGraph g(reg);
    auto base = g.Add("make_shape", {g.Const(0.0)});
    auto t1   = g.Add("make_shape", {g.Const(1.0)});
    auto t2   = g.Add("make_shape", {g.Const(2.0)});
    auto inner = g.Add("fuse", {base, t1});
    auto outer = g.Add("fuse", {inner, t2});
    auto keep  = g.Add("make_shape", {inner});   // 2nd user of inner
    auto root  = g.Add("cut", {outer, keep});    // keeps everything live

    Optimizer opt; opt.AddDefaultRules();
    opt.Run(g);

    // inner has two users, so the (base.t1) fold must NOT be absorbed.
    auto* o = g.Get(outer);
    REQUIRE(o != nullptr);
    CHECK(o->op_name == "fuse");
    CHECK(o->inputs[0] == inner);
    CHECK(o->inputs[1] == t2);
    auto* i = g.Get(inner);
    REQUIRE(i != nullptr);
    CHECK(i->op_name == "fuse");
    CHECK(i->inputs[0] == base);
    CHECK(i->inputs[1] == t1);
}

TEST_CASE("Optimizer: folds fuse(base, linear_pattern) into feature_pattern", "[optimizer][fold]")
{
    OpRegistry reg; RegisterBoolPatternOps(reg);
    IRGraph g(reg);
    auto base = g.Add("make_shape", {g.Const(0.0)});
    auto seed = g.Add("make_shape", {g.Const(1.0)});
    auto lp   = BuildLinearPattern(g, seed);
    auto fz   = g.Add("fuse", {base, lp});

    Optimizer opt; opt.AddDefaultRules();
    opt.Run(g);

    auto* fp = g.Get(fz);
    REQUIRE(fp != nullptr);
    CHECK(fp->op_name == "feature_pattern");
    REQUIRE(fp->inputs.size() == 9);
    CHECK(fp->fixed_input_count == 9);
    CHECK(fp->inputs[0] == base);
    CHECK(fp->inputs[1] == seed);
    auto* kind = g.Get(fp->inputs[2]);
    REQUIRE(kind != nullptr);
    CHECK(kind->op_name == "$int");
    CHECK(std::get<int>(kind->imm) == 0);   // fuse -> boss
    CHECK(g.Get(lp) == nullptr);            // pattern absorbed
}

TEST_CASE("Optimizer: folds cut(base, linear_pattern) with op_kind=hole", "[optimizer][fold]")
{
    OpRegistry reg; RegisterBoolPatternOps(reg);
    IRGraph g(reg);
    auto base = g.Add("make_shape", {g.Const(0.0)});
    auto seed = g.Add("make_shape", {g.Const(1.0)});
    auto lp   = BuildLinearPattern(g, seed);
    auto ct   = g.Add("cut", {base, lp});

    Optimizer opt; opt.AddDefaultRules();
    opt.Run(g);

    auto* fp = g.Get(ct);
    REQUIRE(fp != nullptr);
    CHECK(fp->op_name == "feature_pattern");
    auto* kind = g.Get(fp->inputs[2]);
    REQUIRE(kind != nullptr);
    CHECK(std::get<int>(kind->imm) == 1);   // cut -> hole
}

TEST_CASE("Optimizer: fold bails on a selector-bearing pattern", "[optimizer][fold]")
{
    OpRegistry reg; RegisterBoolPatternOps(reg);
    IRGraph g(reg);
    auto base = g.Add("make_shape", {g.Const(0.0)});
    auto seed = g.Add("make_shape", {g.Const(1.0)});
    auto sel  = g.Add("selector", {seed});
    auto lp   = BuildLinearPattern(g, seed, {sel});   // variadic selector edge
    auto fz   = g.Add("fuse", {base, lp});

    Optimizer opt; opt.AddDefaultRules();
    opt.Run(g);

    // Variadic selectors can't be reordered through the pattern -> no fold.
    auto* node = g.Get(fz);
    REQUIRE(node != nullptr);
    CHECK(node->op_name == "fuse");
    CHECK(g.Get(lp) != nullptr);
}

TEST_CASE("Optimizer: fold bails when the pattern is shared", "[optimizer][fold]")
{
    OpRegistry reg; RegisterBoolPatternOps(reg);
    IRGraph g(reg);
    auto base1 = g.Add("make_shape", {g.Const(0.0)});
    auto base2 = g.Add("make_shape", {g.Const(9.0)});
    auto seed  = g.Add("make_shape", {g.Const(1.0)});
    auto lp    = BuildLinearPattern(g, seed);
    auto fz1   = g.Add("fuse", {base1, lp});
    auto fz2   = g.Add("fuse", {base2, lp});
    auto root  = g.Add("cut", {fz1, fz2});    // keep both live (cut root: A bails)

    Optimizer opt; opt.AddDefaultRules();
    opt.Run(g);

    // Pattern feeds two booleans -> can't be absorbed into either.
    CHECK(g.Get(lp) != nullptr);
    auto* a = g.Get(fz1);
    auto* b = g.Get(fz2);
    REQUIRE(a != nullptr);
    REQUIRE(b != nullptr);
    CHECK(a->op_name == "fuse");
    CHECK(b->op_name == "fuse");
}

TEST_CASE("Optimizer: roots-aware DCE preserves all listed outputs", "[optimizer][dce]")
{
    OpRegistry reg; RegisterBoolPatternOps(reg);
    IRGraph g(reg);
    auto o1   = g.Add("make_shape", {g.Const(1.0)});
    auto o2   = g.Add("make_shape", {g.Const(2.0)});
    auto dead = g.Add("make_shape", {g.Const(9.0)});

    // Single-root DCE keeps only order.back(); roots-aware keeps BOTH o1 + o2.
    Optimizer::DCE(g, {o1, o2});
    g.Compact();

    CHECK(g.Get(o1)   != nullptr);
    CHECK(g.Get(o2)   != nullptr);
    CHECK(g.Get(dead) == nullptr);
}

TEST_CASE("Optimizer: roots-aware Run clusters a chain and keeps a second output", "[optimizer][cluster]")
{
    OpRegistry reg; RegisterBoolPatternOps(reg);
    IRGraph g(reg);
    auto base = g.Add("make_shape", {g.Const(0.0)});
    auto t1   = g.Add("make_shape", {g.Const(1.0)});
    auto t2   = g.Add("make_shape", {g.Const(2.0)});
    auto t3   = g.Add("make_shape", {g.Const(3.0)});
    auto f1 = g.Add("fuse", {base, t1});
    auto f2 = g.Add("fuse", {f1, t2});
    auto f3 = g.Add("fuse", {f2, t3});
    auto sep = g.Add("make_shape", {g.Const(7.0)});  // independent 2nd output

    Optimizer opt; opt.AddDefaultRules();
    opt.Run(g, {f3, sep});   // multi-output: both f3 and sep are live

    // The chain is re-associated onto one boolean...
    auto* root = g.Get(f3);
    REQUIRE(root != nullptr);
    CHECK(root->op_name == "fuse");
    REQUIRE(root->inputs.size() == 2);
    CHECK(root->inputs[0] == base);
    CHECK(g.UsersOf(base).size() == 1);
    auto deps = g.CollectDeps(root->inputs[1]);
    CHECK(deps.count(base.id) == 0);

    // ...while the second output survives (single-root Run would drop it).
    CHECK(g.Get(sep) != nullptr);
}
