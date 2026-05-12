#include <catch2/catch_test_macros.hpp>

#include "CompGraph.h"

using namespace breptopo;

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

TEST_CASE("IRGraph: UpdateImmediate changes value and bumps version", "[irgraph]")
{
    OpRegistry reg;
    IRGraph g(reg);

    auto c = g.Const(10.0);
    auto* nd = g.Get(c);
    uint64_t v0 = nd->version;

    g.UpdateImmediate(c, Val(20.0));
    CHECK(std::get<double>(nd->imm) == 20.0);
    CHECK(nd->version == v0 + 1);
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
