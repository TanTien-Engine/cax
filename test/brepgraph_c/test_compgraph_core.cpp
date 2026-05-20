#include <catch2/catch_test_macros.hpp>

#include "CompGraph.h"

using namespace brepgraph;

// ---------------------------------------------------------------
//  OpHistory additional tests
// ---------------------------------------------------------------

TEST_CASE("OpHistory: UpdateConst modifies existing step", "[ophistory]")
{
    OpHistory h;
    int s0 = h.AddConst(10.0, "width");

    h.UpdateConst(s0, Val(20.0));

    auto* step = h.Get(s0);
    REQUIRE(step != nullptr);
    CHECK(std::get<double>(step->imm) == 20.0);
}

TEST_CASE("OpHistory: UpdateConst with out-of-range id is safe", "[ophistory]")
{
    OpHistory h;
    h.AddConst(1.0, "x");

    h.UpdateConst(-1, Val(99.0));
    h.UpdateConst(100, Val(99.0));

    CHECK(h.Size() == 1);
    CHECK(std::get<double>(h.Get(0)->imm) == 1.0);
}

TEST_CASE("OpHistory: Truncate reduces size", "[ophistory]")
{
    OpHistory h;
    h.AddConst(1.0, "a");
    h.AddConst(2.0, "b");
    h.AddConst(3.0, "c");
    CHECK(h.Size() == 3);

    h.Truncate(2);
    CHECK(h.Size() == 2);
    CHECK(h.Get(2) == nullptr);
    CHECK(h.Get(1) != nullptr);
}

TEST_CASE("OpHistory: Truncate beyond size is a no-op", "[ophistory]")
{
    OpHistory h;
    h.AddConst(1.0, "a");

    h.Truncate(10);
    CHECK(h.Size() == 1);
}

TEST_CASE("OpHistory: Get returns nullptr for invalid ids", "[ophistory]")
{
    OpHistory h;
    h.AddConst(1, "x");

    CHECK(h.Get(-1) == nullptr);
    CHECK(h.Get(1) == nullptr);
    CHECK(h.Get(100) == nullptr);
}

TEST_CASE("OpHistory: AddOp with var_inputs round-trips", "[ophistory]")
{
    OpHistory h1;
    int s0 = h1.AddConst(1.0, "a");
    int s1 = h1.AddConst(2.0, "b");
    int s2 = h1.AddConst(3.0, "c");
    int s3 = h1.AddOp("fillet", {s0}, {s1, s2}, "fillet op");

    uint8_t* buf = nullptr;
    uint32_t len = 0;
    h1.StoreToByteArray(&buf, len);

    OpHistory h2;
    REQUIRE(h2.LoadFromByteArray(buf, len));
    delete[] buf;

    auto* step = h2.Get(s3);
    REQUIRE(step != nullptr);
    CHECK(step->op_name == "fillet");
    CHECK(step->inputs.size() == 1);
    CHECK(step->inputs[0] == s0);
    CHECK(step->var_inputs.size() == 2);
    CHECK(step->var_inputs[0] == s1);
    CHECK(step->var_inputs[1] == s2);
    CHECK(step->desc == "fillet op");
}

TEST_CASE("OpHistory: LoadFromByteArray rejects bad magic", "[ophistory]")
{
    uint8_t bad_data[12] = {0};

    OpHistory h;
    CHECK_FALSE(h.LoadFromByteArray(bad_data, 12));
}

TEST_CASE("OpHistory: LoadFromByteArray rejects truncated data", "[ophistory]")
{
    OpHistory h1;
    h1.AddConst(42.0, "x");

    uint8_t* buf = nullptr;
    uint32_t len = 0;
    h1.StoreToByteArray(&buf, len);

    OpHistory h2;
    CHECK_FALSE(h2.LoadFromByteArray(buf, 12));
    CHECK(h2.Size() == 0);
    delete[] buf;
}

TEST_CASE("OpHistory: Clear removes all steps", "[ophistory]")
{
    OpHistory h;
    h.AddConst(1, "a");
    h.AddConst(2, "b");
    CHECK(h.Size() == 2);

    h.Clear();
    CHECK(h.Size() == 0);
}

// ---------------------------------------------------------------
//  OpHistory: step_id assignment
// ---------------------------------------------------------------

TEST_CASE("OpHistory: step_ids are sequential starting from 0", "[ophistory]")
{
    OpHistory h;
    int s0 = h.AddConst(1.0, "a");
    int s1 = h.AddConst(2.0, "b");
    int s2 = h.AddOp("op", {s0, s1}, {}, "c");

    CHECK(s0 == 0);
    CHECK(s1 == 1);
    CHECK(s2 == 2);

    CHECK(h.Get(s0)->step_id == 0);
    CHECK(h.Get(s1)->step_id == 1);
    CHECK(h.Get(s2)->step_id == 2);
}

// ---------------------------------------------------------------
//  EvalCtx
// ---------------------------------------------------------------

TEST_CASE("EvalCtx: type conversions", "[evalctx]")
{
    std::vector<Val> inputs = {Val(42), Val(3.14), Val(true), Val(Vec3{1,2,3})};
    EvalCtx ctx{inputs, inputs.size(), nullptr};

    CHECK(ctx.Int(0) == 42);
    CHECK(ctx.Num(0) == 42.0);
    CHECK(ctx.Num(1) == 3.14);
    CHECK(ctx.Int(1) == 3);
    CHECK(ctx.Bool(2) == true);
    auto v = ctx.GetVec3(3);
    CHECK(v[0] == 1.0);
    CHECK(v[1] == 2.0);
    CHECK(v[2] == 3.0);
}

TEST_CASE("EvalCtx: out-of-range returns defaults", "[evalctx]")
{
    std::vector<Val> inputs;
    EvalCtx ctx{inputs, 0, nullptr};

    CHECK(ctx.Num(0) == 0.0);
    CHECK(ctx.Int(0) == 0);
    CHECK(ctx.Bool(0) == false);
    auto v = ctx.GetVec3(0);
    CHECK(v[0] == 0.0);
    CHECK(v[1] == 0.0);
    CHECK(v[2] == 0.0);
}

TEST_CASE("EvalCtx: VarShapes extracts shapes from variadic portion", "[evalctx]")
{
    ShapeVal sv1; sv1.shape = nullptr; sv1.tag = 1;
    ShapeVal sv2; sv2.shape = nullptr; sv2.tag = 2;

    // inputs holds fixed + variadic concatenated; fixed_count = 0 here, so
    // all three values are part of the variadic slice.
    std::vector<Val> inputs = {Val(sv1), Val(42), Val(sv2)};
    EvalCtx ctx{inputs, 0, nullptr};

    auto shapes = ctx.VarShapes();
    CHECK(shapes.size() == 2);
    CHECK(shapes[0].tag == 1);
    CHECK(shapes[1].tag == 2);
}

// ---------------------------------------------------------------
//  NRef
// ---------------------------------------------------------------

TEST_CASE("NRef: validity and equality", "[nref]")
{
    NRef a{0};
    NRef b{1};
    NRef c{1};

    CHECK_FALSE(a.valid());
    CHECK(b.valid());
    CHECK(b == c);
    CHECK(a != b);
}

// ---------------------------------------------------------------
//  IRGraph: Dump
// ---------------------------------------------------------------

TEST_CASE("IRGraph: Dump produces non-empty output", "[irgraph]")
{
    OpRegistry reg;
    reg.Define("add", {"a", "b"}, {},
        [](EvalCtx& ctx) -> Val { return ctx.Num(0) + ctx.Num(1); });

    IRGraph g(reg);
    auto c1 = g.Const(1.0);
    auto c2 = g.Const(2.0);
    g.Add("add", {c1, c2});

    auto dump = g.Dump();
    CHECK_FALSE(dump.empty());
    CHECK(dump.find("$num") != std::string::npos);
    CHECK(dump.find("add") != std::string::npos);
}

// ---------------------------------------------------------------
//  IRGraph: var_inputs in Add
// ---------------------------------------------------------------

TEST_CASE("IRGraph: Add with var_inputs", "[irgraph]")
{
    OpRegistry reg;
    reg.Define("multi", {"a"}, {"extras"},
        [](EvalCtx& ctx) -> Val { return 0; });

    IRGraph g(reg);
    auto c1 = g.Const(1.0);
    auto c2 = g.Const(2.0);
    auto c3 = g.Const(3.0);
    auto nd = g.Add("multi", {c1}, {c2, c3});

    auto* node = g.Get(nd);
    // inputs holds fixed followed by variadic, total = 1 + 2 = 3
    CHECK(node->inputs.size() == 3);
    CHECK(node->fixed_input_count == 1);
    CHECK(node->inputs[0] == c1);
    CHECK(node->inputs[1] == c2);
    CHECK(node->inputs[2] == c3);
}

// ---------------------------------------------------------------
//  ShapeVal
// ---------------------------------------------------------------

TEST_CASE("ShapeVal: equality by tag", "[shapeval]")
{
    ShapeVal a; a.shape = nullptr; a.tag = 42;
    ShapeVal b; b.shape = nullptr; b.tag = 42;
    ShapeVal c; c.shape = nullptr; c.tag = 99;

    CHECK(a == b);
    CHECK_FALSE(a == c);
}
