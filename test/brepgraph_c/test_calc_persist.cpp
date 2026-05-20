#include <catch2/catch_test_macros.hpp>
#include "brepgraph_c/computation/CalcGraph.h"

using namespace brepgraph;

TEST_CASE("OpHistory byte-array round-trip", "[calc_persist]")
{
	OpHistory h1;
	int s0 = h1.AddConst(42.0, "width");
	int s1 = h1.AddConst(10.0, "height");
	int s2 = h1.AddConst(Vec3{1.0, 2.0, 3.0}, "origin");
	int s3 = h1.AddOp("box", {s0, s1, s2}, {}, "create box");
	int s4 = h1.AddConst(2.0, "fillet_r");
	int s5 = h1.AddOp("fillet", {s3, s4}, {}, "fillet box");

	uint8_t* buf = nullptr;
	uint32_t len = 0;
	h1.StoreToByteArray(&buf, len);
	REQUIRE(buf != nullptr);
	REQUIRE(len > 0);

	OpHistory h2;
	REQUIRE(h2.LoadFromByteArray(buf, len));
	delete[] buf;

	REQUIRE(h2.Size() == h1.Size());

	auto* p0 = h2.Get(s0);
	REQUIRE(p0 != nullptr);
	CHECK(p0->op_name == "$num");
	CHECK(std::get<double>(p0->imm) == 42.0);
	CHECK(p0->desc == "width");

	auto* p2 = h2.Get(s2);
	REQUIRE(p2 != nullptr);
	CHECK(p2->op_name == "$vec3");
	auto v = std::get<Vec3>(p2->imm);
	CHECK(v[0] == 1.0);
	CHECK(v[1] == 2.0);
	CHECK(v[2] == 3.0);

	auto* p5 = h2.Get(s5);
	REQUIRE(p5 != nullptr);
	CHECK(p5->op_name == "fillet");
	CHECK(p5->inputs.size() == 2);
	CHECK(p5->inputs[0] == s3);
	CHECK(p5->inputs[1] == s4);
	CHECK(p5->desc == "fillet box");
}

TEST_CASE("OpHistory bool and int round-trip", "[calc_persist]")
{
	OpHistory h1;
	int s0 = h1.AddConst(7, "count");
	int s1 = h1.AddConst(true, "flag");

	uint8_t* buf = nullptr;
	uint32_t len = 0;
	h1.StoreToByteArray(&buf, len);

	OpHistory h2;
	REQUIRE(h2.LoadFromByteArray(buf, len));
	delete[] buf;

	CHECK(std::get<int>(h2.Get(s0)->imm) == 7);
	CHECK(std::get<bool>(h2.Get(s1)->imm) == true);
}

TEST_CASE("OpHistory empty round-trip", "[calc_persist]")
{
	OpHistory h1;

	uint8_t* buf = nullptr;
	uint32_t len = 0;
	h1.StoreToByteArray(&buf, len);

	OpHistory h2;
	REQUIRE(h2.LoadFromByteArray(buf, len));
	delete[] buf;

	CHECK(h2.Size() == 0);
}
