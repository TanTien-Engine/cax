#include <catch2/catch_test_macros.hpp>
#include "brepgraph_c/computation/CompGraph.h"

#include <unordered_set>

using namespace brepgraph;

// Simulate the script-side reconnect_node algorithm in C++.
// Given a deserialized OpHistory, find the op step that matches
// (op_name, upstream shape prefix) and extract const param refs.
struct ReconnectResult
{
	int step_id = -1;
	std::vector<std::pair<std::string, int>> const_refs;  // name -> step_id
};

static ReconnectResult reconnect_op(
	const OpHistory& hist,
	std::unordered_set<int>& claimed,
	const std::string& op_name,
	const std::vector<int>& shape_input_ids,
	const std::vector<std::string>& param_names)
{
	ReconnectResult result;

	for (const auto& step : hist.Steps())
	{
		if (step.op_name != op_name)
			continue;
		if (claimed.count(step.step_id))
			continue;

		size_t n_shape = shape_input_ids.size();
		if (step.inputs.size() < n_shape)
			continue;

		bool prefix_match = true;
		for (size_t j = 0; j < n_shape; ++j)
		{
			if (step.inputs[j] != shape_input_ids[j])
			{
				prefix_match = false;
				break;
			}
		}
		if (!prefix_match)
			continue;

		result.step_id = step.step_id;
		claimed.insert(step.step_id);

		for (size_t i = 0; i < param_names.size(); ++i)
		{
			size_t pos = n_shape + i;
			if (pos < step.inputs.size())
			{
				result.const_refs.push_back({param_names[i], step.inputs[pos]});
				claimed.insert(step.inputs[pos]);
			}
		}
		break;
	}

	return result;
}

// Build a typical modeling graph:
//   Box(length, width, height) -> Offset(offset, is_solid) -> Fillet(radius)
static void build_box_offset_fillet(OpHistory& h,
	int& box_step, int& offset_step, int& fillet_step,
	int& c_len, int& c_wid, int& c_hgt,
	int& c_off, int& c_solid, int& c_rad)
{
	c_len = h.AddConst(100.0, "length");
	c_wid = h.AddConst(50.0,  "width");
	c_hgt = h.AddConst(30.0,  "height");
	box_step = h.AddOp("box", {c_len, c_wid, c_hgt}, {}, "create box");

	c_off   = h.AddConst(2.0,  "offset");
	c_solid = h.AddConst(true, "is_solid");
	offset_step = h.AddOp("offset", {box_step, c_off, c_solid}, {}, "offset shell");

	c_rad = h.AddConst(1.5, "radius");
	fillet_step = h.AddOp("fillet", {offset_step, c_rad}, {}, "fillet edges");
}

TEST_CASE("Reconnect single op after round-trip", "[reconnect]")
{
	OpHistory h1;
	int c0 = h1.AddConst(10.0, "length");
	int c1 = h1.AddConst(20.0, "width");
	int c2 = h1.AddConst(30.0, "height");
	int s3 = h1.AddOp("box", {c0, c1, c2}, {}, "create box");

	uint8_t* buf = nullptr;
	uint32_t len = 0;
	h1.StoreToByteArray(&buf, len);

	OpHistory h2;
	REQUIRE(h2.LoadFromByteArray(buf, len));
	delete[] buf;

	std::unordered_set<int> claimed;
	auto result = reconnect_op(h2, claimed, "box", {},
		{"length", "width", "height"});

	CHECK(result.step_id == s3);
	REQUIRE(result.const_refs.size() == 3);
	CHECK(result.const_refs[0] == std::make_pair(std::string("length"), c0));
	CHECK(result.const_refs[1] == std::make_pair(std::string("width"),  c1));
	CHECK(result.const_refs[2] == std::make_pair(std::string("height"), c2));
}

TEST_CASE("Reconnect chain: Box -> Offset -> Fillet", "[reconnect]")
{
	OpHistory h1;
	int box_step, offset_step, fillet_step;
	int c_len, c_wid, c_hgt, c_off, c_solid, c_rad;
	build_box_offset_fillet(h1, box_step, offset_step, fillet_step,
		c_len, c_wid, c_hgt, c_off, c_solid, c_rad);

	uint8_t* buf = nullptr;
	uint32_t len = 0;
	h1.StoreToByteArray(&buf, len);

	OpHistory h2;
	REQUIRE(h2.LoadFromByteArray(buf, len));
	delete[] buf;

	std::unordered_set<int> claimed;

	// reconnect box first (no shape inputs)
	auto r_box = reconnect_op(h2, claimed, "box", {},
		{"length", "width", "height"});
	REQUIRE(r_box.step_id == box_step);

	// reconnect offset (box is upstream shape)
	auto r_offset = reconnect_op(h2, claimed, "offset",
		{r_box.step_id}, {"offset", "is_solid"});
	REQUIRE(r_offset.step_id == offset_step);
	REQUIRE(r_offset.const_refs.size() == 2);
	CHECK(r_offset.const_refs[0].second == c_off);
	CHECK(r_offset.const_refs[1].second == c_solid);

	// reconnect fillet (offset is upstream shape)
	auto r_fillet = reconnect_op(h2, claimed, "fillet",
		{r_offset.step_id}, {"radius"});
	REQUIRE(r_fillet.step_id == fillet_step);
	REQUIRE(r_fillet.const_refs.size() == 1);
	CHECK(r_fillet.const_refs[0].second == c_rad);
}

TEST_CASE("Claim prevents duplicate matching", "[reconnect]")
{
	// Two identical Box nodes -- same op, same param structure
	OpHistory h1;
	int c0 = h1.AddConst(10.0, "length");
	int c1 = h1.AddConst(20.0, "width");
	int c2 = h1.AddConst(30.0, "height");
	int box1 = h1.AddOp("box", {c0, c1, c2}, {}, "box A");

	int c3 = h1.AddConst(5.0, "length");
	int c4 = h1.AddConst(8.0, "width");
	int c5 = h1.AddConst(12.0, "height");
	int box2 = h1.AddOp("box", {c3, c4, c5}, {}, "box B");

	uint8_t* buf = nullptr;
	uint32_t len = 0;
	h1.StoreToByteArray(&buf, len);

	OpHistory h2;
	REQUIRE(h2.LoadFromByteArray(buf, len));
	delete[] buf;

	std::unordered_set<int> claimed;

	// first reconnect gets the first box
	auto r1 = reconnect_op(h2, claimed, "box", {},
		{"length", "width", "height"});
	CHECK(r1.step_id == box1);

	// second reconnect gets the second box (first is claimed)
	auto r2 = reconnect_op(h2, claimed, "box", {},
		{"length", "width", "height"});
	CHECK(r2.step_id == box2);

	// verify they got different const refs
	CHECK(r1.const_refs[0].second == c0);  // first box's length
	CHECK(r2.const_refs[0].second == c3);  // second box's length
}

TEST_CASE("Structural prefix distinguishes ops with same name", "[reconnect]")
{
	// Two "fillet" ops, each on a different upstream shape
	OpHistory h1;
	int c0 = h1.AddConst(10.0, "length");
	int c1 = h1.AddConst(20.0, "width");
	int c2 = h1.AddConst(30.0, "height");
	int box1 = h1.AddOp("box", {c0, c1, c2}, {}, "box A");

	int c3 = h1.AddConst(5.0, "length");
	int c4 = h1.AddConst(8.0, "width");
	int c5 = h1.AddConst(12.0, "height");
	int box2 = h1.AddOp("box", {c3, c4, c5}, {}, "box B");

	int c_r1 = h1.AddConst(1.0, "radius");
	int fillet1 = h1.AddOp("fillet", {box1, c_r1}, {}, "fillet A");

	int c_r2 = h1.AddConst(2.0, "radius");
	int fillet2 = h1.AddOp("fillet", {box2, c_r2}, {}, "fillet B");

	uint8_t* buf = nullptr;
	uint32_t len = 0;
	h1.StoreToByteArray(&buf, len);

	OpHistory h2;
	REQUIRE(h2.LoadFromByteArray(buf, len));
	delete[] buf;

	std::unordered_set<int> claimed;

	// reconnect boxes first
	auto r_box1 = reconnect_op(h2, claimed, "box", {},
		{"length", "width", "height"});
	auto r_box2 = reconnect_op(h2, claimed, "box", {},
		{"length", "width", "height"});

	// reconnect fillet on box2 first -- prefix match skips fillet1
	auto r_f2 = reconnect_op(h2, claimed, "fillet",
		{r_box2.step_id}, {"radius"});
	CHECK(r_f2.step_id == fillet2);
	CHECK(r_f2.const_refs[0].second == c_r2);

	// reconnect fillet on box1
	auto r_f1 = reconnect_op(h2, claimed, "fillet",
		{r_box1.step_id}, {"radius"});
	CHECK(r_f1.step_id == fillet1);
	CHECK(r_f1.const_refs[0].second == c_r1);
}

TEST_CASE("Reconnect with var_inputs", "[reconnect]")
{
	OpHistory h1;
	int c0 = h1.AddConst(10.0, "length");
	int c1 = h1.AddConst(20.0, "width");
	int c2 = h1.AddConst(30.0, "height");
	int box1 = h1.AddOp("box", {c0, c1, c2}, {}, "box A");
	int box2 = h1.AddOp("box", {c0, c1, c2}, {}, "box B");
	int merge = h1.AddOp("merge", {}, {box1, box2}, "merge shapes");

	uint8_t* buf = nullptr;
	uint32_t len = 0;
	h1.StoreToByteArray(&buf, len);

	OpHistory h2;
	REQUIRE(h2.LoadFromByteArray(buf, len));
	delete[] buf;

	// verify var_inputs survived serialization
	auto* step = h2.Get(merge);
	REQUIRE(step != nullptr);
	CHECK(step->op_name == "merge");
	CHECK(step->inputs.empty());
	CHECK(step->var_inputs.size() == 2);
	CHECK(step->var_inputs[0] == box1);
	CHECK(step->var_inputs[1] == box2);
}

TEST_CASE("Reconnect fails gracefully on mismatch", "[reconnect]")
{
	OpHistory h1;
	int c0 = h1.AddConst(10.0, "length");
	int c1 = h1.AddConst(20.0, "width");
	int c2 = h1.AddConst(30.0, "height");
	h1.AddOp("box", {c0, c1, c2}, {}, "create box");

	uint8_t* buf = nullptr;
	uint32_t len = 0;
	h1.StoreToByteArray(&buf, len);

	OpHistory h2;
	REQUIRE(h2.LoadFromByteArray(buf, len));
	delete[] buf;

	std::unordered_set<int> claimed;

	// wrong op_name
	auto r1 = reconnect_op(h2, claimed, "cylinder", {},
		{"length", "width", "height"});
	CHECK(r1.step_id == -1);

	// wrong upstream shape prefix
	auto r2 = reconnect_op(h2, claimed, "box",
		{999}, {"length", "width", "height"});
	CHECK(r2.step_id == -1);
}

TEST_CASE("Param values recoverable after reconnect", "[reconnect]")
{
	OpHistory h1;
	int c0 = h1.AddConst(42.0, "length");
	int c1 = h1.AddConst(true, "flag");
	int c2 = h1.AddConst(Vec3{1.0, 2.0, 3.0}, "pos");
	int c3 = h1.AddConst(7, "count");
	h1.AddOp("custom", {c0, c1, c2, c3}, {}, "custom op");

	uint8_t* buf = nullptr;
	uint32_t len = 0;
	h1.StoreToByteArray(&buf, len);

	OpHistory h2;
	REQUIRE(h2.LoadFromByteArray(buf, len));
	delete[] buf;

	std::unordered_set<int> claimed;
	auto result = reconnect_op(h2, claimed, "custom", {},
		{"length", "flag", "pos", "count"});
	REQUIRE(result.step_id >= 0);
	REQUIRE(result.const_refs.size() == 4);

	// after reconnect, we can read back the param values via const_refs
	auto* p_len = h2.Get(result.const_refs[0].second);
	REQUIRE(p_len != nullptr);
	CHECK(std::get<double>(p_len->imm) == 42.0);

	auto* p_flag = h2.Get(result.const_refs[1].second);
	REQUIRE(p_flag != nullptr);
	CHECK(std::get<bool>(p_flag->imm) == true);

	auto* p_pos = h2.Get(result.const_refs[2].second);
	REQUIRE(p_pos != nullptr);
	auto v = std::get<Vec3>(p_pos->imm);
	CHECK(v[0] == 1.0);
	CHECK(v[1] == 2.0);
	CHECK(v[2] == 3.0);

	auto* p_cnt = h2.Get(result.const_refs[3].second);
	REQUIRE(p_cnt != nullptr);
	CHECK(std::get<int>(p_cnt->imm) == 7);
}

TEST_CASE("UpdateConst works after reconnect", "[reconnect]")
{
	OpHistory h1;
	int c0 = h1.AddConst(10.0, "width");
	int c1 = h1.AddConst(20.0, "height");
	h1.AddOp("box", {c0, c1}, {}, "create box");

	uint8_t* buf = nullptr;
	uint32_t len = 0;
	h1.StoreToByteArray(&buf, len);

	OpHistory h2;
	REQUIRE(h2.LoadFromByteArray(buf, len));
	delete[] buf;

	std::unordered_set<int> claimed;
	auto result = reconnect_op(h2, claimed, "box", {},
		{"width", "height"});
	REQUIRE(result.step_id >= 0);

	int width_ref = result.const_refs[0].second;
	CHECK(std::get<double>(h2.Get(width_ref)->imm) == 10.0);

	// simulate on_param_dirty: update the const
	h2.UpdateConst(width_ref, Val(99.0));
	CHECK(std::get<double>(h2.Get(width_ref)->imm) == 99.0);
}

TEST_CASE("Corrupted magic rejected", "[reconnect]")
{
	OpHistory h1;
	h1.AddConst(1.0, "x");

	uint8_t* buf = nullptr;
	uint32_t len = 0;
	h1.StoreToByteArray(&buf, len);

	// corrupt magic bytes
	buf[0] = 0xFF;
	buf[1] = 0xFF;

	OpHistory h2;
	CHECK_FALSE(h2.LoadFromByteArray(buf, len));
	delete[] buf;
}
