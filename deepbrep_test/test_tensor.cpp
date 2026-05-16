#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "Tensor.h"

#include <cmath>
#include <random>

using namespace deepbrep;
using Catch::Matchers::WithinAbs;

TEST_CASE("Mat at() row-major indexing", "[tensor]")
{
    Mat m(2, 3);
    m.at(0, 0) = 1.0f;
    m.at(0, 1) = 2.0f;
    m.at(0, 2) = 3.0f;
    m.at(1, 0) = 4.0f;
    m.at(1, 1) = 5.0f;
    m.at(1, 2) = 6.0f;

    CHECK(m.row_ptr(0)[0] == 1.0f);
    CHECK(m.row_ptr(0)[2] == 3.0f);
    CHECK(m.row_ptr(1)[1] == 5.0f);
    CHECK(m.data[5] == 6.0f);
}

TEST_CASE("Mat zero()", "[tensor]")
{
    Mat m(2, 2, 9.0f);
    m.zero();
    for (auto v : m.data) {
        CHECK(v == 0.0f);
    }
}

TEST_CASE("Mat xavier_init produces values within bound", "[tensor]")
{
    std::mt19937 rng(123);
    Mat m(8, 8);
    m.xavier_init(rng);
    const float limit = std::sqrt(6.0f / 16.0f);
    for (auto v : m.data) {
        CHECK(v >= -limit);
        CHECK(v <=  limit);
    }
}

TEST_CASE("vec_mat_mul row-vec by matrix", "[tensor]")
{
    Mat W(2, 3);
    W.at(0, 0) = 1; W.at(0, 1) = 2; W.at(0, 2) = 3;
    W.at(1, 0) = 4; W.at(1, 1) = 5; W.at(1, 2) = 6;

    float in[2] = {1.0f, 2.0f};
    float out[3] = {0.0f, 0.0f, 0.0f};
    vec_mat_mul(in, W, out);
    // out[j] = in[0]*W(0,j) + in[1]*W(1,j)
    CHECK(out[0] == 9.0f);
    CHECK(out[1] == 12.0f);
    CHECK(out[2] == 15.0f);
}

TEST_CASE("vec_add", "[tensor]")
{
    float out[3] = {1.0f, 2.0f, 3.0f};
    float b[3]   = {10.0f, 20.0f, 30.0f};
    vec_add(out, b, 3);
    CHECK(out[0] == 11.0f);
    CHECK(out[1] == 22.0f);
    CHECK(out[2] == 33.0f);
}

TEST_CASE("relu clamps negatives", "[tensor]")
{
    float x[5] = {-1.0f, 0.0f, 0.5f, -0.1f, 3.0f};
    relu(x, 5);
    CHECK(x[0] == 0.0f);
    CHECK(x[1] == 0.0f);
    CHECK(x[2] == 0.5f);
    CHECK(x[3] == 0.0f);
    CHECK(x[4] == 3.0f);
}

TEST_CASE("relu_backward zeros grad where activation was non-positive", "[tensor]")
{
    float x[5]    = {-1.0f, 0.0f, 1.0f, -0.5f, 2.0f};
    float grad[5] = { 7.0f, 7.0f, 7.0f,  7.0f, 7.0f};
    relu_backward(grad, x, 5);
    CHECK(grad[0] == 0.0f);
    CHECK(grad[1] == 0.0f);     // x == 0 is treated as non-positive
    CHECK(grad[2] == 7.0f);
    CHECK(grad[3] == 0.0f);
    CHECK(grad[4] == 7.0f);
}

TEST_CASE("softmax sums to 1 and is monotone in input", "[tensor]")
{
    float x[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    softmax(x, 4);
    float sum = 0.0f;
    for (int i = 0; i < 4; ++i) sum += x[i];
    CHECK_THAT(sum, WithinAbs(1.0, 1e-5));
    CHECK(x[0] < x[1]);
    CHECK(x[1] < x[2]);
    CHECK(x[2] < x[3]);
}

TEST_CASE("softmax numerical stability with large values", "[tensor]")
{
    float x[3] = {1000.0f, 1001.0f, 1002.0f};
    softmax(x, 3);
    float sum = x[0] + x[1] + x[2];
    CHECK_THAT(sum, WithinAbs(1.0, 1e-5));
    // Same relative ordering as small-input case.
    CHECK(x[0] < x[1]);
    CHECK(x[1] < x[2]);
}

TEST_CASE("cross_entropy returns -log(p) for the labeled class", "[tensor]")
{
    float p[3] = {0.1f, 0.7f, 0.2f};
    float loss = cross_entropy(p, 1);
    CHECK_THAT(loss, WithinAbs(-std::log(0.7), 1e-5));
}

TEST_CASE("cross_entropy clamps to avoid log(0)", "[tensor]")
{
    float p[2] = {0.0f, 1.0f};
    float loss = cross_entropy(p, 0);
    // Should be a large but finite number, not inf.
    CHECK(std::isfinite(loss));
    CHECK(loss > 5.0f);
}
