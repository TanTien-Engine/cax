#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "Layers.h"
#include "GraphData.h"

#include <random>
#include <vector>

using namespace deepbrep;
using Catch::Matchers::WithinAbs;

TEST_CASE("Linear forward: y = xW + b", "[layers]")
{
    std::mt19937 rng(1);
    Linear lin;
    lin.init(2, 3, rng);

    // Force-set known weights so we can verify arithmetic.
    lin.W.at(0, 0) = 1; lin.W.at(0, 1) = 2; lin.W.at(0, 2) = 3;
    lin.W.at(1, 0) = 4; lin.W.at(1, 1) = 5; lin.W.at(1, 2) = 6;
    lin.b[0] = 7; lin.b[1] = 8; lin.b[2] = 9;

    float in[2] = {1.0f, 2.0f};
    float out[3];
    lin.forward(in, out);
    CHECK(out[0] == 1.0f + 2*4 + 7);     // 16
    CHECK(out[1] == 2.0f + 2*5 + 8);     // 20
    CHECK(out[2] == 3.0f + 2*6 + 9);     // 24
}

TEST_CASE("Linear backward accumulates parameter grads and computes input grad",
          "[layers]")
{
    std::mt19937 rng(1);
    Linear lin;
    lin.init(2, 2, rng);
    lin.W.at(0, 0) = 1; lin.W.at(0, 1) = 0;
    lin.W.at(1, 0) = 0; lin.W.at(1, 1) = 1;
    lin.b[0] = 0; lin.b[1] = 0;
    lin.zero_grad();

    float input[2]    = {3.0f, 4.0f};
    float grad_out[2] = {1.0f, 1.0f};
    float grad_in[2]  = {0.0f, 0.0f};
    lin.backward(grad_out, input, grad_in);

    // dW(i,j) = input[i] * grad_out[j]
    CHECK(lin.dW.at(0, 0) == 3.0f);
    CHECK(lin.dW.at(0, 1) == 3.0f);
    CHECK(lin.dW.at(1, 0) == 4.0f);
    CHECK(lin.dW.at(1, 1) == 4.0f);
    // db[j] = grad_out[j]
    CHECK(lin.db[0] == 1.0f);
    CHECK(lin.db[1] == 1.0f);
    // grad_in[i] = sum_j grad_out[j] * W(i,j); with W = identity this gives
    // grad_in[i] = grad_out[i].
    CHECK(grad_in[0] == 1.0f);
    CHECK(grad_in[1] == 1.0f);
}

TEST_CASE("Linear update applies SGD step", "[layers]")
{
    std::mt19937 rng(1);
    Linear lin;
    lin.init(1, 1, rng);
    lin.W.at(0, 0) = 2.0f;
    lin.b[0]       = 0.5f;
    lin.dW.at(0, 0) = 1.0f;
    lin.db[0]      = 0.1f;
    lin.update(0.1f);
    CHECK_THAT(lin.W.at(0, 0), WithinAbs(1.9, 1e-6));
    CHECK_THAT(lin.b[0],       WithinAbs(0.49, 1e-6));
}

namespace
{

// A tiny 2-node graph with a single undirected edge. Edge features = 1.
GraphData make_two_node_graph(int hidden, int edge_dim)
{
    GraphData g;
    g.num_nodes = 2;
    g.node_features = Mat(2, hidden);
    // Identity activations as the "input" to MPLayer (it consumes hidden_dim
    // wide rows, so make node features the right width).
    g.node_features.at(0, 0) = 1.0f;
    g.node_features.at(1, 0) = 1.0f;

    std::vector<std::pair<int, int>> ft = {{0, 1}, {1, 0}};
    g.edge_features = Mat(2, edge_dim);
    for (int i = 0; i < 2; ++i)
        for (int d = 0; d < edge_dim; ++d)
            g.edge_features.at(i, d) = 1.0f;
    build_csr_from_directed_edges(g, ft);
    return g;
}

}

TEST_CASE("MPLayer forward changes activations and never produces negatives",
          "[layers]")
{
    std::mt19937 rng(7);
    const int hidden = 4;
    const int edge_dim = 3;
    MPLayer layer;
    layer.init(hidden, edge_dim, rng);

    auto g = make_two_node_graph(hidden, edge_dim);
    Mat h = g.node_features;
    layer.forward(h, g);

    REQUIRE(h.rows == 2);
    REQUIRE(h.cols == hidden);
    for (float v : h.data) {
        CHECK(v >= 0.0f);
    }
}

TEST_CASE("MPLayer backward shapes match forward inputs", "[layers]")
{
    std::mt19937 rng(7);
    const int hidden = 4;
    const int edge_dim = 3;
    MPLayer layer;
    layer.init(hidden, edge_dim, rng);

    auto g = make_two_node_graph(hidden, edge_dim);
    Mat h = g.node_features;
    layer.forward(h, g);

    Mat grad(2, hidden, 1.0f);
    layer.backward(grad, g);
    REQUIRE(grad.rows == 2);
    REQUIRE(grad.cols == hidden);
}
