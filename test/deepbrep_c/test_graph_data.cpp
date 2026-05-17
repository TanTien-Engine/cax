#include <catch2/catch_test_macros.hpp>

#include "GraphData.h"

#include <vector>

using namespace deepbrep;

TEST_CASE("build_csr_from_directed_edges produces expected offsets", "[graph_data]")
{
    GraphData g;
    g.num_nodes = 4;

    // Edges (sorted by source):
    //   0 -> 1, 0 -> 2
    //   1 -> 0
    //   3 -> 2
    std::vector<std::pair<int, int>> ft = {
        {0, 1}, {0, 2},
        {1, 0},
        {3, 2},
    };
    build_csr_from_directed_edges(g, ft);

    REQUIRE(g.num_edges == 4);
    REQUIRE(g.adj_offset.size() == 5);
    CHECK(g.adj_offset[0] == 0);
    CHECK(g.adj_offset[1] == 2);   // node 0 has 2 outgoing
    CHECK(g.adj_offset[2] == 3);   // node 1 has 1 outgoing
    CHECK(g.adj_offset[3] == 3);   // node 2 has 0 outgoing
    CHECK(g.adj_offset[4] == 4);   // node 3 has 1 outgoing

    CHECK(g.adj_list[0].neighbor == 1);
    CHECK(g.adj_list[1].neighbor == 2);
    CHECK(g.adj_list[2].neighbor == 0);
    CHECK(g.adj_list[3].neighbor == 2);

    CHECK(g.adj_list[0].edge_idx == 0);
    CHECK(g.adj_list[3].edge_idx == 3);
}

TEST_CASE("build_csr_from_directed_edges with isolated nodes leaves zero-spans",
          "[graph_data]")
{
    GraphData g;
    g.num_nodes = 5;
    std::vector<std::pair<int, int>> ft = { {2, 3} };
    build_csr_from_directed_edges(g, ft);

    REQUIRE(g.num_edges == 1);
    CHECK(g.adj_offset[0] == 0);
    CHECK(g.adj_offset[1] == 0);
    CHECK(g.adj_offset[2] == 0);
    CHECK(g.adj_offset[3] == 1);
    CHECK(g.adj_offset[4] == 1);
    CHECK(g.adj_offset[5] == 1);
}
