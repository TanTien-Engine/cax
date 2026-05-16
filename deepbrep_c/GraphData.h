#pragma once

#include "Tensor.h"

#include <vector>

namespace deepbrep
{

// One BRep sample for the GNN: faces as nodes, face-to-face adjacency as
// (undirected, stored as two directed) edges. CSR layout makes message
// passing cache-friendly.
struct GraphData
{
    struct AdjEntry
    {
        int neighbor = 0;
        int edge_idx = 0;     // index into edge_features
    };

    int num_nodes = 0;
    int num_edges = 0;        // directed-edge count (undirected counted twice)

    Mat node_features;        // [num_nodes, kNodeFeatDim]
    Mat edge_features;        // [num_edges, kEdgeFeatDim]

    std::vector<int>      adj_offset;   // [num_nodes + 1]
    std::vector<AdjEntry> adj_list;     // [num_edges]

    std::vector<int>      labels;       // [num_nodes], optional (training only)
};

// Builds CSR `adj_offset`/`adj_list` and copies edge feature rows from a flat
// list of directed (from, to, edge_feature_index) records sorted by `from`.
// `edge_rows` must be pre-filled in `g.edge_features` in the same order as
// `from_to`. Sets g.num_edges.
//
// Helper for tests and dataset builders -- callers that already have CSR
// don't need this.
void build_csr_from_directed_edges(
    GraphData& g,
    const std::vector<std::pair<int, int>>& from_to);

}
