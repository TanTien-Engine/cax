#pragma once

#include "Tensor.h"

#include <cstdint>
#include <string>
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

    // Optional per-node training labels. labels[i] = semantic class id;
    // instance_ids[i] = which feature instance face i belongs to (faces of
    // the same hole / fillet share an id). 0 / empty means "not labeled".
    std::vector<int>      labels;
    std::vector<uint32_t> instance_ids;
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

// Binary serialization. Format is private to this module and host-native --
// the dataset on-disk format is meant for the same machine that produced it.
//
// Layout: magic 'DGD0', uint32 of num_nodes/num_edges/node_dim/edge_dim, then
// node_features, edge_features, adj_offset, adj_list (interleaved), labels,
// instance_ids. Optional fields are length-prefixed (0 = absent).
bool write_graph_data(const std::string& path, const GraphData& g);
bool read_graph_data (const std::string& path,       GraphData& g);

// Multi-sample dataset: one file holding N graphs back-to-back, prefixed by
// a small header. Used by the trainer for streaming.
struct DatasetHeader
{
    uint32_t num_samples = 0;
    uint32_t node_feat_dim = 0;
    uint32_t edge_feat_dim = 0;
    uint32_t num_classes = 0;     // 0 if labels are heterogeneous / unset
};

}
