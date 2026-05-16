#include "ToyDataset.h"
#include "FeatureLabels.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <utility>
#include <vector>

namespace deepbrep
{

GraphData make_box_with_hole_graph(std::mt19937& /*rng*/)
{
    GraphData g;

    const int NUM_NODES = 8;
    g.num_nodes = NUM_NODES;
    g.node_features = Mat(NUM_NODES, kNodeFeatDim);
    g.labels.assign(NUM_NODES, 0);

    // Faces 0..5: the six box faces -- planar, label = Stock.
    for (int i = 0; i < 6; ++i) {
        g.node_features.at(i, 0) = 1.0f;          // plane
        g.node_features.at(i, 6) = 0.5f;          // area
        g.labels[i] = static_cast<int>(FaceClass::Stock);
    }
    const float normals[6][3] = {
        { 1, 0, 0}, {-1, 0, 0}, { 0, 1, 0},
        { 0,-1, 0}, { 0, 0, 1}, { 0, 0,-1},
    };
    for (int i = 0; i < 6; ++i) {
        g.node_features.at(i, 7) = normals[i][0];
        g.node_features.at(i, 8) = normals[i][1];
        g.node_features.at(i, 9) = normals[i][2];
        g.node_features.at(i, 10) = 4.0f / 10.0f;  // 4 wire edges
        g.node_features.at(i, 11) = 0.0f;
    }
    g.node_features.at(4, 11) = 1.0f;              // top face has hole loop

    // Face 6: cylindrical hole wall, label = Hole.
    g.node_features.at(6, 1) = 1.0f;               // cylinder
    g.node_features.at(6, 6) = 0.2f;
    g.node_features.at(6, 9) = 1.0f;               // axis Z
    g.node_features.at(6, 12) = 0.5f;              // curvature
    g.labels[6] = static_cast<int>(FaceClass::Hole);

    // Face 7: hole-bottom planar face, label = Hole.
    g.node_features.at(7, 0) = 1.0f;
    g.node_features.at(7, 6) = 0.05f;
    g.node_features.at(7, 9) = -1.0f;
    g.labels[7] = static_cast<int>(FaceClass::Hole);

    // Build directed edge list. We accumulate features inline into a vector
    // and feed (from, to) into the CSR builder.
    std::vector<std::pair<int, int>> from_to;
    std::vector<std::array<float, kEdgeFeatDim>> edge_rows;

    auto add_edge = [&](int a, int b, int curve_type, int convexity,
                        float dihedral, float length) {
        std::array<float, kEdgeFeatDim> feats = {};
        feats[curve_type] = 1.0f;
        feats[5 + convexity] = 1.0f;
        feats[8] = dihedral;
        feats[9] = length;
        from_to.emplace_back(a, b);
        edge_rows.push_back(feats);
        from_to.emplace_back(b, a);
        edge_rows.push_back(feats);
    };

    const int box_adj[12][2] = {
        {0,2}, {0,3}, {0,4}, {0,5},
        {1,2}, {1,3}, {1,4}, {1,5},
        {2,4}, {2,5}, {3,4}, {3,5},
    };
    for (auto& a : box_adj) {
        add_edge(a[0], a[1],
                 static_cast<int>(CurveType::Line),
                 static_cast<int>(Convexity::Convex),
                 0.5f, 0.5f);
    }
    add_edge(4, 6, static_cast<int>(CurveType::Circle),
             static_cast<int>(Convexity::Concave), 0.75f, 0.3f);
    add_edge(6, 7, static_cast<int>(CurveType::Circle),
             static_cast<int>(Convexity::Concave), 0.75f, 0.3f);

    const int num_edges = static_cast<int>(from_to.size());
    g.edge_features = Mat(num_edges, kEdgeFeatDim);

    // Sort by source so the CSR cursor walk produces a stable layout.
    std::vector<int> perm(num_edges);
    for (int i = 0; i < num_edges; ++i) perm[i] = i;
    std::sort(perm.begin(), perm.end(), [&](int a, int b) {
        return from_to[a].first < from_to[b].first;
    });

    std::vector<std::pair<int, int>> ft_sorted(num_edges);
    for (int i = 0; i < num_edges; ++i) {
        ft_sorted[i] = from_to[perm[i]];
        std::memcpy(g.edge_features.row_ptr(i),
                    edge_rows[perm[i]].data(),
                    kEdgeFeatDim * sizeof(float));
    }
    build_csr_from_directed_edges(g, ft_sorted);
    return g;
}

}
