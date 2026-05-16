#include "GraphData.h"

namespace deepbrep
{

void build_csr_from_directed_edges(
    GraphData& g,
    const std::vector<std::pair<int, int>>& from_to)
{
    g.num_edges = static_cast<int>(from_to.size());
    g.adj_offset.assign(g.num_nodes + 1, 0);
    g.adj_list.resize(g.num_edges);

    // Count outgoing edges per source.
    for (auto& ft : from_to) {
        g.adj_offset[ft.first + 1]++;
    }
    // Prefix sum -> offsets.
    for (int i = 1; i <= g.num_nodes; ++i) {
        g.adj_offset[i] += g.adj_offset[i - 1];
    }

    // Fill adj_list using a cursor per source.
    std::vector<int> cursor = g.adj_offset;
    for (int i = 0; i < g.num_edges; ++i) {
        int src = from_to[i].first;
        int slot = cursor[src]++;
        g.adj_list[slot].neighbor = from_to[i].second;
        g.adj_list[slot].edge_idx = i;
    }
}

}
