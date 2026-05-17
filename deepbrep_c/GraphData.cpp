#include "GraphData.h"

#include <cstdio>

namespace deepbrep
{

namespace {

template <class T>
bool write_vec(std::FILE* f, const std::vector<T>& v)
{
    const uint32_t n = static_cast<uint32_t>(v.size());
    if (std::fwrite(&n, sizeof(n), 1, f) != 1) return false;
    if (n == 0) return true;
    return std::fwrite(v.data(), sizeof(T), n, f) == n;
}

template <class T>
bool read_vec(std::FILE* f, std::vector<T>& v)
{
    uint32_t n = 0;
    if (std::fread(&n, sizeof(n), 1, f) != 1) return false;
    v.resize(n);
    if (n == 0) return true;
    return std::fread(v.data(), sizeof(T), n, f) == n;
}

}  // anonymous

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

bool write_graph_data(const std::string& path, const GraphData& g)
{
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;

    const char magic[4] = {'D', 'G', 'D', '0'};
    std::fwrite(magic, 1, 4, f);

    const uint32_t hdr[4] = {
        static_cast<uint32_t>(g.num_nodes),
        static_cast<uint32_t>(g.num_edges),
        static_cast<uint32_t>(g.node_features.cols),
        static_cast<uint32_t>(g.edge_features.cols),
    };
    std::fwrite(hdr, sizeof(uint32_t), 4, f);

    bool ok = true;
    ok = ok && write_vec(f, g.node_features.data);
    ok = ok && write_vec(f, g.edge_features.data);
    ok = ok && write_vec(f, g.adj_offset);
    // AdjEntry is a POD pair of ints -- safe to dump as bytes.
    {
        const uint32_t n = static_cast<uint32_t>(g.adj_list.size());
        ok = ok && std::fwrite(&n, sizeof(n), 1, f) == 1;
        if (ok && n) {
            ok = std::fwrite(g.adj_list.data(),
                             sizeof(GraphData::AdjEntry), n, f) == n;
        }
    }
    ok = ok && write_vec(f, g.labels);
    ok = ok && write_vec(f, g.instance_ids);

    std::fclose(f);
    return ok;
}

bool read_graph_data(const std::string& path, GraphData& g)
{
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;

    char magic[4] = {};
    if (std::fread(magic, 1, 4, f) != 4 ||
        magic[0] != 'D' || magic[1] != 'G' ||
        magic[2] != 'D' || magic[3] != '0')
    {
        std::fclose(f);
        return false;
    }

    uint32_t hdr[4] = {};
    if (std::fread(hdr, sizeof(uint32_t), 4, f) != 4) {
        std::fclose(f);
        return false;
    }
    g.num_nodes      = static_cast<int>(hdr[0]);
    g.num_edges      = static_cast<int>(hdr[1]);
    g.node_features  = Mat(g.num_nodes, static_cast<int>(hdr[2]));
    g.edge_features  = Mat(g.num_edges, static_cast<int>(hdr[3]));

    bool ok = true;
    ok = ok && read_vec(f, g.node_features.data);
    ok = ok && read_vec(f, g.edge_features.data);
    ok = ok && read_vec(f, g.adj_offset);
    {
        uint32_t n = 0;
        ok = ok && std::fread(&n, sizeof(n), 1, f) == 1;
        g.adj_list.resize(n);
        if (ok && n) {
            ok = std::fread(g.adj_list.data(),
                            sizeof(GraphData::AdjEntry), n, f) == n;
        }
    }
    ok = ok && read_vec(f, g.labels);
    ok = ok && read_vec(f, g.instance_ids);

    std::fclose(f);
    return ok;
}

}
