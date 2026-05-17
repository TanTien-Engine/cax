#include "DatasetReader.h"

#include <cstdio>
#include <cstring>

namespace deepbrep
{

namespace {

template <class T>
bool read_vec(std::FILE* f, std::vector<T>& v)
{
    uint32_t n = 0;
    if (std::fread(&n, sizeof(n), 1, f) != 1) return false;
    v.resize(n);
    if (n == 0) return true;
    return std::fread(v.data(), sizeof(T), n, f) == n;
}

bool read_one_sample(std::FILE* f, uint32_t node_feat_dim, uint32_t edge_feat_dim,
                     GraphData& g)
{
    uint32_t hdr[2] = {};
    if (std::fread(hdr, sizeof(uint32_t), 2, f) != 2) return false;
    g.num_nodes = static_cast<int>(hdr[0]);
    g.num_edges = static_cast<int>(hdr[1]);

    g.node_features = Mat(g.num_nodes, static_cast<int>(node_feat_dim));
    g.edge_features = Mat(g.num_edges, static_cast<int>(edge_feat_dim));

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
    return ok;
}

}  // anonymous

bool read_dataset(const std::string& path,
                  DatasetHeader& header,
                  std::vector<GraphData>& out)
{
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;

    char magic[4] = {};
    if (std::fread(magic, 1, 4, f) != 4 ||
        magic[0] != 'D' || magic[1] != 'S' ||
        magic[2] != 'E' || magic[3] != 'T')
    {
        std::fclose(f);
        return false;
    }

    uint32_t fields[4] = {};
    if (std::fread(fields, sizeof(uint32_t), 4, f) != 4) {
        std::fclose(f);
        return false;
    }
    header.num_samples   = fields[0];
    header.node_feat_dim = fields[1];
    header.edge_feat_dim = fields[2];
    header.num_classes   = fields[3];

    out.clear();
    out.reserve(header.num_samples);
    for (uint32_t i = 0; i < header.num_samples; ++i) {
        GraphData g;
        if (!read_one_sample(f, header.node_feat_dim, header.edge_feat_dim, g)) {
            std::fclose(f);
            return false;
        }
        out.push_back(std::move(g));
    }

    std::fclose(f);
    return true;
}

}
