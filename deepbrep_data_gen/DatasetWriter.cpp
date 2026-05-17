#include "DatasetWriter.h"

#include <cstdio>
#include <cstring>

namespace deepbrep_data_gen
{

bool merge_labels_into_graph(deepbrep::GraphData& g,
                             const std::vector<FaceLabel>& labels)
{
    if (static_cast<int>(labels.size()) != g.num_nodes) {
        g.labels.clear();
        g.instance_ids.clear();
        return false;
    }
    g.labels.resize(g.num_nodes);
    g.instance_ids.resize(g.num_nodes);
    for (int i = 0; i < g.num_nodes; ++i) {
        g.labels[i]       = labels[i].class_id;
        g.instance_ids[i] = labels[i].instance_id;
    }
    return true;
}

DatasetWriter::DatasetWriter(const std::string& path,
                             uint32_t node_feat_dim,
                             uint32_t edge_feat_dim,
                             uint32_t num_classes)
{
    m_file = std::fopen(path.c_str(), "wb");
    if (!m_file) return;

    const char magic[4] = {'D', 'S', 'E', 'T'};
    std::fwrite(magic, 1, 4, m_file);

    // Reserve a slot for the sample count; patched in Close().
    m_count_offset = std::ftell(m_file);
    const uint32_t zero = 0;
    std::fwrite(&zero, sizeof(uint32_t), 1, m_file);

    std::fwrite(&node_feat_dim, sizeof(uint32_t), 1, m_file);
    std::fwrite(&edge_feat_dim, sizeof(uint32_t), 1, m_file);
    std::fwrite(&num_classes,   sizeof(uint32_t), 1, m_file);
}

DatasetWriter::~DatasetWriter()
{
    Close();
}

void DatasetWriter::Close()
{
    if (!m_file) return;
    std::fseek(m_file, m_count_offset, SEEK_SET);
    std::fwrite(&m_count, sizeof(uint32_t), 1, m_file);
    std::fclose(m_file);
    m_file = nullptr;
}

// TODO(io): this duplicates write_graph_data() logic because the latter
// owns its own FILE*. Refactor write_graph_data to take a FILE* and call it
// here, so the format stays in one place.
bool DatasetWriter::Append(deepbrep::GraphData& g,
                           const std::vector<FaceLabel>& labels)
{
    if (!m_file) return false;
    if (!merge_labels_into_graph(g, labels)) return false;

    auto wvec_int = [&](const std::vector<int>& v) {
        const uint32_t n = static_cast<uint32_t>(v.size());
        std::fwrite(&n, sizeof(n), 1, m_file);
        if (n) std::fwrite(v.data(), sizeof(int), n, m_file);
    };
    auto wvec_u32 = [&](const std::vector<uint32_t>& v) {
        const uint32_t n = static_cast<uint32_t>(v.size());
        std::fwrite(&n, sizeof(n), 1, m_file);
        if (n) std::fwrite(v.data(), sizeof(uint32_t), n, m_file);
    };
    auto wvec_f = [&](const std::vector<float>& v) {
        const uint32_t n = static_cast<uint32_t>(v.size());
        std::fwrite(&n, sizeof(n), 1, m_file);
        if (n) std::fwrite(v.data(), sizeof(float), n, m_file);
    };

    const uint32_t hdr[2] = {
        static_cast<uint32_t>(g.num_nodes),
        static_cast<uint32_t>(g.num_edges),
    };
    std::fwrite(hdr, sizeof(uint32_t), 2, m_file);

    wvec_f(g.node_features.data);
    wvec_f(g.edge_features.data);
    wvec_int(g.adj_offset);
    {
        const uint32_t n = static_cast<uint32_t>(g.adj_list.size());
        std::fwrite(&n, sizeof(n), 1, m_file);
        if (n) std::fwrite(g.adj_list.data(),
                           sizeof(deepbrep::GraphData::AdjEntry), n, m_file);
    }
    wvec_int(g.labels);
    wvec_u32(g.instance_ids);

    ++m_count;
    return true;
}

}
