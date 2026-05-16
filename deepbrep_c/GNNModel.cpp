#include "GNNModel.h"

#include <cstdio>
#include <cstring>

namespace deepbrep
{

void GNNModel::init(int node_feat_dim, int edge_feat_dim, int hidden_dim,
                    int num_classes, int num_layers, std::mt19937& rng)
{
    m_node_feat_dim = node_feat_dim;
    m_edge_feat_dim = edge_feat_dim;
    m_hidden_dim    = hidden_dim;
    m_num_classes   = num_classes;
    m_num_layers    = num_layers;

    m_node_encoder.init(node_feat_dim, hidden_dim, rng);
    m_mp_layers.resize(num_layers);
    for (int i = 0; i < num_layers; ++i) {
        m_mp_layers[i].init(hidden_dim, edge_feat_dim, rng);
    }
    m_classifier.init(hidden_dim, num_classes, rng);
}

float GNNModel::forward(const GraphData& graph, bool compute_loss)
{
    const int N = graph.num_nodes;

    Mat h(N, m_hidden_dim);
    for (int i = 0; i < N; ++i) {
        m_node_encoder.forward(graph.node_features.row_ptr(i), h.row_ptr(i));
        relu(h.row_ptr(i), m_hidden_dim);
    }
    m_encoded_h = h;

    m_layer_outputs.resize(m_num_layers);
    for (int l = 0; l < m_num_layers; ++l) {
        Mat h_before = h;
        m_mp_layers[l].forward(h, graph);
        // Residual.
        const int total = N * m_hidden_dim;
        for (int i = 0; i < total; ++i) {
            h.data[i] += h_before.data[i];
        }
        m_layer_outputs[l] = h;
    }

    m_logits = Mat(N, m_num_classes);
    m_probs  = Mat(N, m_num_classes);
    float total_loss = 0.0f;

    for (int i = 0; i < N; ++i) {
        m_classifier.forward(h.row_ptr(i), m_logits.row_ptr(i));
        std::memcpy(m_probs.row_ptr(i),
                    m_logits.row_ptr(i),
                    m_num_classes * sizeof(float));
        softmax(m_probs.row_ptr(i), m_num_classes);

        if (compute_loss && !graph.labels.empty()) {
            total_loss += cross_entropy(m_probs.row_ptr(i), graph.labels[i]);
        }
    }

    return (compute_loss && !graph.labels.empty()) ? (total_loss / N) : 0.0f;
}

void GNNModel::backward(const GraphData& graph)
{
    const int N = graph.num_nodes;

    // d(softmax + cross_entropy) / d(logits) = probs - one_hot(label), averaged.
    Mat grad_logits(N, m_num_classes);
    const float inv_n = 1.0f / static_cast<float>(N);
    for (int i = 0; i < N; ++i) {
        for (int c = 0; c < m_num_classes; ++c) {
            grad_logits.at(i, c) = m_probs.at(i, c) * inv_n;
        }
        grad_logits.at(i, graph.labels[i]) -= inv_n;
    }

    Mat grad_h(N, m_hidden_dim);
    for (int i = 0; i < N; ++i) {
        const float* cls_input = m_layer_outputs.back().row_ptr(i);
        m_classifier.backward(grad_logits.row_ptr(i),
                              cls_input,
                              grad_h.row_ptr(i));
    }

    for (int l = m_num_layers - 1; l >= 0; --l) {
        Mat grad_residual = grad_h;
        m_mp_layers[l].backward(grad_h, graph);
        const int total = N * m_hidden_dim;
        for (int i = 0; i < total; ++i) {
            grad_h.data[i] += grad_residual.data[i];
        }
    }

    for (int i = 0; i < N; ++i) {
        relu_backward(grad_h.row_ptr(i), m_encoded_h.row_ptr(i), m_hidden_dim);
        m_node_encoder.backward(grad_h.row_ptr(i),
                                graph.node_features.row_ptr(i),
                                nullptr);
    }
}

void GNNModel::zero_grad()
{
    m_node_encoder.zero_grad();
    for (auto& l : m_mp_layers) {
        l.zero_grad();
    }
    m_classifier.zero_grad();
}

void GNNModel::update(float lr)
{
    m_node_encoder.update(lr);
    for (auto& l : m_mp_layers) {
        l.update(lr);
    }
    m_classifier.update(lr);
}

std::vector<int> GNNModel::predict_cached(int num_nodes) const
{
    std::vector<int> preds(num_nodes);
    for (int i = 0; i < num_nodes; ++i) {
        int best = 0;
        for (int c = 1; c < m_num_classes; ++c) {
            if (m_probs.at(i, c) > m_probs.at(i, best)) {
                best = c;
            }
        }
        preds[i] = best;
    }
    return preds;
}

std::vector<int> GNNModel::predict(const GraphData& graph)
{
    forward(graph, false);
    return predict_cached(graph.num_nodes);
}

// ============================================================
// Checkpoint I/O
// ============================================================
// Binary layout:
//   header  : magic 'DBR0', uint32 each of node_dim, edge_dim, hidden,
//             num_classes, num_layers
//   linear  : in_dim u32, out_dim u32, W (in*out floats), b (out floats)
//   model   : encoder, num_layers x (msg_linear, self_linear), classifier
//
// Endianness is host-native -- the file is not portable across platforms.

static bool write_linear(std::FILE* f, const Linear& lin)
{
    const uint32_t in_d  = static_cast<uint32_t>(lin.in_dim);
    const uint32_t out_d = static_cast<uint32_t>(lin.out_dim);
    if (std::fwrite(&in_d,  sizeof(in_d),  1, f) != 1) return false;
    if (std::fwrite(&out_d, sizeof(out_d), 1, f) != 1) return false;
    if (std::fwrite(lin.W.data.data(), sizeof(float),
                    lin.W.data.size(), f) != lin.W.data.size()) return false;
    if (std::fwrite(lin.b.data(), sizeof(float),
                    lin.b.size(), f) != lin.b.size()) return false;
    return true;
}

static bool read_linear(std::FILE* f, Linear& lin)
{
    uint32_t in_d = 0, out_d = 0;
    if (std::fread(&in_d,  sizeof(in_d),  1, f) != 1) return false;
    if (std::fread(&out_d, sizeof(out_d), 1, f) != 1) return false;
    if (static_cast<int>(in_d) != lin.in_dim || static_cast<int>(out_d) != lin.out_dim) {
        return false;
    }
    if (std::fread(lin.W.data.data(), sizeof(float),
                   lin.W.data.size(), f) != lin.W.data.size()) return false;
    if (std::fread(lin.b.data(), sizeof(float),
                   lin.b.size(), f) != lin.b.size()) return false;
    return true;
}

bool GNNModel::save(const std::string& path) const
{
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;

    const char magic[4] = {'D', 'B', 'R', '0'};
    std::fwrite(magic, 1, 4, f);

    const uint32_t hdr[5] = {
        static_cast<uint32_t>(m_node_feat_dim),
        static_cast<uint32_t>(m_edge_feat_dim),
        static_cast<uint32_t>(m_hidden_dim),
        static_cast<uint32_t>(m_num_classes),
        static_cast<uint32_t>(m_num_layers),
    };
    std::fwrite(hdr, sizeof(uint32_t), 5, f);

    bool ok = write_linear(f, m_node_encoder);
    for (auto& l : m_mp_layers) {
        ok = ok && write_linear(f, l.msg_linear);
        ok = ok && write_linear(f, l.self_linear);
    }
    ok = ok && write_linear(f, m_classifier);

    std::fclose(f);
    return ok;
}

// Reads the magic + 5-uint32 header, leaving `f` positioned at the first
// linear-layer payload. Returns false on bad magic or short read.
static bool read_header(std::FILE* f, uint32_t hdr[5])
{
    char magic[4] = {};
    if (std::fread(magic, 1, 4, f) != 4 ||
        magic[0] != 'D' || magic[1] != 'B' ||
        magic[2] != 'R' || magic[3] != '0')
    {
        return false;
    }
    return std::fread(hdr, sizeof(uint32_t), 5, f) == 5;
}

bool GNNModel::load(const std::string& path)
{
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;

    uint32_t hdr[5] = {};
    if (!read_header(f, hdr)) { std::fclose(f); return false; }
    if (static_cast<int>(hdr[0]) != m_node_feat_dim ||
        static_cast<int>(hdr[1]) != m_edge_feat_dim ||
        static_cast<int>(hdr[2]) != m_hidden_dim    ||
        static_cast<int>(hdr[3]) != m_num_classes   ||
        static_cast<int>(hdr[4]) != m_num_layers)
    {
        std::fclose(f);
        return false;
    }

    bool ok = read_linear(f, m_node_encoder);
    for (auto& l : m_mp_layers) {
        ok = ok && read_linear(f, l.msg_linear);
        ok = ok && read_linear(f, l.self_linear);
    }
    ok = ok && read_linear(f, m_classifier);

    std::fclose(f);
    return ok;
}

bool GNNModel::load_auto_init(const std::string& path, std::mt19937& rng)
{
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;

    uint32_t hdr[5] = {};
    if (!read_header(f, hdr)) { std::fclose(f); return false; }
    std::fclose(f);

    init(static_cast<int>(hdr[0]),
         static_cast<int>(hdr[1]),
         static_cast<int>(hdr[2]),
         static_cast<int>(hdr[3]),
         static_cast<int>(hdr[4]),
         rng);
    return load(path);
}

}
