#include "Layers.h"

#include <cstring>

namespace deepbrep
{

// ============================================================
// Linear
// ============================================================

void Linear::init(int in_d, int out_d, std::mt19937& rng)
{
    in_dim = in_d;
    out_dim = out_d;
    W = Mat(in_d, out_d);
    W.xavier_init(rng);
    dW = Mat(in_d, out_d);
    b.assign(out_d, 0.0f);
    db.assign(out_d, 0.0f);
}

void Linear::forward(const float* in, float* out) const
{
    vec_mat_mul(in, W, out);
    vec_add(out, b.data(), out_dim);
}

void Linear::backward(const float* grad_out, const float* input, float* grad_in)
{
    // dW += outer(input, grad_out)
    for (int i = 0; i < in_dim; ++i) {
        const float xi = input[i];
        float* dw_row = dW.row_ptr(i);
        for (int j = 0; j < out_dim; ++j) {
            dw_row[j] += xi * grad_out[j];
        }
    }
    // db += grad_out
    for (int j = 0; j < out_dim; ++j) {
        db[j] += grad_out[j];
    }
    // grad_in = grad_out * W^T
    if (grad_in) {
        for (int i = 0; i < in_dim; ++i) {
            const float* w_row = W.row_ptr(i);
            float sum = 0.0f;
            for (int j = 0; j < out_dim; ++j) {
                sum += grad_out[j] * w_row[j];
            }
            grad_in[i] = sum;
        }
    }
}

void Linear::zero_grad()
{
    dW.zero();
    std::fill(db.begin(), db.end(), 0.0f);
}

void Linear::update(float lr)
{
    const int n = in_dim * out_dim;
    for (int i = 0; i < n; ++i) {
        W.data[i] -= lr * dW.data[i];
    }
    for (int j = 0; j < out_dim; ++j) {
        b[j] -= lr * db[j];
    }
}

// ============================================================
// MPLayer
// ============================================================

void MPLayer::init(int h_dim, int e_dim, std::mt19937& rng)
{
    hidden_dim = h_dim;
    edge_feat_dim = e_dim;
    msg_linear.init(h_dim + e_dim, h_dim, rng);
    self_linear.init(h_dim, h_dim, rng);
}

void MPLayer::forward(Mat& node_h, const GraphData& graph)
{
    const int N = graph.num_nodes;

    h_in = node_h;
    h_pre_relu = Mat(N, hidden_dim);

    edge_msgs.resize(graph.adj_list.size());
    edge_concat.resize(graph.adj_list.size());

    msg_scale.resize(N);
    for (int i = 0; i < N; ++i) {
        self_linear.forward(h_in.row_ptr(i), h_pre_relu.row_ptr(i));

        const int start = graph.adj_offset[i];
        const int end   = graph.adj_offset[i + 1];
        const int degree = end - start;
        msg_scale[i] = degree > 0 ? 1.0f / static_cast<float>(degree) : 0.0f;

        for (int k = start; k < end; ++k) {
            const int j    = graph.adj_list[k].neighbor;
            const int eidx = graph.adj_list[k].edge_idx;

            const int cat_dim = hidden_dim + edge_feat_dim;
            edge_concat[k].resize(cat_dim);
            std::memcpy(edge_concat[k].data(),
                        h_in.row_ptr(j),
                        hidden_dim * sizeof(float));
            std::memcpy(edge_concat[k].data() + hidden_dim,
                        graph.edge_features.row_ptr(eidx),
                        edge_feat_dim * sizeof(float));

            edge_msgs[k].resize(hidden_dim);
            msg_linear.forward(edge_concat[k].data(), edge_msgs[k].data());

            // Accumulate mean-scaled message: self(h_i) + mean(msgs)
            float* dst = h_pre_relu.row_ptr(i);
            const float* src = edge_msgs[k].data();
            for (int d = 0; d < hidden_dim; ++d) {
                dst[d] += src[d] * msg_scale[i];
            }
        }

        std::memcpy(node_h.row_ptr(i),
                    h_pre_relu.row_ptr(i),
                    hidden_dim * sizeof(float));
        relu(node_h.row_ptr(i), hidden_dim);
    }
}

void MPLayer::backward(Mat& grad_h, const GraphData& graph)
{
    const int N = graph.num_nodes;

    for (int i = 0; i < N; ++i) {
        relu_backward(grad_h.row_ptr(i), h_pre_relu.row_ptr(i), hidden_dim);
    }

    Mat grad_h_in(N, hidden_dim);

    std::vector<float> grad_self_in(hidden_dim);
    std::vector<float> grad_concat(hidden_dim + edge_feat_dim);
    std::vector<float> grad_msg(hidden_dim);

    for (int i = 0; i < N; ++i) {
        self_linear.backward(grad_h.row_ptr(i),
                             h_in.row_ptr(i),
                             grad_self_in.data());
        float* gh_in_i = grad_h_in.row_ptr(i);
        for (int d = 0; d < hidden_dim; ++d) {
            gh_in_i[d] += grad_self_in[d];
        }

        const int start = graph.adj_offset[i];
        const int end   = graph.adj_offset[i + 1];
        const float scale = msg_scale[i];
        for (int k = start; k < end; ++k) {
            const int j = graph.adj_list[k].neighbor;

            // Gradient through mean: d(loss)/d(msg_k) = grad_h[i] * scale
            for (int d = 0; d < hidden_dim; ++d) {
                grad_msg[d] = grad_h.row_ptr(i)[d] * scale;
            }

            msg_linear.backward(grad_msg.data(),
                                edge_concat[k].data(),
                                grad_concat.data());

            float* gh_in_j = grad_h_in.row_ptr(j);
            for (int d = 0; d < hidden_dim; ++d) {
                gh_in_j[d] += grad_concat[d];
            }
        }
    }

    grad_h = std::move(grad_h_in);
}

void MPLayer::zero_grad()
{
    msg_linear.zero_grad();
    self_linear.zero_grad();
}

void MPLayer::update(float lr)
{
    msg_linear.update(lr);
    self_linear.update(lr);
}

}
