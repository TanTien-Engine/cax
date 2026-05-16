#pragma once

#include "Tensor.h"
#include "GraphData.h"

#include <vector>
#include <random>

namespace deepbrep
{

// Plain affine layer y = x * W + b. Holds parameters, gradient accumulators
// and a vanilla-SGD update. One-sample forward/backward (no batching) since
// the GNN processes nodes one at a time.
struct Linear
{
    Mat W;                    // [in_dim, out_dim]
    Mat dW;                   // gradient accumulator
    std::vector<float> b;     // [out_dim]
    std::vector<float> db;    // gradient accumulator

    int in_dim = 0;
    int out_dim = 0;

    void init(int in_d, int out_d, std::mt19937& rng);

    void forward(const float* in, float* out) const;

    // grad_in may be null when the caller does not need the input gradient.
    void backward(const float* grad_out, const float* input, float* grad_in);

    void zero_grad();
    void update(float lr);
};

// One message-passing layer with edge features. For node i:
//   msg(i) = sum over neighbors j of  msg_linear( concat(h_j, e_ij) )
//   h_i'  = ReLU( self_linear(h_i) + msg(i) )
//
// Edge features are an input only -- there is no gradient w.r.t. them.
struct MPLayer
{
    int hidden_dim = 0;
    int edge_feat_dim = 0;

    Linear msg_linear;        // [hidden_dim + edge_feat_dim] -> hidden_dim
    Linear self_linear;       // hidden_dim -> hidden_dim

    // Cached for backward. Allocated fresh on every forward() call.
    Mat h_in;
    Mat h_pre_relu;
    std::vector<std::vector<float>> edge_msgs;
    std::vector<std::vector<float>> edge_concat;

    void init(int h_dim, int e_dim, std::mt19937& rng);

    // Updates `node_h` in place: input is the previous layer's hidden state,
    // output is the new hidden state (post-ReLU).
    void forward(Mat& node_h, const GraphData& graph);

    // grad_h: in = upstream gradient w.r.t. this layer's output;
    //         out = gradient w.r.t. this layer's input (h_in).
    void backward(Mat& grad_h, const GraphData& graph);

    void zero_grad();
    void update(float lr);
};

}
