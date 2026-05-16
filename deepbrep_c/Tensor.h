#pragma once

#include <vector>
#include <random>

namespace deepbrep
{

// Row-major dense float matrix. Holds layer weights, activations and gradients
// for the from-scratch GNN. Intentionally no SIMD/BLAS: keep dependencies zero.
struct Mat
{
    int rows = 0;
    int cols = 0;
    std::vector<float> data;

    Mat() = default;
    Mat(int r, int c) : rows(r), cols(c), data(static_cast<size_t>(r) * c, 0.0f) {}
    Mat(int r, int c, float val) : rows(r), cols(c), data(static_cast<size_t>(r) * c, val) {}

    float& at(int r, int c)             { return data[r * cols + c]; }
    float  at(int r, int c) const       { return data[r * cols + c]; }

    float* row_ptr(int r)               { return data.data() + r * cols; }
    const float* row_ptr(int r) const   { return data.data() + r * cols; }

    void zero();
    void xavier_init(std::mt19937& rng);
};

// out[j] = sum_i in[i] * W(i,j). in: W.rows, out: W.cols.
void vec_mat_mul(const float* in, const Mat& W, float* out);

// out[i] += bias[i]
void vec_add(float* out, const float* bias, int len);

void relu(float* x, int len);

// Sets grad[i] to 0 wherever x[i] <= 0; leaves other entries alone.
void relu_backward(float* grad, const float* x, int len);

// In-place softmax over a length-`len` vector. Numerically stable (max-shift).
void softmax(float* x, int len);

// -log(probs[label]), clamped to avoid log(0).
float cross_entropy(const float* probs, int label);

}
