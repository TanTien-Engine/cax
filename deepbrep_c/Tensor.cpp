#include "Tensor.h"

#include <algorithm>
#include <cmath>

namespace deepbrep
{

void Mat::zero()
{
    std::fill(data.begin(), data.end(), 0.0f);
}

void Mat::xavier_init(std::mt19937& rng)
{
    const float limit = std::sqrt(6.0f / static_cast<float>(rows + cols));
    std::uniform_real_distribution<float> dist(-limit, limit);
    for (auto& v : data) {
        v = dist(rng);
    }
}

void vec_mat_mul(const float* in, const Mat& W, float* out)
{
    for (int j = 0; j < W.cols; ++j) {
        float sum = 0.0f;
        for (int i = 0; i < W.rows; ++i) {
            sum += in[i] * W.at(i, j);
        }
        out[j] = sum;
    }
}

void vec_add(float* out, const float* bias, int len)
{
    for (int i = 0; i < len; ++i) {
        out[i] += bias[i];
    }
}

void relu(float* x, int len)
{
    for (int i = 0; i < len; ++i) {
        if (x[i] < 0.0f) {
            x[i] = 0.0f;
        }
    }
}

void relu_backward(float* grad, const float* x, int len)
{
    for (int i = 0; i < len; ++i) {
        if (x[i] <= 0.0f) {
            grad[i] = 0.0f;
        }
    }
}

void softmax(float* x, int len)
{
    const float max_val = *std::max_element(x, x + len);
    float sum = 0.0f;
    for (int i = 0; i < len; ++i) {
        x[i] = std::exp(x[i] - max_val);
        sum += x[i];
    }
    const float inv = 1.0f / sum;
    for (int i = 0; i < len; ++i) {
        x[i] *= inv;
    }
}

float cross_entropy(const float* probs, int label)
{
    const float p = std::max(probs[label], 1e-7f);
    return -std::log(p);
}

}
