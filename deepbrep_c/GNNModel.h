#pragma once

#include "Tensor.h"
#include "Layers.h"
#include "GraphData.h"

#include <random>
#include <vector>
#include <string>

namespace deepbrep
{

// Encoder -> K x message-passing (with residual) -> linear classifier ->
// softmax. Per-node multi-class output.
class GNNModel
{
public:
    void init(int node_feat_dim, int edge_feat_dim, int hidden_dim,
              int num_classes, int num_layers, std::mt19937& rng);

    // Returns the mean cross-entropy loss over nodes if `compute_loss` is true
    // and `graph.labels` is populated; otherwise 0. Always populates `m_probs`.
    float forward(const GraphData& graph, bool compute_loss = true);

    // Must be called after a forward() that cached state for this graph and
    // after labels are available in `graph.labels`.
    void backward(const GraphData& graph);

    void zero_grad();
    void update(float lr);

    // argmax over the cached probs of the most recent forward(). Caller is
    // responsible for ensuring forward() ran on the same graph.
    std::vector<int> predict_cached(int num_nodes) const;

    // Convenience: forward + argmax in one call. Does not require labels.
    std::vector<int> predict(const GraphData& graph);

    int NodeFeatDim()  const { return m_node_feat_dim; }
    int EdgeFeatDim()  const { return m_edge_feat_dim; }
    int HiddenDim()    const { return m_hidden_dim; }
    int NumClasses()   const { return m_num_classes; }
    int NumLayers()    const { return m_num_layers; }

    const Mat& Probs() const { return m_probs; }

    // Lightweight checkpoint to a binary file. Format is private to this
    // module -- not meant to be cross-version stable.
    bool save(const std::string& path) const;

    // Strict load: model must already be init()'d with matching architecture.
    bool load(const std::string& path);

    // Read architecture from the file header, init() the model accordingly,
    // then load the weights. Use this when loading a checkpoint into a fresh
    // model without knowing its dims up-front (e.g. ves callers).
    bool load_auto_init(const std::string& path, std::mt19937& rng);

private:
    int m_node_feat_dim = 0;
    int m_edge_feat_dim = 0;
    int m_hidden_dim = 0;
    int m_num_classes = 0;
    int m_num_layers = 0;

    Linear              m_node_encoder;
    std::vector<MPLayer> m_mp_layers;
    Linear              m_classifier;

    // Forward-pass caches.
    Mat              m_encoded_h;
    std::vector<Mat> m_layer_outputs;
    Mat              m_logits;
    Mat              m_probs;
};

}
