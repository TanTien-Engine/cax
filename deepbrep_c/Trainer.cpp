#include "Trainer.h"

#include <algorithm>
#include <cstdio>
#include <numeric>

namespace deepbrep
{

EpochStat Trainer::train_one_epoch(std::vector<GraphData>& train_data, int epoch_index)
{
    std::vector<int> order(train_data.size());
    std::iota(order.begin(), order.end(), 0);
    if (m_cfg.shuffle) {
        std::shuffle(order.begin(), order.end(), m_rng);
    }

    float total_loss = 0.0f;
    int   total_nodes = 0;
    int   correct = 0;

    for (int idx : order) {
        GraphData& graph = train_data[idx];

        m_model.zero_grad();
        const float loss = m_model.forward(graph, true);
        total_loss  += loss * graph.num_nodes;
        total_nodes += graph.num_nodes;

        // Pull predictions out of the same forward -- no second pass needed
        // since the cache is still intact going into backward.
        auto preds = m_model.predict_cached(graph.num_nodes);
        for (int i = 0; i < graph.num_nodes; ++i) {
            if (preds[i] == graph.labels[i]) {
                correct++;
            }
        }

        m_model.backward(graph);
        m_model.update(m_cfg.learning_rate);
    }

    EpochStat stat;
    stat.epoch    = epoch_index;
    stat.loss     = total_nodes > 0 ? total_loss / total_nodes : 0.0f;
    stat.accuracy = total_nodes > 0
                    ? static_cast<float>(correct) / total_nodes
                    : 0.0f;
    return stat;
}

std::vector<EpochStat> Trainer::train(std::vector<GraphData>& train_data)
{
    std::vector<EpochStat> history;
    history.reserve(m_cfg.epochs);
    for (int e = 0; e < m_cfg.epochs; ++e) {
        auto stat = train_one_epoch(train_data, e + 1);
        history.push_back(stat);
        if (m_cfg.verbose) {
            std::printf("Epoch %3d | Loss: %.4f | Accuracy: %.2f%%\n",
                        stat.epoch, stat.loss, stat.accuracy * 100.0f);
        }
    }
    return history;
}

}
