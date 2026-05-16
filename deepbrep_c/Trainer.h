#pragma once

#include "GNNModel.h"
#include "GraphData.h"

#include <random>
#include <vector>

namespace deepbrep
{

struct TrainConfig
{
    int   epochs = 100;
    float learning_rate = 0.01f;
    bool  shuffle = true;
    bool  verbose = true;
};

struct EpochStat
{
    int   epoch = 0;
    float loss = 0.0f;
    float accuracy = 0.0f;
};

class Trainer
{
public:
    Trainer(GNNModel& model, TrainConfig cfg, std::mt19937& rng)
        : m_model(model), m_cfg(cfg), m_rng(rng) {}

    // Runs `cfg.epochs` passes of SGD over `train_data`. Returns per-epoch
    // stats. Each graph in train_data must carry valid `labels`.
    std::vector<EpochStat> train(std::vector<GraphData>& train_data);

    // Single-epoch step (one full pass over the dataset). Exposed for callers
    // that want their own outer loop -- e.g. early stopping or eval-on-val.
    EpochStat train_one_epoch(std::vector<GraphData>& train_data, int epoch_index);

private:
    GNNModel&     m_model;
    TrainConfig   m_cfg;
    std::mt19937& m_rng;
};

}
