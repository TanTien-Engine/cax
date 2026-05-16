#include <catch2/catch_test_macros.hpp>

#include "GNNModel.h"
#include "Trainer.h"
#include "FeatureLabels.h"
#include "ToyDataset.h"

#include <random>
#include <vector>

using namespace deepbrep;

// A long-running smoke test: train on the same synthetic graph repeated 30
// times and assert that loss strictly decreases and final accuracy is high.
// Marked as a Catch2 [.slow] tag so it can be skipped in fast CI runs.
TEST_CASE("Trainer drives loss down on toy box-with-hole task", "[gnn][.slow]")
{
    std::mt19937 rng(101);
    GNNModel model;
    model.init(kNodeFeatDim, kEdgeFeatDim, 32, kNumFaceClasses, 4, rng);

    std::vector<GraphData> train;
    train.reserve(30);
    for (int i = 0; i < 30; ++i) {
        train.push_back(make_box_with_hole_graph(rng));
    }

    TrainConfig cfg;
    cfg.epochs        = 80;
    cfg.learning_rate = 0.01f;
    cfg.shuffle       = true;
    cfg.verbose       = false;

    Trainer trainer(model, cfg, rng);
    auto history = trainer.train(train);

    REQUIRE(history.size() == 80);
    // Loss must improve over the run.
    CHECK(history.back().loss < history.front().loss);
    // And accuracy should be near-perfect on the synthetic task.
    CHECK(history.back().accuracy > 0.9f);
}
