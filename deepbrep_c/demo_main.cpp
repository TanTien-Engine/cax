// Toy end-to-end demo: train the GNN on a synthetic box-with-hole graph and
// print per-face predictions. Intended as a smoke test for the core math --
// no OCCT involvement.

#include "GNNModel.h"
#include "Trainer.h"
#include "ToyDataset.h"
#include "FeatureLabels.h"

#include <cstdio>
#include <random>
#include <vector>

int main()
{
    std::mt19937 rng(42);

    const int HIDDEN_DIM    = 32;
    const int NUM_LAYERS    = 4;

    deepbrep::GNNModel model;
    model.init(deepbrep::kNodeFeatDim,
               deepbrep::kEdgeFeatDim,
               HIDDEN_DIM,
               deepbrep::kNumFaceClasses,
               NUM_LAYERS,
               rng);

    std::vector<deepbrep::GraphData> train_data;
    train_data.reserve(50);
    for (int i = 0; i < 50; ++i) {
        train_data.push_back(deepbrep::make_box_with_hole_graph(rng));
    }

    std::printf("deepbrep toy demo\n");
    std::printf("  Node features: %d\n",  deepbrep::kNodeFeatDim);
    std::printf("  Edge features: %d\n",  deepbrep::kEdgeFeatDim);
    std::printf("  Hidden dim:    %d\n",  HIDDEN_DIM);
    std::printf("  MP layers:     %d\n",  NUM_LAYERS);
    std::printf("  Classes:       %d\n",  deepbrep::kNumFaceClasses);
    std::printf("  Train samples: %zu\n", train_data.size());
    std::printf("---\n");

    deepbrep::TrainConfig cfg;
    cfg.epochs        = 100;
    cfg.learning_rate = 0.01f;
    cfg.verbose       = true;

    deepbrep::Trainer trainer(model, cfg, rng);
    trainer.train(train_data);

    std::printf("\n--- Inference ---\n");
    auto test_graph = deepbrep::make_box_with_hole_graph(rng);
    auto preds = model.predict(test_graph);
    for (int i = 0; i < test_graph.num_nodes; ++i) {
        const int pred = preds[i];
        const int actual = test_graph.labels[i];
        std::printf("Face %d: predicted=%s, actual=%s %s\n",
                    i,
                    deepbrep::face_class_name(pred),
                    deepbrep::face_class_name(actual),
                    pred == actual ? "OK" : "WRONG");
    }
    return 0;
}
