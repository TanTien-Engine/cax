#include "DatasetReader.h"
#include "GNNModel.h"
#include "Trainer.h"
#include "FeatureLabels.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

static void print_usage(const char* prog)
{
    std::printf("Usage: %s <dataset.dset> [options]\n", prog);
    std::printf("Options:\n");
    std::printf("  --output, -o <path>    Save trained weights (default: model.bin)\n");
    std::printf("  --epochs <n>           Number of epochs (default: 100)\n");
    std::printf("  --lr <f>               Learning rate (default: 0.01)\n");
    std::printf("  --hidden <n>           Hidden dimension (default: 64)\n");
    std::printf("  --layers <n>           Message-passing layers (default: 4)\n");
    std::printf("  --seed <n>             Random seed (default: 42)\n");
    std::printf("  --val-split <f>        Validation fraction (default: 0.1)\n");
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    std::string dataset_path = argv[1];
    std::string output_path  = "model.bin";
    int   epochs     = 100;
    float lr         = 0.01f;
    int   hidden_dim = 64;
    int   num_layers = 4;
    int   seed       = 42;
    float val_split  = 0.1f;

    for (int i = 2; i < argc; ++i) {
        if ((std::strcmp(argv[i], "--output") == 0 || std::strcmp(argv[i], "-o") == 0) && i + 1 < argc) {
            output_path = argv[++i];
        } else if (std::strcmp(argv[i], "--epochs") == 0 && i + 1 < argc) {
            epochs = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--lr") == 0 && i + 1 < argc) {
            lr = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--hidden") == 0 && i + 1 < argc) {
            hidden_dim = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--layers") == 0 && i + 1 < argc) {
            num_layers = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            seed = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--val-split") == 0 && i + 1 < argc) {
            val_split = static_cast<float>(std::atof(argv[++i]));
        } else {
            std::fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    std::printf("Loading dataset: %s\n", dataset_path.c_str());

    deepbrep::DatasetHeader header;
    std::vector<deepbrep::GraphData> all_data;
    if (!deepbrep::read_dataset(dataset_path, header, all_data)) {
        std::fprintf(stderr, "Error: failed to read dataset '%s'\n", dataset_path.c_str());
        return 1;
    }

    std::printf("  Samples:       %u\n", header.num_samples);
    std::printf("  Node feat dim: %u\n", header.node_feat_dim);
    std::printf("  Edge feat dim: %u\n", header.edge_feat_dim);
    std::printf("  Num classes:   %u\n", header.num_classes);

    if (all_data.empty()) {
        std::fprintf(stderr, "Error: dataset is empty\n");
        return 1;
    }

    std::mt19937 rng(static_cast<uint32_t>(seed));

    // Shuffle then split into train / val.
    std::shuffle(all_data.begin(), all_data.end(), rng);

    int val_count = static_cast<int>(all_data.size() * val_split);
    if (val_count < 1 && all_data.size() > 1) val_count = 1;
    int train_count = static_cast<int>(all_data.size()) - val_count;

    std::vector<deepbrep::GraphData> train_data(
        all_data.begin(), all_data.begin() + train_count);
    std::vector<deepbrep::GraphData> val_data(
        all_data.begin() + train_count, all_data.end());

    std::printf("  Train samples: %d\n", train_count);
    std::printf("  Val samples:   %d\n", val_count);
    std::printf("Config:\n");
    std::printf("  Hidden dim:    %d\n", hidden_dim);
    std::printf("  MP layers:     %d\n", num_layers);
    std::printf("  Epochs:        %d\n", epochs);
    std::printf("  Learning rate: %.4f\n", lr);
    std::printf("  Seed:          %d\n", seed);
    std::printf("---\n");

    deepbrep::GNNModel model;
    model.init(static_cast<int>(header.node_feat_dim),
               static_cast<int>(header.edge_feat_dim),
               hidden_dim,
               static_cast<int>(header.num_classes),
               num_layers,
               rng);

    deepbrep::TrainConfig cfg;
    cfg.epochs        = epochs;
    cfg.learning_rate = lr;
    cfg.shuffle       = true;
    cfg.verbose       = true;

    deepbrep::Trainer trainer(model, cfg, rng);

    float best_val_acc = 0.0f;
    int best_epoch = 0;

    for (int e = 0; e < epochs; ++e) {
        auto stat = trainer.train_one_epoch(train_data, e);

        // Evaluate on validation set.
        float val_correct = 0;
        float val_total = 0;
        for (auto& vg : val_data) {
            auto preds = model.predict(vg);
            for (int i = 0; i < vg.num_nodes; ++i) {
                if (preds[i] == vg.labels[i]) val_correct += 1.0f;
                val_total += 1.0f;
            }
        }
        float val_acc = val_total > 0 ? val_correct / val_total : 0.0f;

        std::printf("Epoch %3d  loss=%.4f  train_acc=%.3f  val_acc=%.3f",
                    e + 1, stat.loss, stat.accuracy, val_acc);

        if (val_acc > best_val_acc) {
            best_val_acc = val_acc;
            best_epoch = e + 1;
            model.save(output_path);
            std::printf("  * saved");
        }
        std::printf("\n");
    }

    std::printf("---\nBest val_acc=%.3f at epoch %d\n", best_val_acc, best_epoch);
    std::printf("Model saved to: %s\n", output_path.c_str());
    return 0;
}
