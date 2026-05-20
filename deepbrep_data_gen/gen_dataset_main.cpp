// Multi-feature dataset generator for deepbrep GNN training.
//
// Generates parametric CAD models with random combinations of:
//   - Box base (Stock faces)
//   - Through-hole (Cylinder Cut -> Hole faces)
//   - Fillet (rounded edges -> Fillet faces)
//   - Chamfer (beveled edges -> Chamfer faces)
//
// Each sample is a different random combination with different dimensions,
// producing a diverse dataset for the face-classification GNN.

#include "OpNameVocabulary.h"
#include "HistoryGraphLabeler.h"
#include "DatasetWriter.h"

#include "deepbrep_c/BRepGraphBuilder.h"
#include "deepbrep_c/GraphData.h"
#include "deepbrep_c/FeatureLabels.h"

#include "brepgraph_c/TopoNaming.h"
#include "brepgraph_c/HistGraph.h"
#include "brepkit_c/PrimMaker.h"
#include "brepkit_c/TopoAlgo.h"
#include "brepkit_c/TopoShape.h"

#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

using namespace deepbrep_data_gen;

// Collects all edges from a shape as TopoShape wrappers (for Fillet/Chamfer).
std::vector<std::shared_ptr<brepkit::TopoShape>>
collect_edges(const std::shared_ptr<brepkit::TopoShape>& shape)
{
    std::vector<std::shared_ptr<brepkit::TopoShape>> result;
    TopTools_IndexedMapOfShape edge_map;
    TopExp::MapShapes(shape->GetShape(), TopAbs_EDGE, edge_map);
    for (int i = 1; i <= edge_map.Extent(); ++i) {
        result.push_back(std::make_shared<brepkit::TopoShape>(edge_map(i)));
    }
    return result;
}

// Randomly select a subset of edges.
std::vector<std::shared_ptr<brepkit::TopoShape>>
random_edge_subset(const std::vector<std::shared_ptr<brepkit::TopoShape>>& all_edges,
                   int min_count, int max_count, std::mt19937& rng)
{
    if (all_edges.empty()) return {};
    int n = static_cast<int>(all_edges.size());
    int lo = std::min(min_count, n);
    int hi = std::min(max_count, n);
    if (lo > hi) lo = hi;
    std::uniform_int_distribution<int> count_dist(lo, hi);
    int count = count_dist(rng);

    // Shuffle indices and pick first `count`.
    std::vector<int> indices(n);
    for (int i = 0; i < n; ++i) indices[i] = i;
    std::shuffle(indices.begin(), indices.end(), rng);

    std::vector<std::shared_ptr<brepkit::TopoShape>> result;
    for (int i = 0; i < count; ++i) {
        result.push_back(all_edges[indices[i]]);
    }
    return result;
}

// Feature recipe flags -- each sample picks a random subset.
enum Feature : uint32_t
{
    FEAT_HOLE    = 1 << 0,
    FEAT_FILLET  = 1 << 1,
    FEAT_CHAMFER = 1 << 2,
};

bool emit_sample(DatasetWriter& writer,
                 const std::shared_ptr<OpNameVocabulary>& vocab,
                 std::mt19937& rng)
{
    auto tn = std::make_shared<brepgraph::TopoNaming>();
    std::unordered_map<uint32_t, std::string> op_names;

    // --- Base Box ---
    std::uniform_real_distribution<double> dim(10.0, 30.0);
    const double dx = dim(rng);
    const double dy = dim(rng);
    const double dz = dim(rng);

    const uint32_t op_box = tn->NextOpId();
    auto shape = brepkit::PrimMaker::Box(dx, dy, dz, op_box, tn);
    op_names[op_box] = "PrimMaker.Box";

    if (!shape) return false;

    // Decide which features to add (random subset, can be empty = plain box).
    std::uniform_int_distribution<uint32_t> feat_dist(0, 7);
    uint32_t features = feat_dist(rng);

    // --- Hole: cylinder cut through the top face ---
    if (features & FEAT_HOLE) {
        double min_dim = std::min({dx, dy, dz});
        double max_radius = min_dim * 0.3;
        if (max_radius > 1.5) {
            std::uniform_real_distribution<double> r_dist(1.0, max_radius);
            double hole_r = r_dist(rng);
            // Place cylinder taller than the box so it cuts through.
            double cyl_h = dz * 2.0;

            const uint32_t op_cyl = tn->NextOpId();
            auto cylinder = brepkit::PrimMaker::Cylinder(hole_r, cyl_h, op_cyl, tn);
            op_names[op_cyl] = "PrimMaker.Cylinder";

            if (cylinder) {
                // Translate cylinder to a random position on the top face.
                std::uniform_real_distribution<double> pos_x(hole_r + 1.0, dx - hole_r - 1.0);
                std::uniform_real_distribution<double> pos_y(hole_r + 1.0, dy - hole_r - 1.0);
                double cx = (dx > 2 * hole_r + 2.0) ? pos_x(rng) : dx * 0.5;
                double cy = (dy > 2 * hole_r + 2.0) ? pos_y(rng) : dy * 0.5;
                double cz = -dz * 0.5; // start below to cut through

                const uint32_t op_trans = tn->NextOpId();
                cylinder = brepkit::TopoAlgo::Translate(cylinder, cx, cy, cz, op_trans, tn);
                op_names[op_trans] = "PrimMaker.Cylinder"; // translation is part of tool setup

                if (cylinder) {
                    const uint32_t op_cut = tn->NextOpId();
                    auto cut_result = brepkit::TopoAlgo::Cut(shape, cylinder, op_cut, tn);
                    if (cut_result) {
                        shape = cut_result;
                        op_names[op_cut] = "TopoAlgo.Cut";
                    }
                }
            }
        }
    }

    // --- Fillet: round some edges ---
    if (features & FEAT_FILLET) {
        auto edges = collect_edges(shape);
        if (!edges.empty()) {
            auto subset = random_edge_subset(edges, 1, 4, rng);
            std::uniform_real_distribution<double> r_dist(0.5, 2.0);
            double fillet_r = r_dist(rng);

            const uint32_t op_fillet = tn->NextOpId();
            auto filleted = brepkit::TopoAlgo::Fillet(shape, fillet_r, subset, op_fillet, tn);
            if (filleted) {
                shape = filleted;
                op_names[op_fillet] = "TopoAlgo.Fillet";
            }
        }
    }

    // --- Chamfer: bevel some edges ---
    if (features & FEAT_CHAMFER) {
        auto edges = collect_edges(shape);
        if (!edges.empty()) {
            auto subset = random_edge_subset(edges, 1, 3, rng);
            std::uniform_real_distribution<double> d_dist(0.5, 1.5);
            double chamfer_d = d_dist(rng);

            const uint32_t op_chamfer = tn->NextOpId();
            auto chamfered = brepkit::TopoAlgo::Chamfer(shape, chamfer_d, subset, op_chamfer, tn);
            if (chamfered) {
                shape = chamfered;
                op_names[op_chamfer] = "TopoAlgo.Chamfer";
            }
        }
    }

    // --- Label + serialize ---
    HistoryGraphLabeler labeler(
        tn->GetFaceGraph(),
        [&op_names](uint32_t op_id) -> std::string {
            auto it = op_names.find(op_id);
            return it == op_names.end() ? std::string() : it->second;
        },
        vocab);

    auto labels = labeler.Label(*shape);
    auto graph = deepbrep::BRepGraphBuilder::Build(shape);
    if (graph.num_nodes == 0) return false;

    return writer.Append(graph, labels);
}

}  // anonymous

int main(int argc, char** argv)
{
    const std::string out_path  = (argc > 1) ? argv[1] : "dataset.dset";
    const int         n_samples = (argc > 2) ? std::atoi(argv[2]) : 200;
    const uint32_t    seed      = (argc > 3) ? static_cast<uint32_t>(std::atoi(argv[3])) : 1;

    std::printf("[deepbrep_data_gen] Generating %d samples (seed=%u) -> %s\n",
                n_samples, seed, out_path.c_str());

    auto vocab = std::make_shared<OpNameVocabulary>();

    DatasetWriter writer(out_path,
                         deepbrep::kNodeFeatDim,
                         deepbrep::kEdgeFeatDim,
                         deepbrep::kNumFaceClasses);

    std::mt19937 rng(seed);
    int ok = 0;
    int fail = 0;
    for (int i = 0; i < n_samples; ++i) {
        if (emit_sample(writer, vocab, rng)) {
            ++ok;
        } else {
            ++fail;
        }
        if ((i + 1) % 50 == 0) {
            std::printf("  ... %d / %d done (%d failed)\n", i + 1, n_samples, fail);
        }
    }
    writer.Close();

    std::printf("[deepbrep_data_gen] Done: %d succeeded, %d failed, wrote to %s\n",
                ok, fail, out_path.c_str());
    return ok > 0 ? 0 : 1;
}
