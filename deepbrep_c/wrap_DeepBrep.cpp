#include "deepbrep_c/wrap_DeepBrep.h"
#include "deepbrep_c/GNNModel.h"
#include "deepbrep_c/GraphData.h"
#include "deepbrep_c/BRepGraphBuilder.h"
#include "deepbrep_c/FeatureLabels.h"

#include "deepbrep_data_gen/OpNameVocabulary.h"
#include "deepbrep_data_gen/HistoryGraphLabeler.h"
#include "deepbrep_data_gen/DatasetWriter.h"

#include "breptopo_c/TopoNaming.h"
#include "breptopo_c/CompGraph.h"
#include "breptopo_c/HistGraph.h"
#include "brepkit_c/TopoShape.h"
#include "brepkit_c/GlobalConfig.h"

#include <wrapper/Proxy.h>

#include <cstring>
#include <ctime>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

// FeatureRecognizer wraps a trained GNNModel + RNG (the RNG is used only to
// re-init layer shapes; weights come from the file). The `loaded` flag lets
// predict() bail out cleanly when the checkpoint is missing or corrupt.
struct Recognizer
{
    deepbrep::GNNModel model;
    std::mt19937       rng{42};
    bool               loaded = false;
};

void w_FeatureRecognizer_allocate()
{
    auto proxy = (wrapper::Proxy<Recognizer>*)
        ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<Recognizer>));
    proxy->obj = std::make_shared<Recognizer>();

    const char* path = ves_tostring(1);
    if (path && path[0] != '\0') {
        proxy->obj->loaded =
            proxy->obj->model.load_auto_init(path, proxy->obj->rng);
    }
}

int w_FeatureRecognizer_finalize(void* data)
{
    auto proxy = (wrapper::Proxy<Recognizer>*)data;
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<Recognizer>);
}

void w_FeatureRecognizer_predict()
{
    auto r = ((wrapper::Proxy<Recognizer>*)ves_toforeign(0))->obj;
    auto s = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;

    std::vector<int> preds;
    if (r->loaded && s) {
        deepbrep::GraphData g = deepbrep::BRepGraphBuilder::Build(s);
        if (g.num_nodes > 0) {
            preds = r->model.predict(g);

            auto& probs = r->model.Probs();
            for (int i = 0; i < g.num_nodes; ++i) {
                float conf = probs.at(i, preds[i]);
                std::printf("  face %d: %s (%.1f%%)\n",
                            i, deepbrep::face_class_name(preds[i]), conf * 100.0f);
            }
        }
    }

    const int num = static_cast<int>(preds.size());
    ves_pop(ves_argnum());
    ves_newlist(num);
    for (int i = 0; i < num; ++i) {
        ves_pushnumber(static_cast<double>(preds[i]));
        ves_seti(-2, i);
        ves_pop(1);
    }
}

void w_FeatureRecognizer_is_loaded()
{
    auto r = ((wrapper::Proxy<Recognizer>*)ves_toforeign(0))->obj;
    ves_set_boolean(0, r->loaded);
}

// DataExporter holds shared state (vocab) and exposes a one-shot export
// method that takes the current model shape and writes (graph, labels) to
// a dataset file.
struct DataExporter
{
    std::shared_ptr<deepbrep_data_gen::OpNameVocabulary> vocab;
};

void w_DataExporter_allocate()
{
    auto proxy = (wrapper::Proxy<DataExporter>*)
        ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<DataExporter>));
    proxy->obj = std::make_shared<DataExporter>();
    proxy->obj->vocab = std::make_shared<deepbrep_data_gen::OpNameVocabulary>();
}

int w_DataExporter_finalize(void* data)
{
    auto proxy = (wrapper::Proxy<DataExporter>*)data;
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<DataExporter>);
}

void w_DataExporter_export()
{
    auto exporter = ((wrapper::Proxy<DataExporter>*)ves_toforeign(0))->obj;
    auto shape    = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    const char* filepath = ves_tostring(2);

    bool ok = false;

    if (shape && filepath && filepath[0] != '\0')
    {
        auto cfg = brepkit::GlobalConfig::Instance();
        auto tn  = cfg->GetTopoNaming();
        auto cg  = cfg->GetCompGraph();

        std::unordered_map<uint32_t, std::string> op_map;
        if (cg) {
            op_map = cg->GetIROpIdMap();
        }

        auto face_hg = tn ? tn->GetFaceGraph() : nullptr;

        if (face_hg)
        {
            deepbrep_data_gen::HistoryGraphLabeler labeler(
                face_hg,
                [&op_map](uint32_t op_id) -> std::string {
                    auto it = op_map.find(op_id);
                    return it == op_map.end() ? std::string() : it->second;
                },
                exporter->vocab);

            auto labels = labeler.Label(*shape);
            auto graph  = deepbrep::BRepGraphBuilder::Build(shape);

            if (graph.num_nodes > 0)
            {
                deepbrep_data_gen::DatasetWriter writer(
                    filepath,
                    deepbrep::kNodeFeatDim,
                    deepbrep::kEdgeFeatDim,
                    deepbrep::kNumFaceClasses);

                ok = writer.Append(graph, labels);
                writer.Close();
            }
        }
    }

    ves_set_boolean(0, ok);
}

void w_DataExporter_export_graph_only()
{
    auto shape    = ((wrapper::Proxy<brepkit::TopoShape>*)ves_toforeign(1))->obj;
    const char* filepath = ves_tostring(2);

    bool ok = false;

    if (shape && filepath && filepath[0] != '\0')
    {
        auto graph = deepbrep::BRepGraphBuilder::Build(shape);
        if (graph.num_nodes > 0) {
            ok = deepbrep::write_graph_data(filepath, graph);
        }
    }

    ves_set_boolean(0, ok);
}

void w_DataExporter_export_augmented()
{
    auto exporter = ((wrapper::Proxy<DataExporter>*)ves_toforeign(0))->obj;
    const char* filepath = ves_tostring(1);
    int num_samples = static_cast<int>(ves_tonumber(2));
    double noise_ratio = ves_tonumber(3);

    int ok_count = 0;

    if (!filepath || filepath[0] == '\0' || num_samples <= 0) {
        ves_set_number(0, 0);
        return;
    }

    auto cfg = brepkit::GlobalConfig::Instance();
    auto tn  = cfg->GetTopoNaming();
    auto cg  = cfg->GetCompGraph();

    if (!cg || !tn) {
        ves_set_number(0, 0);
        return;
    }

    struct ParamInfo 
    {
        int    step_id;
        double original;
        std::string name;
    };
    std::vector<ParamInfo> params;

    const auto& history = cg->GetHistory();
    for (auto& step : history.Steps()) {

        if (std::holds_alternative<double>(step.imm) && step.desc != "uid") {
            params.push_back({step.step_id, std::get<double>(step.imm),step.desc });
        }
    }

    if (params.empty()) {
        ves_set_number(0, 0);
        return;
    }

    int last_op_step = -1;
    for (int i = static_cast<int>(history.Steps().size()) - 1; i >= 0; --i) {
        if (!history.Steps()[i].op_name.empty()) {
            last_op_step = history.Steps()[i].step_id;
            break;
        }
    }
    if (last_op_step < 0) {
        ves_set_number(0, 0);
        return;
    }

    deepbrep_data_gen::DatasetWriter writer(
        filepath,
        deepbrep::kNodeFeatDim,
        deepbrep::kEdgeFeatDim,
        deepbrep::kNumFaceClasses);

    std::mt19937 rng(static_cast<uint32_t>(std::time(nullptr)));
    std::uniform_real_distribution<double> dist(-noise_ratio, noise_ratio);

    for (int s = 0; s < num_samples; ++s)
    {
        for (auto& p : params) {
            double perturbed = p.original * (1.0 + dist(rng));
            if (p.original > 0 && perturbed <= 0) perturbed = p.original * 0.1;
            cg->UpdateConst(p.step_id, perturbed);
        }

        cg->Lower();
        auto val = cg->Eval(last_op_step);

        if (!std::holds_alternative<breptopo::ShapeVal>(val)) continue;
        auto& sv = std::get<breptopo::ShapeVal>(val);
        if (!sv.shape) continue;

        auto op_map = cg->GetIROpIdMap();

        auto face_hg = tn->GetFaceGraph();
        if (!face_hg) continue;

        deepbrep_data_gen::HistoryGraphLabeler labeler(
            face_hg,
            [&op_map](uint32_t op_id) -> std::string {
                auto it = op_map.find(op_id);
                return it == op_map.end() ? std::string() : it->second;
            },
            exporter->vocab);

        auto labels = labeler.Label(*sv.shape);
        auto graph  = deepbrep::BRepGraphBuilder::Build(sv.shape);

        if (graph.num_nodes > 0 && writer.Append(graph, labels)) {
            ++ok_count;
        }
    }

    for (auto& p : params) {
        cg->UpdateConst(p.step_id, p.original);
    }
    cg->Lower();
    cg->Eval(last_op_step);

    writer.Close();
    ves_set_number(0, static_cast<double>(ok_count));
}

}  // anonymous namespace

namespace deepbrep
{

VesselForeignMethodFn DeepBrepBindMethod(const char* signature)
{
    if (strcmp(signature, "FeatureRecognizer.predict(_)")  == 0) return w_FeatureRecognizer_predict;
    if (strcmp(signature, "FeatureRecognizer.is_loaded()") == 0) return w_FeatureRecognizer_is_loaded;
    if (strcmp(signature, "DataExporter.do_export(_,_)") == 0) return w_DataExporter_export;
    if (strcmp(signature, "DataExporter.export_graph_only(_,_)") == 0) return w_DataExporter_export_graph_only;
    if (strcmp(signature, "DataExporter.export_augmented(_,_,_)") == 0) return w_DataExporter_export_augmented;
    return nullptr;
}

void DeepBrepBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
    if (strcmp(class_name, "FeatureRecognizer") == 0) {
        methods->allocate = w_FeatureRecognizer_allocate;
        methods->finalize = w_FeatureRecognizer_finalize;
        return;
    }
    if (strcmp(class_name, "DataExporter") == 0) {
        methods->allocate = w_DataExporter_allocate;
        methods->finalize = w_DataExporter_finalize;
        return;
    }
}

}
