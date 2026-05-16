#include "deepbrep_c/wrap_DeepBrep.h"
#include "deepbrep_c/GNNModel.h"
#include "deepbrep_c/GraphData.h"
#include "deepbrep_c/BRepGraphBuilder.h"

#include <partgraph_c/TopoShape.h>
#include <wrapper/Proxy.h>

#include <cstring>
#include <memory>
#include <random>
#include <string>

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
    auto s = ((wrapper::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;

    std::vector<int> preds;
    if (r->loaded && s) {
        deepbrep::GraphData g = deepbrep::BRepGraphBuilder::Build(s);
        if (g.num_nodes > 0) {
            preds = r->model.predict(g);
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

}  // anonymous namespace

namespace deepbrep
{

VesselForeignMethodFn DeepBrepBindMethod(const char* signature)
{
    if (strcmp(signature, "FeatureRecognizer.predict(_)")  == 0) return w_FeatureRecognizer_predict;
    if (strcmp(signature, "FeatureRecognizer.is_loaded()") == 0) return w_FeatureRecognizer_is_loaded;
    return nullptr;
}

void DeepBrepBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
    if (strcmp(class_name, "FeatureRecognizer") == 0) {
        methods->allocate = w_FeatureRecognizer_allocate;
        methods->finalize = w_FeatureRecognizer_finalize;
        return;
    }
}

}
