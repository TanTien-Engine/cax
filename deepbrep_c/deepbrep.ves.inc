#define QUOTE(...) #__VA_ARGS__
static const char* deepbrepModuleSource = QUOTE(

// Loads a trained GNN checkpoint and runs per-face feature recognition on a
// TopoShape. Training lives in a separate C++ tool; this is inference-only.
//
// init(weights_path):
//   Reads the architecture from the checkpoint header and loads the weights.
//   Constructor "succeeds" even on a missing/bad file; predict() will then
//   return an empty list. Check return value explicitly if you need to know.
//
// predict(shape):
//   Returns a list of per-face class ids (integers), in the same face order
//   as breptopo::TopoGraphBuilder. Empty list if no weights loaded or the
//   shape has no faces.

foreign class FeatureRecognizer
{
	init(weights_path) {}

	foreign predict(shape)
	foreign is_loaded()
}

foreign class DataExporter
{
	init() {}

	foreign do_export(shape, filepath)
	foreign export_graph_only(shape, filepath)
	foreign export_augmented(filepath, num_samples, noise_ratio)
}

);
