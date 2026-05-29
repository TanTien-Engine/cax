#pragma once

#include "cadapp_c/ir/FeatureIR.h"

#include <memory>
#include <string>
#include <vector>

// ============================================================
// cadapp/emitter/Replayer.h
//
// Build a CalcGraph from a DocumentIR. The graph encodes every
// feature as op + const nodes; geometry is materialised lazily by
// brepgraph's evaluator, not by Replayer.
//
// Workflow per feature:
//   1. Sketch          -> $sketch const + 3 Vec3 consts feeding a
//                          "sketch_face" op (cadapp::RegisterSketchOps).
//   2. Sketch-based    -> "prism" / "cut" / "fuse" / ... op consuming
//                          the sketch_face result.
//   3. Fillet/Chamfer/ -> per-ref "resolve_edge_ref" / "resolve_face_ref"
//      Shell             op (cadapp::RegisterResolveOps); the geo
//                          match runs at Eval time, not graph-build
//                          time. The op outputs a ShapeVal whose tag
//                          carries the TopoNaming uid.
//   4. History         -> TopoNaming::Update and VersionTree::Commit
//                          fire from inside the ops during Eval.
//   5. write_back      -> post-loop pass eval's each resolve_*_ref
//                          node and copies the uid back into the
//                          caller's TopoRefIR fields.
//
// The header only depends on cadapp/ir; OCCT stays in the .cpp.
// ============================================================

namespace brepgraph
{
class TopoNaming;
class CalcGraph;
}
namespace brepdb
{
class VersionTree;
}
namespace brepkit
{
class TopoShape;
}

namespace cadapp
{

struct ReplayOptions
{
    // Geometric match tolerance used by TopoRefResolver.
    double topo_tolerance = 1e-3;

    // Whether to commit a VersionTree node after each step.
    bool commit_versions = true;

    // Whether to write the resolved uid back into the
    // FeatureIR::edge_refs / face_refs of the input doc, so a
    // second pass can skip geometric matching.
    bool write_back_resolved = true;

    // Whether construction geometry participates in profile / wire
    // building (default false matches project convention).
    bool profile_uses_construction = false;

    // Whether to append a UnifySameDomain "refine" op after each
    // primitive-producing feature (Pad / Pocket = Extrude, Revolve,
    // Loft, Sweep). FreeCAD's PartDesign::Pad does this by default
    // via `refineShapeIfActive`.
    //
    // Default OFF: enabling this on Page_015 introduced a visible
    // "broken face" regression vs the pre-1b9aec5e baseline, while
    // the matching fix (body_n = last_node above the resolver) is
    // independent and still in effect. If you want FreeCAD-style
    // topology cleanup at the cost of that regression, opt-in
    // per-call.
    bool refine_after_primitive = false;

    // Analysis-only: build the CalcGraph and resolve the live output
    // candidates, but DO NOT materialise any geometry (skips the
    // authored-shape eager-eval fallback and the final per-part eval).
    // ReplayResult::live_feat_ids is filled with the surviving output
    // candidates in emission order; ReplayResult::shape / parts stay
    // empty. Used by ReplayParts() to discover, without paying the
    // geometry cost, exactly which features are top-level parts so it
    // can split the document and replay each part independently.
    bool analyze_only = false;
};

// One emitted top-level shape plus the appearance the source GUI
// authored for it. The Replayer fills one ReplayPart per surviving
// output candidate (the same set it folds into ReplayResult::shape),
// so a caller that wants per-shape rendering -- e.g. FreeCAD-style
// per-part transparency -- can draw the parts individually instead
// of the merged compound. transparency is 0 (opaque) .. 1 (fully
// transparent), mirrored from FeatureIR::MaterialIR.
struct ReplayPart
{
    std::shared_ptr<brepkit::TopoShape> shape;
    double                              transparency = 0.0;
    uint32_t                           feat_id      = 0;
};

struct ReplayResult
{
    bool                                  ok = false;
    std::string                           err_msg;

    // Final (top-of-chain) shape produced by Replay. When there is
    // more than one live output candidate this is their compound;
    // ReplayResult::parts carries the same shapes split out with
    // per-part appearance.
    std::shared_ptr<brepkit::TopoShape> shape;

    // Per-emitted-shape split of `shape`, with source appearance.
    // One entry per surviving output candidate, in the same order
    // they were added to the compound. Empty only when no candidate
    // produced a shape.
    std::vector<ReplayPart>               parts;

    // Per-feature op_id (0 means skipped). Same length as
    // doc.features.
    std::vector<uint32_t>                 op_ids;

    // feat_id of each surviving top-level output candidate, in
    // emission order (same order as `parts`). Always populated by
    // Replay, including analyze_only runs where `parts` is empty.
    // ReplayParts() reads this to know which features are top-level
    // parts so it can split the document per part.
    std::vector<uint32_t>                 live_feat_ids;

    // History / naming structures created by Replay. The caller
    // forwards them to BrepDB::Flush().
    std::shared_ptr<brepgraph::TopoNaming> naming;
    std::shared_ptr<brepdb::VersionTree>  vtree;

    // Computation graph built during replay (persistent).
    std::shared_ptr<brepgraph::CalcGraph>  calc_graph;
};

class Replayer
{
public:
    Replayer();
    ~Replayer();

    Replayer(const Replayer&)            = delete;
    Replayer& operator=(const Replayer&) = delete;

    // Reuse an external naming / version tree. When unset Replay
    // allocates fresh ones.
    void SetNaming(const std::shared_ptr<brepgraph::TopoNaming>& tn);
    void SetVersionTree(const std::shared_ptr<brepdb::VersionTree>& vt);

    // Main entry. doc is in/out: when write_back_resolved is true
    // the TopoRefIR uid / topo_index fields get filled in.
    bool Replay(DocumentIR& doc, const ReplayOptions& opt, ReplayResult& out);

    // Per-part replay: split `doc` into one independent sub-document
    // per top-level output part (each carrying just that part's
    // feature closure), replay each through its OWN Replayer / CalcGraph
    // / TopoNaming, and merge the resulting shapes back into `out`
    // (out.parts in document order, out.shape their compound).
    //
    // Because every part is fully isolated, the per-part replays can run
    // concurrently: when `parallel` is true they are dispatched across a
    // thread pool; the merged result is geometry-identical to a serial
    // Replay (only the work is spread over cores). Intended for the
    // one-shot load path -- naming / version state is per-part and not
    // persisted, so callers that need a coherent TopoNaming / VersionTree
    // (write_back_resolved / commit_versions) must use Replay instead.
    bool ReplayParts(DocumentIR& doc, const ReplayOptions& opt,
                     ReplayResult& out, bool parallel);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace cadapp
