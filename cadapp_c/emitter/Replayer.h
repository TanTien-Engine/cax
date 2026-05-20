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
};

struct ReplayResult
{
    bool                                  ok = false;
    std::string                           err_msg;

    // Final (top-of-chain) shape produced by Replay.
    std::shared_ptr<brepkit::TopoShape> shape;

    // Per-feature op_id (0 means skipped). Same length as
    // doc.features.
    std::vector<uint32_t>                 op_ids;

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

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace cadapp
