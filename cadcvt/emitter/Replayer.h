#pragma once

#include "cadcvt/ir/FeatureIR.h"

#include <memory>
#include <string>
#include <vector>

// ============================================================
// cadcvt/emitter/Replayer.h
//
// Rebuild a DocumentIR on the project's OCCT kernel.
//
// Workflow per feature:
//   1. Reconstruct the sketch
//      (SketchBridge::ImportToScene -> Scene::Solve).
//   2. Lift the solved 2D wire to a 3D wire + face on the sketch
//      plane.
//   3. Invoke the matching partgraph::TopoAlgo operator
//      (Prism / Cut / Fillet / ...).
//   4. Register history via TopoNaming::Update and
//      VersionTree::Commit.
//   5. Use TopoRefResolver to resolve any TopoRefIR that does not
//      yet carry a uid, then write it back.
//
// The header only depends on cadcvt/ir; OCCT stays in the .cpp.
// ============================================================

namespace breptopo
{
class TopoNaming;
}
namespace brepdb
{
class VersionTree;
}
namespace partgraph
{
class TopoShape;
}

namespace cadcvt
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
    std::shared_ptr<partgraph::TopoShape> shape;

    // Per-feature op_id (0 means skipped). Same length as
    // doc.features.
    std::vector<uint32_t>                 op_ids;

    // History / naming structures created by Replay. The caller
    // forwards them to BrepDB::Flush().
    std::shared_ptr<breptopo::TopoNaming> naming;
    std::shared_ptr<brepdb::VersionTree>  vtree;
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
    void SetNaming(const std::shared_ptr<breptopo::TopoNaming>& tn);
    void SetVersionTree(const std::shared_ptr<brepdb::VersionTree>& vt);

    // Main entry. doc is in/out: when write_back_resolved is true
    // the TopoRefIR uid / topo_index fields get filled in.
    bool Replay(DocumentIR& doc, const ReplayOptions& opt, ReplayResult& out);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace cadcvt
