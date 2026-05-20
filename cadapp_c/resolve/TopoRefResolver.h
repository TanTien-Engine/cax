#pragma once

#include "cadapp_c/ir/TopoRefIR.h"

#include <cstdint>
#include <vector>

// ============================================================
// cadapp/resolve/TopoRefResolver.h
//
// Locate every TopoRefIR on a live OCCT shape via geometric
// matching, then ask TopoNaming for the uid (the second layer of
// the strategy).
//
//   ResolvedRef::topo_index = 1-based TopExp::MapShapes index
//   ResolvedRef::uid        = TopoNaming uid (0 means no binding)
//   ResolvedRef::match_dist = geometric match distance (0 perfect)
//
// The caller (Replayer) writes ResolvedRef::uid back onto the
// FeatureStore's TopoRefIR. From then on edits can take the uid
// fast-path and skip geometric matching.
//
// The header only depends on cadapp/ir; OCCT and brepgraph are
// inside the cpp.
// ============================================================

class TopoDS_Shape;
namespace brepgraph
{
class TopoNaming;
}

namespace cadapp
{

struct ResolvedRef
{
    TopoRefIR::Kind kind       = TopoRefIR::Kind::Edge;
    int32_t         topo_index = 0;   // 1-based, 0 means unmatched
    uint32_t        uid        = 0;   // 0 means not bound to TopoNaming
    double          match_dist = 0.0;
};

class TopoRefResolver
{
public:
    // Primary entry. shape is the model state right before
    // Replay(feat_i) consumes refs.
    //
    // tolerance : maximum match distance to accept; over this the
    //             topo_index is set to 0.
    // naming    : optional. When non-null the resolved sub-shape
    //             is looked up in TopoNaming via HistGraph::GetUID;
    //             when null the uid stays at 0.
    //
    // The result is one ResolvedRef per input, in the same order.
    static std::vector<ResolvedRef> Resolve(const TopoDS_Shape&            shape,
                                            const std::vector<TopoRefIR>&  refs,
                                            brepgraph::TopoNaming*          naming,
                                            double                         tolerance = 1e-3);
};

} // namespace cadapp
