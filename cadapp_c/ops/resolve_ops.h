#pragma once

namespace breptopo
{
class OpRegistry;
}

namespace cadapp
{

// Register TopoRef-resolution ops on a breptopo::OpRegistry.
//
// Registers:
//   "resolve_edge_ref" (shape, ref, tolerance) -> ShapeVal
//     Geo-matches the TopoRefIR (carried opaquely in a TopoRefVal)
//     against the input shape, returns the matching edge sub-shape.
//     ShapeVal::tag carries the TopoNaming uid (0 means no binding).
//
//   "resolve_face_ref" (shape, ref, tolerance) -> ShapeVal
//     Same, but for faces.
//
// These ops let Replayer build the whole graph upfront without
// having to materialise the upstream shape for geometric matching
// at construction time. The match runs at Eval time, hits the
// LruCache on rebuild, and frees Replayer to remain a pure graph
// builder.
//
// breptopo cannot register these itself: it would have to depend on
// cadapp::TopoRefIR. The wiring happens here where both sides are
// visible.
void RegisterResolveOps(breptopo::OpRegistry& reg);

} // namespace cadapp
