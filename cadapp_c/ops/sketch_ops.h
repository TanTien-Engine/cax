#pragma once

namespace breptopo
{
class OpRegistry;
}

namespace cadapp
{

// Register sketch-related ops on a breptopo::OpRegistry.
//
// Currently registers:
//   "sketch_face" -- consumes a SketchVal (carrying a cadapp::SketchIR),
//                    runs sketchlib's constraint solver, lifts the
//                    solved 2D wire onto the plane carried by the
//                    Vec3 inputs, returns a planar TopoDS_Face.
//
// This op lives in cadapp (not breptopo) because it knows the
// concrete cadapp::SketchIR type underneath the type-erased
// SketchVal. breptopo stays kernel-/IR-agnostic.
void RegisterSketchOps(breptopo::OpRegistry& reg);

} // namespace cadapp
