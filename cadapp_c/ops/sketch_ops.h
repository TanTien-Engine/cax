#pragma once

#include <memory>

namespace brepgraph
{
class OpRegistry;
}

namespace brepkit
{
class TopoShape;
}

namespace cadapp
{

struct SketchIR;

// Build a planar face from a SketchIR on the given plane (origin / x_dir /
// normal, each a 3-double array). Runs sketchlib's solver, stitches the
// solved 2D geometry into wires (ConnectEdgesToWires) and fills them into a
// face with hole nesting -- the exact path the "sketch_face" op uses, exposed
// so a host can rebuild a face from edited sketch geometry. nullptr on
// failure. Construction geometry in the SketchIR is ignored by the importer.
std::shared_ptr<brepkit::TopoShape> BuildSketchFace(
    const SketchIR& sk,
    const double*   origin,
    const double*   x_dir,
    const double*   normal);

// Register sketch-related ops on a brepgraph::OpRegistry.
//
// Currently registers:
//   "sketch_face" -- consumes a SketchVal (carrying a cadapp::SketchIR),
//                    runs sketchlib's constraint solver, lifts the
//                    solved 2D wire onto the plane carried by the
//                    Vec3 inputs, returns a planar TopoDS_Face.
//
// This op lives in cadapp (not brepgraph) because it knows the
// concrete cadapp::SketchIR type underneath the type-erased
// SketchVal. brepgraph stays kernel-/IR-agnostic.
void RegisterSketchOps(brepgraph::OpRegistry& reg);

} // namespace cadapp
