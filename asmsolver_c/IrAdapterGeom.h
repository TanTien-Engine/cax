#pragma once

// OCCT-dependent companion to IrAdapter: resolve geometry that the joint
// metadata alone can't carry. Currently fills the radius of plane-to-
// cylinder Distance joints from the referenced cylindrical face in the
// document's authored OCCT shapes.
//
// Separated from IrAdapter.{h,cpp} (which is OCCT-free) because this pulls
// OCCT + brepkit::TopoShape. Build it only where OCCT is available.

#include "asmsolver_c/IrAdapter.h"

namespace cadapp { struct DocumentIR; }

namespace asmsolver {

// For every Distance joint in `result`, inspect the FreeCAD element refs
// (joint_ref{1,2}_elem) of the matching FeatType::Joint feature in `doc`;
// if one names a cylindrical face, set that joint's `radius` from the
// authored OCCT geometry (so the solver uses the plane-to-cylinder
// residual). Returns the number of radii resolved.
int ResolveCylinderRadii(ImportResult& result, const cadapp::DocumentIR& doc);

} // namespace asmsolver
