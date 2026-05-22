#pragma once

// ============================================================
// cadapp/resolve/TopoGeomUtils.h
//
// Tiny geometry primitives used by TopoRefResolver to score
// ref<->subshape matches, and by FreeCadReader to populate
// TopoRefIR.point / normal / measure from authored .brp files.
//
// Kept light: just thin wrappers over OCCT's BRepAdaptor +
// BRepGProp + BRep_Tool. No state.
// ============================================================

#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>

namespace cadapp
{

// Midpoint of the (non-degenerate) edge and the unit tangent at that
// midpoint. Returns false for degenerate or zero-length edges; in
// that case mid / tangent are left untouched.
bool EdgeMidpoint(const TopoDS_Edge& edge, gp_Pnt& mid, gp_Dir& tangent);

// UV-center of the face and the unit surface normal at that
// parametric center. Returns false when the surface degenerates
// there (cross-product of du, dv is zero).
bool FaceCenter(const TopoDS_Face& face, gp_Pnt& center, gp_Dir& normal);

// Arc length / surface area via BRepGProp. Both return 0 for
// degenerate inputs.
double EdgeArcLength(const TopoDS_Edge& edge);
double FaceArea(const TopoDS_Face& face);

} // namespace cadapp
