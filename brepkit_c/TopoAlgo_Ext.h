#pragma once

#include "TopoShape.h"

#include <SM_Vector.h>

#include <memory>
#include <vector>
#include <cstdint>

namespace brepgraph { class TopoNaming; }

namespace brepkit
{

// Extrude end-condition type, matches SolidWorks endCondition semantics.
// Integer values must stay in lock-step with cadapp::ExtrudeEndType
// (cadapp_c/ir/Enums.h): the Replayer forwards the cadapp value
// straight into a const node that calc_ops static_cast<>s back to
// this enum, and cadapp's values are pinned by FeatureStore's U8
// serialization. Append new entries at the bottom on both sides.
enum class ExtrudeEndType : uint8_t
{
    Blind             = 0, // Fixed distance
    ThroughAll        = 1, // Cut through all
    UpToSurface       = 2, // Up to a reference surface/face
    UpToVertex        = 3, // Up to a reference vertex
    MidPlane          = 4, // Symmetric, half on each side
    OffsetFromSurface = 5, // Offset from a reference surface
    // UpToFirst: stop at the FIRST face of `ref` encountered along
    // the prism direction. Implemented by building a ThroughAll-size
    // prism, intersecting with ref, picking the connected piece
    // nearest the sketch plane. FreeCAD PartDesign Pocket/Pad Type=2.
    UpToFirst         = 6,
};

class TopoAlgo_Ext
{
public:
    // ========================================================
    // Enhanced Extrude with end condition (SW Boss-Extrude)
    // ========================================================
    //
    // shape:    profile shape (face or wire)
    // dir_x/y/z: extrude direction (will be normalized internally)
    // dist1:    primary distance (used by Blind/MidPlane/Offset)
    // dist2:    secondary distance for two-direction extrude
    // end1/end2: end conditions for primary/secondary
    // ref:      reference shape for UpToSurface/UpToVertex
    static std::shared_ptr<TopoShape> ExtrudeEx(
        const std::shared_ptr<TopoShape>& shape,
        double dir_x,
        double dir_y,
        double dir_z,
        double dist1,
        double dist2,
        ExtrudeEndType end1,
        ExtrudeEndType end2,
        const std::shared_ptr<TopoShape>& ref,
        uint32_t op_id,
        const std::shared_ptr<brepgraph::TopoNaming>& tn);

    // ========================================================
    // Revolve (SW Boss-Revolve / Cut-Revolve)
    // ========================================================
    //
    // shape:        profile shape
    // axis_origin:  point on rotation axis
    // axis_dir:     rotation axis direction
    // angle:        rotation angle in radians (positive = right-hand rule)
    // is_full:      if true, ignore angle and rotate 2*PI
    static std::shared_ptr<TopoShape> Revolve(
        const std::shared_ptr<TopoShape>& shape,
        const sm::vec3& axis_origin,
        const sm::vec3& axis_dir,
        double angle,
        bool is_full,
        uint32_t op_id,
        const std::shared_ptr<brepgraph::TopoNaming>& tn);

    // ========================================================
    // Sweep (SW Sweep)
    // ========================================================
    //
    // profile: cross-section wire/face
    // path:    sweep path wire
    // is_solid: produce solid (true) or shell (false)
    static std::shared_ptr<TopoShape> Sweep(
        const std::shared_ptr<TopoShape>& profile,
        const std::shared_ptr<TopoShape>& path,
        bool is_solid,
        uint32_t op_id,
        const std::shared_ptr<brepgraph::TopoNaming>& tn);

    // ========================================================
    // HelixWire (FreeCAD Part::Helix spine)
    // ========================================================
    //
    // Builds a helical wire in the local frame (axis = +Z, the curve
    // starts at (radius, 0, 0)). Mirrors FreeCAD's TopoShape::
    // makeLongHelix: a 2D line laid on a cylindrical (or conical)
    // surface's (u,v) parameter space, lifted to a 3D edge by
    // BRepLib::BuildCurves3d. The caller positions the wire (the
    // FreeCAD Part::Helix Placement is applied upstream when needed;
    // a PartDesign FeatureBase imports the local geometry verbatim).
    //
    // pitch:       axial advance per full turn (> 0)
    // height:      total axial height (> 0)
    // radius:      helix radius at the base (> 0)
    // cone_angle:  cone half-angle in radians; ~0 (or degenerate,
    //              |angle| >= pi/2) yields a cylindrical helix,
    //              otherwise a conical one
    // left_handed: reverse the winding sense (LocalCoord left-handed)
    static std::shared_ptr<TopoShape> HelixWire(
        double pitch,
        double height,
        double radius,
        double cone_angle,
        bool   left_handed,
        uint32_t op_id,
        const std::shared_ptr<brepgraph::TopoNaming>& tn);

    // ========================================================
    // Linear Pattern (SW Linear Pattern)
    // ========================================================
    //
    // base:     shape to replicate
    // dir1:     primary direction
    // count1:   primary count (>= 1, includes original)
    // spacing1: distance between instances along dir1
    // dir2:     secondary direction (zero vec disables 2D)
    // count2:   secondary count
    // spacing2: secondary spacing
    static std::shared_ptr<TopoShape> LinearPattern(
        const std::shared_ptr<TopoShape>& base,
        const sm::vec3& dir1,
        int count1,
        double spacing1,
        const sm::vec3& dir2,
        int count2,
        double spacing2,
        uint32_t op_id,
        const std::shared_ptr<brepgraph::TopoNaming>& tn);

    // ========================================================
    // Circular Pattern (SW Circular Pattern)
    // ========================================================
    //
    // base:        shape to replicate
    // axis_origin: point on rotation axis
    // axis_dir:    rotation axis direction
    // count:       number of instances (includes original)
    // angle:       total angle covered (e.g. 2*PI for full circle)
    static std::shared_ptr<TopoShape> CircularPattern(
        const std::shared_ptr<TopoShape>& base,
        const sm::vec3& axis_origin,
        const sm::vec3& axis_dir,
        int count,
        double angle,
        uint32_t op_id,
        const std::shared_ptr<brepgraph::TopoNaming>& tn);

    // ========================================================
    // Hole Wizard (SW Hole Wizard / NX Hole Feature)
    // ========================================================
    //
    // Hole types:
    //   Simple      - straight cylindrical hole
    //   Counterbore - stepped hole (larger dia on top)
    //   Countersink - conical entry (e.g. 82 or 90 degrees)
    //   Tapped      - threaded hole (modeled as cylinder)
    enum class HoleType : uint8_t {
        Simple = 0,
        Counterbore,
        Countersink,
        Tapped
    };

    // body:          solid to cut the hole from
    // face:          placement face (hole starts from this face)
    // pos:           hole center on the face
    // dir:           hole axis direction (into the body)
    // diameter:      main hole diameter
    // depth:         hole depth (0 = through all)
    // hole_type:     see HoleType
    // cb_diameter:   counterbore diameter (HoleType::Counterbore)
    // cb_depth:      counterbore depth
    // cs_diameter:   countersink diameter (HoleType::Countersink)
    // cs_angle:      countersink included angle in radians
    static std::shared_ptr<TopoShape> HoleWizard(
        const std::shared_ptr<TopoShape>& body,
        const sm::vec3& pos,
        const sm::vec3& dir,
        double diameter,
        double depth,
        HoleType hole_type,
        double cb_diameter,
        double cb_depth,
        double cs_diameter,
        double cs_angle,
        uint32_t op_id,
        const std::shared_ptr<brepgraph::TopoNaming>& tn);

    // ========================================================
    // Variable Fillet (variable radius along edge)
    // ========================================================
    //
    // shape:   solid to fillet
    // edges:   edges to fillet
    // params:  pairs of (parameter_on_edge [0..1], radius)
    //          flattened: [u0, r0, u1, r1, ...]
    static std::shared_ptr<TopoShape> VariableFillet(
        const std::shared_ptr<TopoShape>& shape,
        const std::vector<std::shared_ptr<TopoShape>>& edges,
        const std::vector<double>& params,
        uint32_t op_id,
        const std::shared_ptr<brepgraph::TopoNaming>& tn);

    // ========================================================
    // Sweep With Guide (SW Sweep with guide curves)
    // ========================================================
    //
    // profile: cross-section wire/face
    // path:    sweep spine wire
    // guides:  one or more guide wires that control the profile
    // is_solid: produce solid (true) or shell (false)
    static std::shared_ptr<TopoShape> SweepWithGuide(
        const std::shared_ptr<TopoShape>& profile,
        const std::shared_ptr<TopoShape>& path,
        const std::vector<std::shared_ptr<TopoShape>>& guides,
        bool is_solid,
        uint32_t op_id,
        const std::shared_ptr<brepgraph::TopoNaming>& tn);

    // ========================================================
    // Rib (SW Rib / structural web)
    // ========================================================
    //
    // body:        existing solid to fuse rib into
    // profile:     open sketch profile (wire) for rib cross-section
    // dir:         extrude direction for the rib
    // thickness:   rib wall thickness
    // is_symmetric: if true, extrude half thickness on each side
    static std::shared_ptr<TopoShape> Rib(
        const std::shared_ptr<TopoShape>& body,
        const std::shared_ptr<TopoShape>& profile,
        const sm::vec3& dir,
        double thickness,
        bool is_symmetric,
        uint32_t op_id,
        const std::shared_ptr<brepgraph::TopoNaming>& tn);

}; // TopoAlgo_Ext

} // namespace brepkit
