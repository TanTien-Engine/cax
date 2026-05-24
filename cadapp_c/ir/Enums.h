#pragma once

#include <cstdint>

// ============================================================
// cadapp/ir/Enums.h
//
// Public enums for sketch geometry, sketch constraints, feature
// types and extrude end conditions. CAD-kernel independent and
// independent of sketchlib / OCCT.
//
// SkConsType is a 1:1 mirror of sketchlib::ConsType, kept here so
// readers can include this header without dragging in the solver.
// SketchBridge.cpp owns the conversion table; the two enums must
// stay in lock-step.
// ============================================================

namespace cadapp
{

// Sketch geometry kind. 1:1 with sketchlib::GeoType for the items
// the solver understands, plus Spline which is sketchlib-unsupported
// and only flows through emit / replay paths.
enum class SkGeoType : uint8_t
{
    None    = 0,
    Point   = 1,
    Line    = 2,
    Arc     = 3,
    Circle  = 4,
    Ellipse = 5,
    Spline  = 6,
};

// Point position on a geometry. Mirrors sketchgraph/variant.ves
// GEO_PT_ID_*. Center is flattened in here for Arc/Circle, even
// though sketchlib expresses it via a separate geo.
enum class SkPointPos : uint8_t
{
    None   = 0,
    Start  = 1,
    Mid    = 2,
    End    = 3,
    Center = 4,
};

// Sketch constraint kind. 1:1 with sketchlib::ConsType. Order must
// stay stable across releases; only append at the bottom.
enum class SkConsType : uint8_t
{
    None                = 0,

    // basic
    Distance            = 1,
    DistanceX           = 2,
    DistanceY           = 3,
    Angle               = 4,
    Parallel            = 5,
    Perpendicular       = 6,
    Coincident          = 7,
    Horizontal          = 8,
    Vertical            = 9,
    Equal               = 10,

    // point on
    PointOnLine         = 11,
    PointOnCircle       = 12,
    PointOnArc          = 13,
    PointOnEllipse      = 14,
    PointOnPerpBisector = 15,
    MidpointOnLine      = 16,

    // tangent
    Tangent             = 17,
    TangentCircumf      = 18,

    // params
    CircleRadius        = 19,
    CircleDiameter      = 20,
    ArcRadius           = 21,
    ArcDiameter         = 22,

    // IR-only kinds; SketchBridge skips these when sketchlib has
    // no matching constraint yet.
    Symmetric           = 23,
    Concentric          = 24,
    Colinear            = 25,
    Fix                 = 26,
};

// Feature kind. Covers SW / FreeCAD / NX / Creo common features.
// Self-CAD subset can use only the first ten.
enum class FeatType : uint8_t
{
    Unknown          = 0,

    // sketch
    Sketch           = 1,

    // sketch-based
    BossExtrude      = 10,  // pad (FreeCAD) / extrude+add (SW)
    CutExtrude       = 11,  // pocket / extrude+cut
    BossRevolve      = 12,
    CutRevolve       = 13,
    Loft             = 14,
    Sweep            = 15,
    Rib              = 16,

    // dress-up
    Fillet           = 30,
    Chamfer          = 31,
    Shell            = 32,
    Draft            = 33,
    Offset           = 34,

    // transform
    Translate        = 50,
    Rotate           = 51,
    Mirror           = 52,
    Scale            = 53,

    // pattern
    LinearPattern    = 60,
    CircularPattern  = 61,
    MultiTransform   = 62,  // ordered chain of Mirror / LinearPattern / CircularPattern

    // boolean
    Fuse             = 70,
    Cut              = 71,
    Common           = 72,

    // primitive (no sketch)
    PrimBox          = 80,
    PrimCylinder     = 81,
    PrimCone         = 82,
    PrimSphere       = 83,
    PrimTorus        = 84,
    PrimEllipsoid    = 85,

    // hole wizard
    HoleWizard       = 90,
};

// Extrude / revolve end condition.
enum class ExtrudeEndType : uint8_t
{
    Blind             = 0,  // fixed distance (default)
    ThroughAll        = 1,
    UpToSurface       = 2,
    UpToVertex        = 3,
    MidPlane          = 4,
    OffsetFromSurface = 5,
};

} // namespace cadapp
