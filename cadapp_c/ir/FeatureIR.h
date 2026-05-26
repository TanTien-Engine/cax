#pragma once

#include "cadapp_c/ir/Enums.h"
#include "cadapp_c/ir/SketchIR.h"
#include "cadapp_c/ir/TopoRefIR.h"

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

// Forward-declare so DocumentIR can hold authored-shape handles
// without pulling brepkit / OCCT headers into the IR layer.
namespace brepkit { class TopoShape; }

// ============================================================
// cadapp/ir/FeatureIR.h
//
// A FeatureIR is split into:
//   - common header  : id / name / suppressed / extension bag
//   - typed payload  : one variant alternative per FeatType, holding
//                      only the fields that kind actually needs
//
// Why a variant (and not a flat struct):
//   * Each kind exposes only its own fields - no more "PrimBox
//     stuffs length/width/height into direction[3]" hacks.
//   * Replayer / FeatureStore / VesEmitter dispatch through
//     std::visit, so adding a new feature kind fails to compile in
//     every consumer until handled. Bug-by-omission goes away.
//   * Memory: each FeatureIR carries sizeof(largest payload), not
//     sizeof(all kinds combined). Sketch features stop wasting 500
//     bytes of zeroed Fillet/Pattern slots.
//
// Adding a new feature kind:
//   1. Define a FeatPayload<Name> struct here.
//   2. Add it to the FeaturePayload variant alias below, AT THE
//      BOTTOM (never insert in the middle - the variant index is
//      what the FeatureStore serializes).
//   3. Add the FeatType enum value in Enums.h, keeping numbering
//      stable.
//   4. Handle the new alternative in FeatureStore (encode + decode)
//      and Replayer (visit case).
// ============================================================

namespace cadapp
{

// ---- typed payloads ----

struct FeatPayloadSketch
{
    // The Sketch feature itself owns the sketch_id; the actual
    // geometry / constraints live in SketchStore keyed by this id.
    uint32_t sketch_id = 0xFFFFFFFF;
};

struct FeatPayloadExtrude
{
    // Used for BossExtrude / CutExtrude. Cut vs Boss is selected
    // by FeatureIR::type, not by this struct.
    uint32_t       sketch_id      = 0xFFFFFFFF;
    double         direction[3]   = { 0.0, 0.0, 1.0 };
    double         distance       = 0.0;   // main side distance
    double         distance2      = 0.0;   // second side distance (0 = one-sided)
    ExtrudeEndType end_type       = ExtrudeEndType::Blind;
    ExtrudeEndType end_type2      = ExtrudeEndType::Blind;
    bool           flip_direction = false;
    bool           is_thin        = false;
    double         thin_thickness = 0.0;

    // UpToSurface / UpToVertex targets. Single ref each side.
    TopoRefIR end1_target;
    TopoRefIR end2_target;
    bool      has_end1_target = false;
    bool      has_end2_target = false;
};

struct FeatPayloadRevolve
{
    // Used for BossRevolve / CutRevolve.
    uint32_t sketch_id        = 0xFFFFFFFF;
    double   axis_origin[3]   = { 0.0, 0.0, 0.0 };
    double   axis_dir   [3]   = { 0.0, 0.0, 1.0 };
    double   angle            = 0.0;       // radians
    double   angle2           = 0.0;       // second side angle, 0 = one-sided
    bool     flip_direction   = false;
    bool     is_thin          = false;
    double   thin_thickness   = 0.0;
};

struct FeatPayloadLoft
{
    // Profiles are sketches; guide_refs are optional 3D guide curves.
    std::vector<uint32_t>  profile_sketch_ids;
    std::vector<TopoRefIR> guide_refs;
    bool                   closed = false;
};

struct FeatPayloadSweep
{
    uint32_t  profile_sketch_id = 0xFFFFFFFF;
    TopoRefIR path_ref;                       // single 3D path edge / wire
    bool      twist_along_path = false;
};

struct FeatPayloadFillet
{
    double                 radius = 0.0;
    std::vector<TopoRefIR> edges;

    // World-coord points (in IR units, metres for FreeCAD) used to
    // pre-split the running body's edges before fillet runs. The
    // reader fills these from base brep vertices on each picked
    // face (face-pick handler) -- those vertices represent edge
    // junctions FreeCAD kept that cax's BOP collapsed into a
    // single merged closed/long edge. Without splitting, ChFi3d
    // can fail on the merged edge because of curvature spikes at
    // the former join points. See Page_015 Fillet002's 163 mm
    // closed BSpline on Pad002.
    //
    // Empty for edge-typed picks (the picked edge is already
    // unambiguous) and for FreeCAD files with no BOP merge.
    std::vector<std::array<double, 3>> split_hints;
};

struct FeatPayloadChamfer
{
    double                 distance1 = 0.0;  // distance from edge to chamfer
    double                 distance2 = 0.0;  // 0 = symmetric (distance1 = distance2)
    std::vector<TopoRefIR> edges;

    // Same role as FeatPayloadFillet::split_hints; see comment there.
    std::vector<std::array<double, 3>> split_hints;
};

struct FeatPayloadShell
{
    double                 thickness     = 0.0;
    std::vector<TopoRefIR> faces_to_open; // faces to remove
    bool                   shell_outward = false;
};

struct FeatPayloadDraft
{
    double                 angle     = 0.0;   // radians
    double                 pull_dir[3] = { 0.0, 0.0, 1.0 };
    std::vector<TopoRefIR> faces;            // faces to draft
    TopoRefIR              neutral_plane;
    bool                   has_neutral_plane = false;
};

struct FeatPayloadOffset
{
    double                 distance = 0.0;
    std::vector<TopoRefIR> faces;
};

struct FeatPayloadTransform
{
    // Translate / Rotate / Scale share the same payload because the
    // tail differs only by which fields are read.
    //   Translate : translation[3]
    //   Rotate    : axis_origin[3], axis_dir[3], angle
    //   Scale     : scale[3] (uniform when all three equal)
    double translation[3] = { 0.0, 0.0, 0.0 };
    double axis_origin[3] = { 0.0, 0.0, 0.0 };
    double axis_dir   [3] = { 0.0, 0.0, 1.0 };
    double angle          = 0.0;
    double scale[3]       = { 1.0, 1.0, 1.0 };
};

struct FeatPayloadMirror
{
    double plane_origin[3] = { 0.0, 0.0, 0.0 };
    double plane_normal[3] = { 1.0, 0.0, 0.0 };
};

struct FeatPayloadLinearPattern
{
    double  dir1[3]    = { 1.0, 0.0, 0.0 };
    int32_t count1     = 2;
    double  spacing1   = 0.0;
    double  dir2[3]    = { 0.0, 1.0, 0.0 };
    int32_t count2     = 1;       // 1 = single direction
    double  spacing2   = 0.0;
};

struct FeatPayloadCircularPattern
{
    double  axis_origin[3] = { 0.0, 0.0, 0.0 };
    double  axis_dir   [3] = { 0.0, 0.0, 1.0 };
    int32_t count          = 2;
    double  total_angle    = 0.0;     // radians; 0 means equal-spaced full
};

// One step inside a MultiTransform. Mirrors / linear patterns /
// circular patterns share the slot; the kind discriminator picks
// which subset of fields is meaningful (other fields hold defaults).
struct MultiTransformStep
{
    enum class Kind : uint8_t
    {
        Mirror          = 0,
        LinearPattern   = 1,
        CircularPattern = 2,
    };
    Kind kind = Kind::Mirror;

    // Mirror
    double plane_origin[3] = { 0.0, 0.0, 0.0 };
    double plane_normal[3] = { 1.0, 0.0, 0.0 };

    // LinearPattern
    double  dir1[3]  = { 1.0, 0.0, 0.0 };
    int32_t count1   = 2;
    double  spacing1 = 0.0;
    double  dir2[3]  = { 0.0, 1.0, 0.0 };
    int32_t count2   = 1;
    double  spacing2 = 0.0;

    // CircularPattern
    double  axis_origin[3] = { 0.0, 0.0, 0.0 };
    double  axis_dir   [3] = { 0.0, 0.0, 1.0 };
    int32_t count          = 2;
    double  total_angle    = 0.0;
};

// FreeCAD's PartDesign::MultiTransform: an ordered list of single
// transformations applied to the previous shape. Each step's effect
// includes the original (mirror produces orig + reflection; pattern
// ops produce orig + copies), so the Replayer just chains the ops.
struct FeatPayloadMultiTransform
{
    std::vector<MultiTransformStep> steps;
};

struct FeatPayloadBoolean
{
    // Fuse / Cut / Common. Operands are referenced by feature_id
    // (other features in this DocumentIR producing solids). Order is
    // significant: for Cut the first operand is the base (kept) and
    // subsequent operands are tools subtracted from it; for Fuse /
    // Common all operands are folded pairwise in order. FreeCAD's
    // Part::Cut maps to exactly two operands; Part::MultiFuse and
    // Part::MultiCommon can carry any number.
    std::vector<uint32_t> operand_feature_ids;
};

struct FeatPayloadPrimBox
{
    double length = 1.0;
    double width  = 1.0;
    double height = 1.0;
};

struct FeatPayloadPrimCylinder
{
    double radius = 0.5;
    double height = 1.0;
};

struct FeatPayloadPrimCone
{
    double radius1 = 0.5;
    double radius2 = 0.0;
    double height  = 1.0;
};

struct FeatPayloadPrimSphere
{
    double radius = 0.5;
};

struct FeatPayloadPrimTorus
{
    double major_radius = 1.0;
    double minor_radius = 0.25;
};

// FreeCAD's Part::Ellipsoid stores three semi-axis radii. Following
// FreeCAD's own execute(): the X/Y semi-axes equal radius2, and the
// Z semi-axis equals radius3 when radius3 >= Confusion, otherwise
// radius1. radius1/radius2/radius3 are kept as-is here so the
// downstream maker can apply the same rule.
struct FeatPayloadPrimEllipsoid
{
    double radius1 = 2.0;
    double radius2 = 4.0;
    double radius3 = 0.0;
};

struct FeatPayloadHoleWizard
{
    // SW-specific extras are kept inside ext_strings in the parent
    // FeatureIR (e.g. thread spec like "M6x1.0").
    uint32_t  sketch_id   = 0xFFFFFFFF;
    double    diameter    = 0.0;
    double    depth       = 0.0;
    bool      through_all = false;
    TopoRefIR placement_face;
    bool      has_placement_face = false;
};

struct FeatPayloadRib
{
    uint32_t sketch_id    = 0xFFFFFFFF;
    double   thickness    = 0.0;
    double   direction[3] = { 0.0, 0.0, 1.0 };
    bool     flip         = false;
};

// Catch-all for kinds we recognise but haven't typed yet, or for
// reader-side experiments. Payload is the named-key bag.
struct FeatPayloadOpaque
{
    std::map<std::string, double>      params;
    std::map<std::string, std::string> strings;
    std::vector<TopoRefIR>             edge_refs;
    std::vector<TopoRefIR>             face_refs;
};


// ---- variant alias ----
//
// IMPORTANT: never reorder; the variant index is what FeatureStore
// serializes on disk. Append new payload types only at the end,
// and bump FEAT_VERSION when you do.
using FeaturePayload = std::variant<
    FeatPayloadSketch,           //  0
    FeatPayloadExtrude,          //  1
    FeatPayloadRevolve,          //  2
    FeatPayloadLoft,             //  3
    FeatPayloadSweep,            //  4
    FeatPayloadFillet,           //  5
    FeatPayloadChamfer,          //  6
    FeatPayloadShell,            //  7
    FeatPayloadDraft,            //  8
    FeatPayloadOffset,           //  9
    FeatPayloadTransform,        // 10
    FeatPayloadMirror,           // 11
    FeatPayloadLinearPattern,    // 12
    FeatPayloadCircularPattern,  // 13
    FeatPayloadBoolean,          // 14
    FeatPayloadPrimBox,          // 15
    FeatPayloadPrimCylinder,     // 16
    FeatPayloadPrimCone,         // 17
    FeatPayloadPrimSphere,       // 18
    FeatPayloadPrimTorus,        // 19
    FeatPayloadHoleWizard,       // 20
    FeatPayloadRib,              // 21
    FeatPayloadOpaque,           // 22
    FeatPayloadMultiTransform,   // 23
    FeatPayloadPrimEllipsoid     // 24
>;


// ---- FeatureIR ----

struct FeatureIR
{
    // Stable id within a DocumentIR; matches VersionTree node_id
    // once the Replayer commits.
    uint32_t       id         = 0;
    std::string    name;
    FeatType       type       = FeatType::Unknown;
    bool           suppressed = false;

    // Typed payload; default-constructs to Opaque so a fresh
    // FeatureIR is always queryable.
    FeaturePayload data       = FeatPayloadOpaque{};

    // CAD-specific extras the variant doesn't model. Replayer
    // forwards them only to readers that recognise the keys.
    std::map<std::string, double>      ext_params;
    std::map<std::string, std::string> ext_strings;
};

// Convenience: build a FeatureIR around a payload, deducing type
// from the payload struct when possible.
template <typename P>
inline FeatureIR MakeFeature(uint32_t id, FeatType type, std::string name, P&& payload)
{
    FeatureIR f;
    f.id   = id;
    f.type = type;
    f.name = std::move(name);
    f.data = std::forward<P>(payload);
    return f;
}


// ---- DocumentIR ----

struct DocumentIR
{
    std::string            source;     // "self" / "freecad" / "sw" / ...
    std::string            doc_path;   // origin file path, for diagnostics
    std::vector<SketchIR>  sketches;
    std::vector<FeatureIR> features;

    // feature_id -> the source-side authored body for that feature.
    // Populated by readers that have access to a ground-truth shape
    // (FreeCAD's .brp dumps inside the .FCStd archive); empty for
    // readers that don't. Already scaled into IR units. Used by the
    // Replayer as a last-resort substitute when cax's own replay of
    // a feature returns a null shape (typically an OCCT bug on a
    // specific geometry that the SEH harness in TopoAlgo demoted to
    // a clean failure -- see e.g. Page_037's MakeThickSolidByJoin
    // BRepTools_History AV). Not consulted for soft divergences; the
    // intent is "let the doc finish loading when OCCT is wedged",
    // not "silently mask normal-path bugs".
    std::map<uint32_t, std::shared_ptr<brepkit::TopoShape>> authored_shapes;
};

} // namespace cadapp
