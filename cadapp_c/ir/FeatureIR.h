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

    // ZW3D patterns carry a Boolean setting; "none" leaves every copy a
    // STANDALONE body instead of fusing it onto the running body (the
    // copies stay free until a later boolean absorbs them -- R2900_100's
    // Pattern9 holds 3 free bodies for exactly one feature, truth
    // n_shape 1 -> 4 -> 1). false = standalone copies; true = the
    // classic fuse-onto-base. NOT serialized by FeatureStore yet: the
    // FEAT_VERSION 3 payload encoding is fixed-format and the header
    // check rejects any other version, so adding a byte would orphan
    // every existing save (TODO fold into FEAT_VERSION 4). The flag is
    // re-derived from the snapshot on every cax.json import, which is
    // where it matters.
    bool    fuse       = true;
};

struct FeatPayloadCircularPattern
{
    double  axis_origin[3] = { 0.0, 0.0, 0.0 };
    double  axis_dir   [3] = { 0.0, 0.0, 1.0 };
    int32_t count          = 2;
    double  total_angle    = 0.0;     // radians; 0 means equal-spaced full
    // Per-copy overlap into the body, along -axis_dir, applied AFTER the
    // rotation. A thin boss whose base sits coplanar on the body surface does
    // not overlap it volumetrically, so the fuse leaves it a separate solid
    // (OCCT merges coplanar contact at metre scale but not mm). Pushing each
    // copy a hair into the body makes the fuse a clean overlap -> one solid.
    // 0 = no overlap (default; the linear / FreeCAD paths leave it 0).
    double  penetration    = 0.0;
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
    // Fuse / Cut / Common. The operand list lives in the parent
    // FeatureIR::input_feature_ids with InputRole::Operand on each
    // entry; the typed payload itself carries no extra data because
    // the variant kind already conveys the boolean op and the
    // FeatureIR id pinpoints the feature. Operand order is
    // significant and is preserved by the Reader's PushInput calls:
    // for Cut the first Operand entry is the base (kept) and
    // subsequent ones are tools subtracted; for Fuse / Common all
    // operands fold pairwise in order. P3.3.B migrated this off a
    // dedicated payload vector.
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

// Instance of an external part / sub-document, placed with a rigid
// transform. FreeCAD Assembly4 App::Link is the driving case:
//   - linked_file           the sibling .FCStd referenced via
//                           PropertyXLink (relative to host doc dir)
//   - linked_object_name    the root object inside that file (typically
//                           "Model", the App::Part container)
//   - sub_tip_feature_id    after the host reader recursively reads the
//                           sub-doc and re-ids its features into the
//                           parent DocumentIR, this points at the
//                           sub-doc's body tip feature. Replayer's
//                           Base-role input edge to the Link feature
//                           also carries this id; the field is kept
//                           explicit so a future "swap linked file"
//                           operation has the binding.
//   - placement_*           rigid transform applied to the sub-tip
//                           shape. Same axis-angle convention as
//                           StashPlacement: rotate(axis, angle) about
//                           origin, then translate(px,py,pz). Already
//                           in IR units (metres / radians).
struct FeatPayloadLink
{
    std::string linked_file;
    std::string linked_object_name;
    uint32_t    sub_tip_feature_id = 0;

    double placement_px = 0.0;
    double placement_py = 0.0;
    double placement_pz = 0.0;
    double placement_ox = 0.0;
    double placement_oy = 0.0;
    double placement_oz = 1.0;
    double placement_angle = 0.0;
};

// Baked geometry passthrough: a feature with no synthesizable
// parameters in our IR; geometry is whatever lives in
// DocumentIR::authored_shapes[feat.id] (typically a FreeCAD .brp
// dump). FreeCAD's Part::Feature is the driving case -- it surfaces
// in collapsed PartDesign Bodies, e.g. Piston.FCStd's Fillet001_solid,
// where the original feature history has been compacted to a single
// shape carrier. Identical mechanism could host other "we don't model
// the parameters yet but we DO have authored geometry" cases.
//
// The struct is intentionally empty: all useful information lives
// outside (authored_shapes for the shape, ext_strings for any
// reader-side diagnostic like "freecad_type"). The variant alternative
// exists so the Replayer can std::visit-dispatch a typed arm rather
// than overloading FeatPayloadOpaque's semantics.
struct FeatPayloadBakedShape
{
};

// Assembly4 constr_* object: an LCS-to-LCS coincidence with an
// optional rigid offset. Captured for round-trip / future solver
// integration; the Replayer ignores this payload because Assembly4
// has already baked the resulting placement onto each App::Link.
//
//   - linked_link_feature_id   the FeatPayloadLink this constraint
//                              positions (the host-side feature id
//                              after re-id); 0 if unresolved
//   - first_lcs_name           LCS in the parent assembly's frame
//                              (typically a child of the asm root or
//                              of an earlier Link)
//   - second_lcs_name          LCS inside the linked part's body
//   - parent_link_feature_id   when first_lcs lives on another Link
//                              (chained asm), the id of that parent
//                              Link feature; 0 when LCS is on the
//                              root assembly itself
//   - is_attached_to           raw FreeCAD string, kept verbatim for
//                              round-trip (e.g. "Parent Assembly")
//   - offset_*                 same axis-angle convention as
//                              FeatPayloadLink
//   - constraint_type          0 = coincident (default in Assembly4);
//                              reserved for future kinds
struct FeatPayloadAsmConstraint
{
    uint32_t    linked_link_feature_id = 0;
    uint32_t    parent_link_feature_id = 0;
    std::string first_lcs_name;
    std::string second_lcs_name;
    std::string is_attached_to;

    double  offset_px = 0.0;
    double  offset_py = 0.0;
    double  offset_pz = 0.0;
    double  offset_ox = 0.0;
    double  offset_oy = 0.0;
    double  offset_oz = 1.0;
    double  offset_angle = 0.0;

    uint8_t constraint_type = 0;
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
    FeatPayloadPrimEllipsoid,    // 24
    FeatPayloadLink,             // 25
    FeatPayloadAsmConstraint,    // 26
    FeatPayloadBakedShape        // 27
>;


// ---- MaterialIR ----
//
// Visual appearance of a feature's surface(s) as authored by the
// source CAD GUI layer. Held alongside the geometric FeatureIR so a
// downstream renderer (or a future material-aware emitter / writer)
// can apply the same look the source application showed.
//
// FreeCAD origin (the only reader populating this today):
//   GuiDocument.xml carries one PropertyMaterial per ViewProvider:
//     ambient/diffuse/specular/emissive : 32-bit packed RGBA, MSB->LSB
//                                          = R G B A. Alpha=0xFF means
//                                          opaque; FreeCAD often
//                                          writes 0x00 for ambient
//                                          because the alpha channel
//                                          is not actually used for
//                                          that slot.
//     shininess                          : 0..1 phong exponent normaliser
//     transparency                       : 0..1, 1 = fully transparent
//                                          (mirror of the integer
//                                          PropertyPercent "Transparency",
//                                          same value rescaled)
//
// App::Link inheritance:
//   By default a Link inherits the linked object's material at render
//   time -- the Link's own ShapeMaterial is read but the Link's
//   OverrideMaterial bool decides whether to apply it. has_override
//   mirrors that bool so a renderer can branch:
//     has_override == false -> walk to the linked object (in our IR,
//                              the inlined sub-features each carry
//                              their own MaterialIR) and apply theirs
//     has_override == true  -> use this MaterialIR for every output
//                              shape this feature contributes
//   For non-Link features has_override is ignored.
//
// "present" gates the whole struct: if false, the rest of the fields
// are default-constructed garbage and the renderer should fall back
// to whatever default it would use absent a material override.
struct MaterialIR
{
    bool     present       = false;

    uint32_t ambient_rgba  = 0;
    uint32_t diffuse_rgba  = 0;
    uint32_t specular_rgba = 0;
    uint32_t emissive_rgba = 0;
    double   shininess     = 0.0;
    double   transparency  = 0.0;

    bool     has_override  = false;
};


// ---- FeatureIR ----

struct FeatureIR
{
    // Stable id within a DocumentIR; matches VersionTree node_id
    // once the Replayer commits.
    uint32_t       id         = 0;
    std::string    name;
    FeatType       type       = FeatType::Unknown;
    bool           suppressed = false;

    // Explicit upstream features this one consumes, paired with
    // input_roles to disambiguate "what does this input mean".
    // Together these form the DAG edges the Replayer walks. P3.1
    // introduced input_feature_ids; P3.3 added the parallel
    // input_roles so multiple inputs of different semantics can
    // coexist (body chain pred + Boolean operands + Pattern tools
    // + sketch supports etc.).
    //
    // Invariant: input_roles.size() == input_feature_ids.size().
    // The Reader is responsible for keeping the two in lock-step;
    // see PushInput() in FreeCadReader for the helper that does so.
    //
    // Per-id conventions (all roles):
    //   id == 0           -- explicit "no upstream" sentinel
    //                        (Role::Base on body roots; not used
    //                        for other roles in practice).
    //                        Real feature ids start at 1, so 0 is
    //                        unambiguous.
    //   id == 0xFFFFFFFF  -- unresolved link (Reader couldn't
    //                        match the FreeCAD object name).
    //                        Replayer treats this as -1.
    //   id >= 1           -- ordinary upstream feature.
    //
    // Today only Role::Base is in use; Roles Operand / Tool /
    // PatternTarget / Reference migrate in P3.3.B-E. Until those
    // land, the corresponding ext_params / payload fields still
    // carry the data and the typed channel is empty for them.
    std::vector<uint32_t>  input_feature_ids;
    std::vector<InputRole> input_roles;

    // Typed payload; default-constructs to Opaque so a fresh
    // FeatureIR is always queryable.
    FeaturePayload data       = FeatPayloadOpaque{};

    // CAD-specific extras the variant doesn't model. Replayer
    // forwards them only to readers that recognise the keys.
    std::map<std::string, double>      ext_params;
    std::map<std::string, std::string> ext_strings;

    // Visual material as authored by the source GUI layer. Default-
    // constructed (present=false) means "no material override on
    // this feature, use the renderer default". See MaterialIR doc.
    MaterialIR                         material;
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
