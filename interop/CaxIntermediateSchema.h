#pragma once

// ============================================================
// interop/CaxIntermediateSchema.h
//
// The single, shared contract between the two halves of the ZW3D
// import path:
//
//   - WRITER : plugins/zw_export/ZwCaxExport.cpp -- runs INSIDE ZW3D,
//              walks the History Manager and emits a neutral JSON
//              snapshot ("<part>.cax.json").
//   - READER : cadcvt_c/reader/ZwReader.cpp -- runs in the cax process,
//              parses that JSON into cadapp::DocumentIR.
//
// The two never share a process, a build, or any SDK type -- the ONLY
// thing they share is this wire format. So this header carries the wire
// format's invariants as named constants instead of letting both sides
// repeat bare string literals that could silently drift apart:
//
//   - kSchemaVersion : the writer stamps it into the JSON, the reader
//     refuses a snapshot whose version it does not understand. This is
//     the safety net against a format change landing on one side only.
//   - the token strings : every dispatch token (feature kind, geometry
//     type, constraint type, end condition, ...). Both sides reference
//     the SAME constant, so a typo is a compile error here rather than a
//     feature that silently falls through to opaque at runtime.
//
// This header has ZERO dependencies (no JSON library, no cadapp IR), so
// both the SDK-bound plugin and the SDK-free reader can include it.
//
// The constants use plain `constexpr const char*` (internal linkage at
// namespace scope) so the header is valid all the way back to C++11 --
// the plugin side may build under an older toolchain than cax's C++20.
//
// When the wire format changes incompatibly, bump kSchemaVersion AND
// update both sides in the SAME commit (the whole point of keeping the
// writer and reader in one repository).
// ============================================================

namespace cax_schema
{

// Bump on any incompatible change to the wire format. The writer emits
// this as the top-level "schema_version"; the reader rejects anything
// it does not recognise.
constexpr int kSchemaVersion = 1;

// Top-level "source" tag (informational / diagnostics only).
constexpr const char* kSourceZw3d = "zw3d";

// ---- length_unit tokens (drive the reader's unit scale to metres) ----
namespace unit
{
constexpr const char* Mm = "mm";   // -> scale 0.001
constexpr const char* Cm = "cm";   // -> scale 0.01
constexpr const char* M  = "m";    // -> scale 1.0
constexpr const char* In = "in";   // -> scale 0.0254
} // namespace unit

// ---- feature "kind" tokens (top-level history dispatch) ----
namespace kind
{
constexpr const char* Sketch  = "sketch";
constexpr const char* Extrude = "extrude";
constexpr const char* Box     = "box";     // sketch-less box primitive -> FeatPayloadPrimBox
constexpr const char* Opaque  = "opaque";
} // namespace kind

// ---- extrude "subkind" tokens ----
namespace subkind
{
constexpr const char* Boss = "boss";
constexpr const char* Cut  = "cut";
} // namespace subkind

// ---- input role tokens (body-chain wiring) ----
namespace role
{
constexpr const char* Base          = "base";
constexpr const char* Operand       = "operand";
constexpr const char* Tool          = "tool";
constexpr const char* PatternTarget = "pattern_target";
constexpr const char* Reference     = "reference";
} // namespace role

// ---- sketch geometry type tokens ----
namespace geo
{
constexpr const char* Point   = "point";
constexpr const char* Line    = "line";
constexpr const char* Arc     = "arc";
constexpr const char* Circle  = "circle";
constexpr const char* Ellipse = "ellipse";
constexpr const char* Spline  = "spline";
constexpr const char* Unknown = "unknown";
} // namespace geo

// ---- sketch-point reference position tokens ----
namespace pos
{
constexpr const char* None   = "none";
constexpr const char* Start  = "start";
constexpr const char* Mid    = "mid";
constexpr const char* End    = "end";
constexpr const char* Center = "center";
} // namespace pos

// ---- constraint type tokens ----
namespace cons
{
constexpr const char* None           = "none";
constexpr const char* Distance       = "distance";
constexpr const char* DistanceX      = "distance_x";
constexpr const char* DistanceY      = "distance_y";
constexpr const char* Angle          = "angle";
constexpr const char* Parallel       = "parallel";
constexpr const char* Perpendicular  = "perpendicular";
constexpr const char* Coincident     = "coincident";
constexpr const char* Horizontal     = "horizontal";
constexpr const char* Vertical       = "vertical";
constexpr const char* Equal          = "equal";
constexpr const char* Tangent        = "tangent";
constexpr const char* Concentric     = "concentric";
constexpr const char* Symmetric      = "symmetric";
constexpr const char* Colinear       = "colinear";
constexpr const char* Fix            = "fix";
constexpr const char* CircleRadius   = "circle_radius";
constexpr const char* CircleDiameter = "circle_diameter";
constexpr const char* ArcRadius      = "arc_radius";
constexpr const char* ArcDiameter    = "arc_diameter";
} // namespace cons

// ---- extrude end-condition tokens ----
namespace end_cond
{
constexpr const char* Blind             = "blind";
constexpr const char* ThroughAll        = "through_all";
constexpr const char* UpToSurface       = "up_to_surface";
constexpr const char* UpToVertex        = "up_to_vertex";
constexpr const char* MidPlane          = "mid_plane";
constexpr const char* OffsetFromSurface = "offset_from_surface";
constexpr const char* UpToFirst         = "up_to_first";
} // namespace end_cond

} // namespace cax_schema
