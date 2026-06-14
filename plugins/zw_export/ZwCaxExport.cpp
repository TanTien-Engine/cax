// ============================================================
// plugins/zw_export/ZwCaxExport.cpp
//
// ZW3D plugin-side exporter. Runs INSIDE ZW3D (loaded as a DLL via
// Application -> Applications and Plugin Manager, or auto-loaded from
// the apilibs folder). Walks the active part's History Manager and
// writes the cax neutral intermediate (.cax.json) that cadcvt::ZwReader
// consumes out-of-process.
//
// Why this exists at all: unlike SolidWorks (out-of-process COM via
// ISldWorks, driven from the cax process), ZW3D exposes only an
// in-process C/C++ plugin API. So the feature-tree walk must live here,
// inside ZW3D, and hand cax a serialized snapshot.
//
// Layering:
//   namespace zwapi  -- the ONLY place ZW3D SDK calls appear. Every
//                       function is a thin wrapper returning neutral
//                       C++ data. All marked "TODO: bind" -- replace
//                       the placeholder calls with the real ZW3D API
//                       names/signatures from the zw3d API headers.
//   export logic     -- walks zwapi's neutral data, builds JSON. Never
//                       touches an SDK type.
//
// Wire format: the JSON tokens are NOT spelled inline here -- they come
// from interop/CaxIntermediateSchema.h, the single contract shared with
// cadcvt::ZwReader. That header also carries kSchemaVersion, stamped
// into every snapshot so the reader can reject a format it predates.
//
// Dependencies: nlohmann/json (header-only) for emit, and the shared
// schema header (zero-dependency). Both are wired by this plugin's own
// CMakeLists.txt (plugins/zw_export/CMakeLists.txt), kept entirely out
// of the cax build.
//
// Neutral intermediate schema (see CaxIntermediateSchema.h for tokens):
//   {
//     "schema_version": 1,
//     "source": "zw3d",
//     "length_unit": "mm",
//     "document": {
//       "name": "<part name>",
//       "features": [ <feature>, ... ]   // history order
//     }
//   }
//
//   feature (sketch):
//     { "id":1, "kind":"sketch", "name":"...",
//       "plane": { "origin":[x,y,z], "x_dir":[x,y,z], "normal":[x,y,z] },
//       "geoms": [ <geom>, ... ],
//       "constraints": [ <cons>, ... ] }
//
//   feature (extrude):
//     { "id":2, "kind":"extrude", "subkind":"boss"|"cut", "name":"...",
//       "profile_id":1,
//       "inputs":[ {"id":0,"role":"base"} ],
//       "end_cond":"blind", "depth":20.0,
//       "end_cond2":"blind", "depth2":0.0,
//       "flip":false, "thin":false, "thin_thickness":0.0 }
//
//   feature (opaque, unrecognized):
//     { "id":3, "kind":"opaque", "name":"...", "zw_type":"Fillet" }
//
//   geom (coords sketch-local, in length_unit; angles in radians):
//     point   : { "geo_id":1, "type":"point",  "construction":false, "pt":[x,y] }
//     line    : { "geo_id":2, "type":"line",   "construction":false, "p0":[x,y], "p1":[x,y] }
//     arc     : { "geo_id":3, "type":"arc",     "center":[cx,cy], "radius":r,
//                 "start_ang":a, "end_ang":b }
//     circle  : { "geo_id":4, "type":"circle",  "center":[cx,cy], "radius":r }
//     ellipse : { "geo_id":5, "type":"ellipse", "center":[cx,cy],
//                 "major_r":mr, "minor_r":nr }
//     spline  : { "geo_id":6, "type":"spline",  "ctrl":[[x,y], ...] }
//
//   cons:
//     { "type":"horizontal", "value":0.0, "driving":true,
//       "a":{"geo":1,"pos":"none"}, "b":{"geo":-1,"pos":"none"} }
//     geo == -1 means "no reference"; pos in
//     none|start|mid|end|center.
// ============================================================

#include "interop/CaxIntermediateSchema.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>
#include <array>

// ZW3D's C API hands back file paths as UTF-8. On Windows the CRT's narrow
// std::fopen opens paths in the ANSI code page, so a non-ASCII path (e.g. a
// Chinese part name) never opens. We need MultiByteToWideChar + _wfopen to
// write the snapshot the way ZW3D's own cvxFileExport writes the STEP.
#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#endif

namespace
{

using json = nlohmann::json;
namespace sc = cax_schema;

// ============================================================
// Neutral data the export logic works with. The zwapi layer below
// is responsible for filling these from the ZW3D SDK; nothing above
// this point knows an SDK type exists.
// ============================================================

enum class FeatKind
{
    Sketch,
    Extrude,
    Box,
    Opaque,
};

enum class GeoKind
{
    Point,
    Line,
    Arc,
    Circle,
    Ellipse,
    Spline,
    Unknown,
};

enum class ConsKind
{
    None,
    Distance,
    DistanceX,
    DistanceY,
    Angle,
    Parallel,
    Perpendicular,
    Coincident,
    Horizontal,
    Vertical,
    Equal,
    Tangent,
    Concentric,
    Symmetric,
    Colinear,
    Fix,
    CircleRadius,
    CircleDiameter,
    ArcRadius,
    ArcDiameter,
};

enum class EndCond
{
    Blind,
    ThroughAll,
    UpToSurface,
    UpToVertex,
    MidPlane,
    OffsetFromSurface,
    UpToFirst,
};

enum class PointPos
{
    None,
    Start,
    Mid,
    End,
    Center,
};

struct GeomData
{
    uint32_t            geo_id       = 0;
    GeoKind             kind         = GeoKind::Unknown;
    bool                construction = false;
    std::vector<double> params;       // sketch-local, in length_unit
};

struct ConsRef
{
    int32_t  geo = -1;                // -1 = no reference
    PointPos pos = PointPos::None;
};

struct ConsData
{
    ConsKind kind    = ConsKind::None;
    ConsRef  a;
    ConsRef  b;
    double   value   = 0.0;           // length / angle by kind
    bool     driving = true;
};

struct SketchData
{
    double                origin[3] = { 0.0, 0.0, 0.0 };
    double                x_dir [3] = { 1.0, 0.0, 0.0 };
    double                normal[3] = { 0.0, 0.0, 1.0 };
    std::vector<GeomData> geoms;
    std::vector<ConsData> cons;
};

struct ExtrudeData
{
    bool     boss           = true;   // true = boss/add, false = cut
    uint32_t profile_id     = 0;
    EndCond  end_cond       = EndCond::Blind;
    EndCond  end_cond2      = EndCond::Blind;
    double   depth          = 0.0;
    double   depth2         = 0.0;
    bool     flip           = false;
    bool     thin           = false;
    double   thin_thickness = 0.0;
};

// Sketch-less box primitive (ZW3D "Block" / FtAllBox). Sizes plus the
// world min-corner (where OCCT's origin-anchored box must be placed).
struct BoxData
{
    double length = 1.0;        // X
    double width  = 1.0;        // Y
    double height = 1.0;        // Z
    double place[3] = { 0.0, 0.0, 0.0 };   // world min-corner (mm)
    bool   has_place = false;
};

// Geometric signature of a referenced entity (face / edge / ...), used
// later for reference resolution by GEOMETRIC MATCHING in OCCT: a
// representative point (on-face point for faces, bbox centre otherwise)
// plus a normal for faces. ZW3D's topological ids don't survive into the
// OCCT rebuild, so the geometry is what we match on.
struct EntSig
{
    std::string kind;                       // "face" / "edge" / "ent"
    double      anchor[3] = { 0.0, 0.0, 0.0 };
    double      normal[3] = { 0.0, 0.0, 0.0 };
    bool        has_normal = false;
    bool        has_num    = false;         // scalar tied to this pick, e.g. a
    double      num        = 0.0;           // chamfer / fillet per-edge setback
    // The JSON ordinal id (FeatNode.id == loop index + 1) of the feature that
    // OWNS this entity, from cvxPartInqEntFtr. Lets a pattern's Base entities
    // carry feature IDENTITY the reader can use directly -- e.g. a pattern of a
    // pattern (Pattern4 over Pattern3's ring) resolves to the inner pattern
    // feature instead of being guessed from a sub-mm cluster of anchors. 0 when
    // the owner isn't a history feature in the exported list.
    bool        has_feat   = false;
    int         feat       = 0;
    // Blanked (hidden) in ZW3D. Construction skins in surface-modeling
    // parts are routinely blanked once consumed; the STEP translator
    // excludes them by default (VX_EXCLUDE_BLANKED), so downstream truth
    // comparisons need to know which result shapes are invisible.
    bool        blanked    = false;
};

// One field of a feature's data container, read generically. fld_id is
// the STABLE key (fld_name may be empty / localized). The value shape
// depends on the field type: scalar, point, text, or a list of entity
// references (each carrying a geometric signature).
struct FieldDump
{
    int         id   = 0;
    std::string name;
    std::string type;                       // number/distance/angle/point/entity/text/list/other
    bool        has_num  = false; double num = 0.0;
    bool        has_pt   = false; double pt[3] = { 0.0, 0.0, 0.0 };
    bool        has_dir  = false; double dir[3] = { 0.0, 0.0, 0.0 };
    bool        has_text = false; std::string text;
    std::vector<EntSig> ents;
    // Multi-point field: a PNT_TO_PNT pattern's "To points" (fld 14) carries
    // the FULL list of instance locations, but cvxDataGetPnt returns only the
    // first -- so an irregular hole row collapses to a single point. Every
    // svxData.Pnt the field's fld_data array carries is captured here.
    std::vector<std::array<double, 3>> pts;
    int         list_count = -1;            // VX_FLD_DATA: # top-level tree
                                            // items found (diagnostic; -1 = n/a)
};

// Typed data of an external-geometry-copy feature (ZW3D "CdGeomCopy").
// Its data is NOT in the generic field container -- it needs the
// dedicated ZwExternalGeometryCopyDataGet inquiry. Carries WHERE the
// geometry was copied from (external file + root, or local entities).
struct GeomCopyData
{
    bool        ok = false;
    std::string source;            // "external" / "local"
    std::string file;              // external source file (external only)
    std::string root;              // root name within the source file
    int         entity_count = 0;  // number of copied entities
    int         associative  = 0;  // ezwAssociativeCopyType
    int         idxfer_rc    = -999;  // ZwEntityIdTransfer return (diag)
    int         get_rc       = -999;  // ZwExternalGeometryCopyDataGet return (diag)
};

// The geometry a feature PRODUCED (its associated entities), read via
// cvxPartInqFtrEnts. This is the fallback for features that don't expose
// editable parameters to the API (e.g. CdGeomCopy, a geometry-import):
// their value is the geometry they inject, not parameters. Counts let us
// see what a feature emits; sigs carry the geometry for downstream use.
struct ResultEnts
{
    int n_shape = 0;   // solids / facesets
    int n_face  = 0;
    int n_curve = 0;   // wireframe curves (profile geometry)
    std::vector<EntSig> shapes;
    std::vector<EntSig> curves;
};

// One curve of an extrude/revolve profile (a feature's built-in sketch),
// in world 3D coords. Only the fields its kind needs are set.
struct ProfileCurve
{
    std::string kind;                       // line / arc / circle / ellipse / nurb
    double p0[3]     = { 0.0, 0.0, 0.0 };   // start (line/arc/nurb)
    double p1[3]     = { 0.0, 0.0, 0.0 };   // end   (line/arc/nurb)
    double center[3] = { 0.0, 0.0, 0.0 };   // arc/circle/ellipse centre
    double radius    = 0.0;
    double a0 = 0.0, a1 = 0.0;              // arc start/end angle (degrees)
    bool   is_ref    = false;               // reference / projected geometry
                                            // (cvxSkInqRefById / GeomXById),
                                            // not a drawn 2D curve. A sketch
                                            // can use these as INNER loops:
                                            // R2900 Extrude48's rect carries
                                            // its tower/hex cutouts only
                                            // here, and dropping them made
                                            // the replay extrude a SOLID
                                            // block (+8.9 cm^3).
};

// An extrude/revolve profile: the curves of the feature's built-in
// sketch(es). This is what the scalar params (End E, draft, ...) act on.
// The curves come back in the sketch's LOCAL 2D frame (z==0 on the plane),
// so the plane is what positions them in the world -- a sketch drawn on a
// face at z=1.5 reports its curves at z==0, and only the plane carries the
// 1.5. Without it the reader would drop every profile onto world z==0.
struct Profile
{
    int n_sketch = 0;
    std::vector<ProfileCurve> curves;

    // Sketch insertion plane in WORLD coords (from ZwEntityMatrixGet on the
    // sketch handle): origin = matrix offset column, x_dir / normal = its
    // x / z axis columns. has_plane stays false for a profile we couldn't
    // resolve a plane for (reader then falls back to world XY at z==0).
    bool   has_plane = false;
    double origin[3] = { 0.0, 0.0, 0.0 };
    double x_dir [3] = { 1.0, 0.0, 0.0 };
    double normal[3] = { 0.0, 0.0, 1.0 };

    // Diagnostics for the "sketch found but 0 curves" case (R2900_100's
    // Extrude21/26/30/31 export n_profile_sketch=1 / n_profile_curve=0, so
    // they reconstruct as opaque). curvelist_rc is the LAST
    // ZwSketch2DCurveListGet return code and curvelist_cn its reported count.
    // CONFIRMED on R2900_100: rc==0, cn==0 -> the sketch exposes no native 2D
    // DRAWN curves; its profile is REFERENCE / projected geometry that
    // ZwSketch2DCurveListGet does not enumerate. The fallback below reads it
    // via cvxSkInqRefById (reference geometry) / cvxSkInqGeomXById (extended).
    // ref_cn / geomx_cn record how many curves each fallback found (-1 = the
    // fallback did not run, i.e. the 2D path already yielded curves).
    // -999 / -1 mean read_sketch_curves never ran (no sketch resolved).
    int    curvelist_rc = -999;
    int    curvelist_cn = -1;
    int    ref_cn       = -1;
    int    geomx_cn     = -1;
};

// One walked feature in history order.
struct FeatNode
{
    uint32_t    id      = 0;
    FeatKind    kind    = FeatKind::Opaque;
    std::string name;
    std::string zw_type;              // raw ZW3D type token (opaque only)

    SketchData  sketch;               // valid when kind == Sketch
    ExtrudeData extrude;              // valid when kind == Extrude
    BoxData     box;                  // valid when kind == Box
};

} // namespace

// ============================================================
// zwapi -- the binding surface. EVERY ZW3D SDK call lives here, and
// nowhere else (mirrors how SwReader confined all COM to one TU). A
// ZW3D feature is identified by an int feature id; the history walk and
// type dispatch below are bound to the real Vx C API. The per-feature
// geometry reads (ReadSketch / ReadExtrude) are still stubs -- they need
// a real part to map fields against, and the FIRST run reveals the exact
// template tokens to dispatch on (see MapFeatKind).
//
// All identifiers verified against this SDK's headers (api/inc):
//   cvxPartInqFtrList(int*, int**)            [zwapi_part_history.h]
//   cvxPartInqFtrTemplate(int, vxName)        [zwapi_part_history.h]
//   cvxEntName(int, char*, int)               [zwapi_general_ent.h]
//   cvxMemFree(void**)                        [zwapi_memory.h]
//   vxName == char[32], ZW_API_NO_ERROR       [zwapi_util.h]
// ============================================================

#include "zwapi_part_history.h"            // cvxPartInqFtrList / cvxPartInqFtrTemplate / cvxPartInqFtrData / cvxPartInqFtrEnts
#include "zwapi_memory.h"                  // cvxMemFree
#include "zwapi_util.h"                    // vxName, evxErrors, ZW_API_NO_ERROR, svxBndBox, svxPoint/Vector, VX_ENT_*
#include "zwapi_general_ent.h"             // cvxEntBndBox / cvxEntExists / cvxEntName (feature tree label)
#include "zwapi_brep_face.h"               // cvxFaceParam / cvxFaceEval (face normal for matching signatures)
#include "zwapi_brep_shape.h"              // cvxPartInqShapes / cvxPartInqShapeFaces / cvxPartInqShapeEdges /
                                           // cvxPartInqShapeMass (per-feature cumulative _state truth)
#include "zwapi_file.h"                    // cvxFileExportInit / cvxFileExport (STEP truth geometry)
#include "zwapi_entity.h"                  // ZwEntityIdTransfer / ZwEntityHandleFree (int id -> szwEntityHandle); ZwEntityMatrixGet
#include "zwapi_matrix_data.h"             // szwMatrix (sketch insertion-plane world transform)
#include "zwapi_datum.h"                   // ZwDatumAxisDirectionGet (pattern direction axis -> unit vector)
#include "zwapi_brep_edge.h"               // cvxPartInqEdgeCrv (on-curve edge anchor for SnapEnt)
#include "zwapi_part_objs.h"               // cvxCurveFree
#include "zwapi_dataexchange.h"            // ZwExternalGeometryCopyDataGet/Free, szwExternalGeometryCopyData (CdGeomCopy)
#include "zwapi_part_opts.h"               // cvxPartHistScrollTo (roll the history bar to read a feature in context)
#include "zwapi_history.h"                 // ZwHistoryReplay + ezwHistoryModelStopLinePosition (the real history roll-back)
#include "zwapi_sketch_general.h"          // ZwSketch2DCurveListGet (extrude profile = a feature's built-in sketch)
#include "zwapi_curve.h"                   // ZwCurveNURBSDataGet
#include "zwapi_curve_data.h"              // szwCurve (line/arc/circle geometry)
#include "zwapi_cmd_paramdefine_param.h"   // cvxDataGetAll / cvxDataGetNum / cvxDataGetPnt / cvxDataGetEnts /
                                           // cvxDataGetText / cvxFldDataFree / cvxDataFree;
                                           // svxFldData, evxFldType (VX_FLD_NUM / DST / ANG / PNT / ENT / TXT / DATA)
// Pending the per-feature reads (bind when wiring ReadSketch/ReadExtrude):
//   #include "zwapi_sk_data.h"        // sketch geometry entities
//   #include "zwapi_sk_cons.h"        // sketch constraints
//   #include "zwapi_sk_dim.h"         // sketch dimensions
//   #include "zwapi_cmd_shape_data.h" // extrude parameter struct

#include "zwapi_sk_objs.h"                 // cvxSkInqRefById / cvxSkInqGeomXById
                                           // (sketch REFERENCE / EXTENDED geometry):
                                           // ReadProfile's fallback when
                                           // ZwSketch2DCurveListGet finds no drawn
                                           // 2D curves -- a reference-geometry
                                           // profile (R2900_100 Extrude21/26/30/31,
                                           // confirmed _diag rc==0/cn==0).

namespace zwapi
{

// TODO: bind -- length unit of the active part as a string token. ZW3D
// parts are commonly millimetres; confirm against the active part's unit
// setting and return one of the unit:: tokens.
std::string LengthUnit()
{
    return sc::unit::Mm;
}

// TODO: bind -- active part display name (diagnostics only).
std::string ActivePartName()
{
    return "part";
}

// Feature ids of the active part, in history order. The list (including
// hidden features) is allocated by ZW3D and freed here with cvxMemFree.
std::vector<int> HistoryFeatures()
{
    std::vector<int> out;
    int   count = 0;
    int*  ids   = nullptr;
    if (cvxPartInqFtrList(&count, &ids) != ZW_API_NO_ERROR || ids == nullptr) {
        return out;
    }
    out.assign(ids, ids + count);
    cvxMemFree(reinterpret_cast<void**>(&ids));
    return out;
}

// Scroll the history rollback bar to a feature so its data re-evaluates
// in the model state where it executed. cvxPartInqFtrData otherwise
// re-evaluates against the CURRENT (final) state -- the documented reason
// an early feature (e.g. the leading CdGeomCopy) reads back empty. This
// MUTATES the live document; the export restores the bar to the end when
// done.
int ScrollHistoryTo(int idFtr)
{
    return static_cast<int>(cvxPartHistScrollTo(idFtr));
}

// Diagnostic probe: report the return codes of the data-access path for a
// feature, so an empty export tells us WHERE it failed instead of leaving
// us to guess. data_rc is cvxPartInqFtrData's return (0 = ok, -1 = data
// undefined, other = error enum); field_count is how many fields the
// container yielded.
struct FtrProbe
{
    int data_rc     = -999;
    int field_count = -1;
};

FtrProbe ProbeFeature(int idFtr)
{
    FtrProbe p;
    int idData = 0;
    p.data_rc = cvxPartInqFtrData(idFtr, 0, &idData);
    if (p.data_rc == ZW_API_NO_ERROR)
    {
        int         n = 0;
        svxFldData* f = nullptr;
        if (cvxDataGetAll(idData, &n, &f) == ZW_API_NO_ERROR)
        {
            p.field_count = n;
            cvxFldDataFree(n, &f);
        }
        cvxDataFree(idData);
    }
    return p;
}

// Stable, language-independent feature type token: the feature's command
// template name, the same string regardless of UI language (a Chinese
// install still reports the template, not the localized "草图1"). This is
// the ZW3D analogue of SolidWorks' GetTypeName2.
std::string FeatureType(int idFtr)
{
    vxName tmpl;
    tmpl[0] = '\0';
    if (cvxPartInqFtrTemplate(idFtr, tmpl) != ZW_API_NO_ERROR) {
        return std::string();
    }
    return std::string(tmpl);
}

// Convert a ZW3D API string to guaranteed-valid UTF-8 for the neutral JSON.
// Modern ZW3D SDKs hand back UTF-8, but older ones (and some locales) return
// system ANSI (e.g. GBK). nlohmann's dump() runs with error_handler::replace,
// so raw GBK bytes would be emitted as U+FFFD -- a garbled "倒角1" on the cax
// side. Pass valid UTF-8 through untouched; transcode anything else from the
// system ANSI code page.
std::string ToUtf8(const char* s)
{
#ifdef _WIN32
    if (!s || !*s) {
        return std::string();
    }
    if (MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, s, -1, nullptr, 0) > 0) {
        return std::string(s);   // already valid UTF-8
    }
    int wlen = MultiByteToWideChar(CP_ACP, 0, s, -1, nullptr, 0);
    if (wlen <= 0) {
        return std::string(s);
    }
    std::wstring w(static_cast<size_t>(wlen), L'\0');
    MultiByteToWideChar(CP_ACP, 0, s, -1, &w[0], wlen);
    int u8 = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0, nullptr, nullptr);
    if (u8 <= 0) {
        return std::string(s);
    }
    std::string out(static_cast<size_t>(u8), '\0');
    WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, &out[0], u8, nullptr, nullptr);
    if (!out.empty() && out.back() == '\0') {
        out.pop_back();          // the -1 length added a trailing NUL
    }
    return out;
#else
    return s ? std::string(s) : std::string();
#endif
}

// Display name: the feature's user-facing name from the history tree -- the
// localized label like "草图1" / "倒角1" on a Chinese install -- via the entity
// name getter cvxEntName (the ZW3D analogue of SolidWorks' IFeature::GetName()).
// A feature id is an entity id, so cvxEntName returns its tree name. Falls back
// to the stable template token when the feature has no name, so a node is never
// left unnamed. NOTE: type DISPATCH stays on FeatureType() / "zw_type" (a stable
// language-independent token), so the real name here never changes how the cax
// reader maps the feature -- it only sets the display label.
std::string FeatureName(int idFtr)
{
    char name[256];
    name[0] = '\0';
    if (cvxEntName(idFtr, name, static_cast<int>(sizeof(name))) == ZW_API_NO_ERROR
        && name[0] != '\0')
    {
        return ToUtf8(name);
    }
    return FeatureType(idFtr);
}

// Map a ZW3D feature template token to our FeatKind. The exact tokens for
// sketch / extrude are NOT yet confirmed against a real part, so for now
// everything maps to Opaque -- which makes the FIRST export
// self-documenting: every feature lands as opaque carrying its true
// template string in "zw_type", so you read those tokens straight out of
// the emitted JSON and fill the cases below (then bind ReadSketch /
// ReadExtrude for the kinds you recognise).
FeatKind MapFeatKind(const std::string& tmpl)
{
    // Observed from a real part (a ZW3D Block primitive):
    if (tmpl == "FtAllBox") { return FeatKind::Box; }
    // TODO: bind once observed:
    //   if (tmpl == "FtSketch")  { return FeatKind::Sketch; }
    //   if (tmpl == "FtExtrude") { return FeatKind::Extrude; }
    return FeatKind::Opaque;
}

// TODO: bind -- read a sketch feature's plane + geometry + constraints
// into the neutral SketchData. Geometry entities: zwapi_sk_data.h;
// constraints: zwapi_sk_cons.h; dimensions: zwapi_sk_dim.h. Coordinates
// come back sketch-local, angles in radians. ZW3D analogue of SwReader's
// ReadSketchPlane + the geometry/constraint walk.
void ReadSketch(int idFtr, SketchData& out)
{
    (void)idFtr;
    (void)out;
}

// TODO: bind -- read an extrude feature's parameters + the profile sketch
// it consumes into ExtrudeData. Fetch the feature's data container via
// cvxPartInqFtrData(idFtr, 0, &idData), then read the extrude parameter
// struct (zwapi_cmd_shape_data.h).
void ReadExtrude(int idFtr, ExtrudeData& out)
{
    (void)idFtr;
    (void)out;
}

// Read a numeric field (number / distance / angle) by its stable field
// id from the feature's data container. Returns false (out untouched) if
// the field is absent / non-numeric.
bool ReadFtrNum(int idFtr, int fld_id, double& out)
{
    int idData = 0;
    if (cvxPartInqFtrData(idFtr, 0, &idData) != ZW_API_NO_ERROR) {
        return false;
    }
    double v = 0.0;
    bool ok = (cvxDataGetNum(idData, fld_id, &v) == ZW_API_NO_ERROR);
    cvxDataFree(idData);
    if (ok) {
        out = v;
    }
    return ok;
}

// World-space min-corner of the feature's resulting b-rep shape, read
// from its axis-aligned bounding box. This is MODE-INDEPENDENT: it reads
// the actual geometry, not the Block's defining points (which differ by
// creation mode -- center+corner vs two corners), so it sidesteps all
// that ambiguity. For an axis-aligned box the AABB min-corner IS the box
// corner, exactly where OCCT's origin-anchored MakeBox must be translated.
// Limitation: a rotated box's AABB min is not its corner (rotation is
// future work); and this reads the feature's shape as it stands, which
// for a root primitive is the box as created.
bool ReadShapeMinCorner(int idFtr, double out[3])
{
    int  cnt  = 0;
    int* ents = nullptr;
    if (cvxPartInqFtrEnts(idFtr, VX_ENT_SHAPE, &cnt, &ents) != ZW_API_NO_ERROR ||
        ents == nullptr || cnt < 1)
    {
        if (ents != nullptr) {
            cvxMemFree(reinterpret_cast<void**>(&ents));
        }
        return false;
    }

    svxBndBox box;
    bool ok = (cvxEntBndBox(ents[0], &box) == ZW_API_NO_ERROR);
    cvxMemFree(reinterpret_cast<void**>(&ents));
    if (!ok) {
        return false;
    }

    out[0] = box.X.min;
    out[1] = box.Y.min;
    out[2] = box.Z.min;
    return true;
}

// Read a ZW3D Block (FtAllBox) primitive's dimensions + placement. Field
// ids are the stable indices observed from a real part's data container
// (Length=3, Width=4, Height=5). We dispatch on the id, NOT the label:
// the data carries TWO fields named "Height" (5 and 12) and the labels
// are localizable, so the id is the only reliable key. Height is signed
// in ZW3D (the box can grow along -Z); OCCT's MakeBox needs positive
// extents, so magnitudes are taken here, and the placement (min-corner)
// carries the real position.
void ReadBox(int idFtr, BoxData& out)
{
    double v = 0.0;
    if (ReadFtrNum(idFtr, 3, v)) { out.length = std::fabs(v); }
    if (ReadFtrNum(idFtr, 4, v)) { out.width  = std::fabs(v); }
    if (ReadFtrNum(idFtr, 5, v)) { out.height = std::fabs(v); }
    out.has_place = ReadShapeMinCorner(idFtr, out.place);
}

// Maps a ZW3D feature id -> the 1-based ordinal the exporter writes as the
// JSON feature "id" (FeatNode.id == loop index + 1). Populated once at the
// start of the export (SetJsonIdMap) so EntitySig can stamp each referenced
// entity with the JSON id of its OWNING feature. ZW feature ids and the JSON
// ordinal are DIFFERENT id spaces -- cvxPartInqEntFtr returns a ZW id, which
// only this map can translate into the id the reader keys features by.
std::unordered_map<int, int> g_zwfid_to_jsonid;

void SetJsonIdMap(const std::vector<int>& feats)
{
    g_zwfid_to_jsonid.clear();
    for (size_t i = 0; i < feats.size(); ++i) {
        g_zwfid_to_jsonid[feats[i]] = static_cast<int>(i + 1);
    }
}

// Geometric signature of a referenced entity, for later matching in OCCT.
// Faces get an on-surface point (evaluated at mid-UV) + normal; anything
// else gets its bounding-box centre. The ZW3D entity id is NOT recorded
// -- it is meaningless once the part is rebuilt in OCCT; only geometry
// survives a cross-kernel rebuild. The OWNING feature (cvxPartInqEntFtr,
// translated to the JSON ordinal) IS recorded: it survives as feature
// identity the reader uses to wire pattern Base links structurally.
EntSig EntitySig(int idEnt)
{
    EntSig s;
    if (cvxEntExists(idEnt, VX_ENT_FACE)) { s.kind = "face"; }
    else if (cvxEntExists(idEnt, VX_ENT_EDGE)) { s.kind = "edge"; }
    else { s.kind = "ent"; }
    s.blanked = (cvxEntIsBlanked(idEnt) == 1);

    int owner_fid = 0;
    if (cvxPartInqEntFtr(idEnt, &owner_fid) == ZW_API_NO_ERROR) {
        auto it = g_zwfid_to_jsonid.find(owner_fid);
        if (it != g_zwfid_to_jsonid.end()) {
            s.feat     = it->second;
            s.has_feat = true;
        }
    }

    svxBndBox box;
    if (cvxEntBndBox(idEnt, &box) == ZW_API_NO_ERROR)
    {
        s.anchor[0] = 0.5 * (box.X.min + box.X.max);
        s.anchor[1] = 0.5 * (box.Y.min + box.Y.max);
        s.anchor[2] = 0.5 * (box.Z.min + box.Z.max);
    }

    if (s.kind == "face")
    {
        svxLimit U, V;
        if (cvxFaceParam(idEnt, &U, &V) == ZW_API_NO_ERROR)
        {
            svxPoint  p;
            svxVector n;
            if (cvxFaceEval(idEnt, 0.5 * (U.min + U.max), 0.5 * (V.min + V.max),
                            &p, &n) == ZW_API_NO_ERROR)
            {
                s.anchor[0] = p.x; s.anchor[1] = p.y; s.anchor[2] = p.z;
                s.normal[0] = n.x; s.normal[1] = n.y; s.normal[2] = n.z;
                s.has_normal = true;
            }
        }
    }
    else if (s.kind == "edge")
    {
        // ON-CURVE anchor for curved edges. The bbox-centre fallback above
        // sits R/2 (half-circle) to a FULL RADIUS (full circle) off an arc's
        // curve -- far past the resolver tolerance of a small dressup
        // (R2900: 0.5 mm fillets on R 2.4..10.5 mm rims got tol 2.6 mm, so
        // Fillet6/10 MISSed outright and Fillet7/Chamfer6 resolved a WRONG
        // nearby edge). cvxPartInqEdgeCrv answers on the same consumed-state
        // entity that cvxEntBndBox does, and lines/arcs/circles come back
        // ANALYTIC: the arc's mid-angle point (or the line midpoint) is
        // exactly ON the curve, which is what TopoRefResolver scores by.
        // NURB edges keep the bbox centre -- no cheap exact eval, and the
        // reader side has an arc-aware disc fallback for legacy snapshots.
        svxCurve crv = {};
        if (cvxPartInqEdgeCrv(idEnt, 0, &crv) == ZW_API_NO_ERROR)
        {
            if (crv.Type == VX_CRV_LINE)
            {
                s.anchor[0] = 0.5 * (crv.P1.x + crv.P2.x);
                s.anchor[1] = 0.5 * (crv.P1.y + crv.P2.y);
                s.anchor[2] = 0.5 * (crv.P1.z + crv.P2.z);
            }
            else if ((crv.Type == VX_CRV_ARC || crv.Type == VX_CRV_CIRCLE) &&
                     crv.R > 0.0)
            {
                double a1 = crv.A1;
                double a2 = crv.A2;
                if (crv.Type == VX_CRV_CIRCLE && std::fabs(a2 - a1) < 1e-9) {
                    a2 = a1 + 360.0;   // degenerate full-circle range
                }
                if (a2 < a1) { a2 += 360.0; }
                const double am = 0.5 * (a1 + a2)
                                * 3.14159265358979323846 / 180.0;
                const double c  = std::cos(am);
                const double sn = std::sin(am);
                // Frame axes are stored as ROWS: X = (xx,xy,xz), Y =
                // (yx,yy,yz), origin (xt,yt,zt) = arc centre. P = O +
                // R(cos*X + sin*Y). Verified empirically on R2900
                // Fillet6's cylinder rim: the column reading put the
                // radial offset into the rim plane's NORMAL component
                // (anchor y 82.27 vs the rim plane's y 92.73).
                const svxMatrix& m = crv.Frame;
                s.anchor[0] = m.xt + crv.R * (c * m.xx + sn * m.yx);
                s.anchor[1] = m.yt + crv.R * (c * m.xy + sn * m.yy);
                s.anchor[2] = m.zt + crv.R * (c * m.xz + sn * m.yz);
            }
            cvxCurveFree(&crv);
        }
    }
    // A datum PLANE has no face to evaluate, so the face branch above leaves it
    // with only a bbox-centre anchor and no normal. A mirror / symmetry feature
    // (FtMirrorFtr) references such a datum as its mirror plane (fld "Plane"),
    // and the reader needs the plane's NORMAL to reflect across it. Recover it
    // from the datum's world matrix the same way read_sketch_curves does for a
    // sketch insertion plane: szwMatrix's z-axis column (zx,zy,zz) is the plane
    // normal and the offset column (xt,yt,zt) a point ON the plane. (The datum's
    // bbox centre is already in-plane, so the existing anchor stays valid as the
    // plane point; we only add the missing normal. A datum AXIS has no single
    // meaningful normal, but a mirror plane is a datum PLANE, not an axis.)
    if (!s.has_normal && cvxEntExists(idEnt, VX_ENT_DATUM))
    {
        szwEntityHandle h;
        if (ZwEntityIdTransfer(1, &idEnt, &h) == ZW_API_NO_ERROR)
        {
            szwMatrix m;
            if (ZwEntityMatrixGet(h, &m) == ZW_API_NO_ERROR)
            {
                s.normal[0] = m.zx; s.normal[1] = m.zy; s.normal[2] = m.zz;
                s.has_normal = true;
            }
            ZwEntityHandleFree(&h);
        }
    }
    // NOTE: the bbox centre remains the fallback anchor when the edge branch
    // above could not improve it (cvxPartInqEdgeCrv failed, or a NURB edge).
    // For a curved edge the bbox centre is NOT on the curve (a circular rim's
    // centre is up to a full radius off it) while TopoRefResolver scores
    // edges by point-to-curve distance -- the reader compensates with an
    // arc-aware disc fallback, but an analytic on-curve anchor (edge branch
    // above) is always preferred when ZW3D will answer the curve inquiry.
    return s;
}

// A chamfer / fillet edge list is a nested VDATA TREE, not a flat list: the
// field's items are themselves sub-containers, each holding the picked edge(s)
// (idEntity) and the per-edge setback (Num) one or more levels down. The flat
// accessors (cvxDataGetEnts / cvxDataGetList) only see the top level and
// returned nothing for the chamfer, so walk the tree with the per-ITEM API:
// cvxDataGetItemList gives item handles, and each handle is BOTH an item
// (cvxDataGetItemData -> its svxData) AND a sub-container (cvxDataGetAll ->
// its own fields, whose VX_FLD_DATA sub-fields recurse). Collect every edge id
// (deduped) and the first non-zero number encountered as the setback.
void CollectVDataTree(int handle,
                      std::vector<int>& edge_ids,
                      double&           setback,
                      bool&             has_setback,
                      int               depth)
{
    if (depth > 6) {
        return;                         // guard against a pathological cycle
    }
    auto add_edge = [&](int id)
    {
        if (id <= 0) { return; }
        for (int e : edge_ids) { if (e == id) { return; } }   // dedup
        edge_ids.push_back(id);
    };

    // (a) the handle AS A LEAF ITEM: its svxData may carry the edge + setback.
    svxData sd;
    cvxDataZero(&sd);
    if (cvxDataGetItemData(handle, &sd) == ZW_API_NO_ERROR)
    {
        if (sd.isEntity) { add_edge(sd.idEntity); }
        if (sd.isNumber && !has_setback && sd.Num != 0.0) {
            setback     = sd.Num;
            has_setback = true;
        }
    }

    // (b) the handle AS A SUB-CONTAINER: walk its fields, pulling entities and
    //     the first non-zero number directly, and recursing into nested lists.
    int         numFld = 0;
    svxFldData* flds   = nullptr;
    if (cvxDataGetAll(handle, &numFld, &flds) == ZW_API_NO_ERROR && flds != nullptr)
    {
        for (int i = 0; i < numFld; ++i)
        {
            const svxFldData& f = flds[i];
            if (f.fld_type == VX_FLD_ENT)
            {
                int  cnt = 0;
                int* ids = nullptr;
                if (cvxDataGetEnts(handle, f.fld_id, &cnt, &ids) == ZW_API_NO_ERROR && ids != nullptr)
                {
                    for (int k = 0; k < cnt; ++k) { add_edge(ids[k]); }
                    cvxMemFree(reinterpret_cast<void**>(&ids));
                }
            }
            else if ((f.fld_type == VX_FLD_NUM || f.fld_type == VX_FLD_DST) && !has_setback)
            {
                double v = 0.0;
                if (cvxDataGetNum(handle, f.fld_id, &v) == ZW_API_NO_ERROR && v != 0.0) {
                    setback     = v;
                    has_setback = true;
                }
            }
            else if (f.fld_type == VX_FLD_DATA)
            {
                int  count = 0;
                int* items = nullptr;
                if (cvxDataGetItemList(handle, f.fld_id, &count, &items) == ZW_API_NO_ERROR && items != nullptr)
                {
                    for (int k = 0; k < count; ++k) {
                        CollectVDataTree(items[k], edge_ids, setback, has_setback, depth + 1);
                    }
                    cvxMemFree(reinterpret_cast<void**>(&items));
                }
            }
        }
        cvxFldDataFree(numFld, &flds);
    }
}

// Full dump of a feature's data container: EVERY field, by type. Unlike
// the earlier scalar-only dump, this also captures points, text, and
// (crucially) entity references with a geometric signature -- so a
// reference-driven feature's real inputs (which face/edge it operates on,
// where) are visible, instead of just secondary scalars. This is what a
// part like dkba80218026 needs to even judge reconstructability.
std::vector<FieldDump> DumpFields(int idFtr)
{
    std::vector<FieldDump> out;

    int idData = 0;
    if (cvxPartInqFtrData(idFtr, 0, &idData) != ZW_API_NO_ERROR) {
        return out;
    }

    int         numFld  = 0;
    svxFldData* fldData = nullptr;
    if (cvxDataGetAll(idData, &numFld, &fldData) == ZW_API_NO_ERROR && fldData != nullptr)
    {
        for (int i = 0; i < numFld; ++i)
        {
            const svxFldData& f = fldData[i];

            FieldDump d;
            d.id   = f.fld_id;
            d.name = f.fld_name;   // may be empty / localized

            if (f.fld_type == VX_FLD_NUM ||
                f.fld_type == VX_FLD_DST ||
                f.fld_type == VX_FLD_ANG)
            {
                d.type = (f.fld_type == VX_FLD_NUM) ? "number"
                       : (f.fld_type == VX_FLD_DST) ? "distance" : "angle";
                double v = 0.0;
                if (cvxDataGetNum(idData, f.fld_id, &v) == ZW_API_NO_ERROR) {
                    d.num = v;
                    d.has_num = true;
                }
            }
            else if (f.fld_type == VX_FLD_PNT)
            {
                d.type = "point";
                svxPoint p;
                if (cvxDataGetPnt(idData, f.fld_id, &p) == ZW_API_NO_ERROR) {
                    d.pt[0] = p.x; d.pt[1] = p.y; d.pt[2] = p.z;
                    d.has_pt = true;
                }
            }
            else if (f.fld_type == VX_FLD_ENT)
            {
                d.type = "entity";
                int  cnt = 0;
                int* ids = nullptr;
                if (cvxDataGetEnts(idData, f.fld_id, &cnt, &ids) == ZW_API_NO_ERROR && ids != nullptr)
                {
                    for (int k = 0; k < cnt; ++k) {
                        if (ids[k] > 0) {
                            d.ents.push_back(EntitySig(ids[k]));
                        }
                    }
                    cvxMemFree(reinterpret_cast<void**>(&ids));
                }
            }
            else if (f.fld_type == VX_FLD_TXT)
            {
                d.type = "text";
                char buf[512];
                buf[0] = '\0';
                if (cvxDataGetText(idData, f.fld_id, static_cast<int>(sizeof(buf)), buf) == ZW_API_NO_ERROR) {
                    d.text = buf;
                    d.has_text = true;
                }
            }
            else if (f.fld_type == VX_FLD_DATA)
            {
                d.type = "list";   // a nested VDATA tree (e.g. a chamfer /
                                   // fillet edge list). Walk it per-item with
                                   // cvxDataGetItemList + CollectVDataTree to
                                   // pull the picked edge(s) + setback that the
                                   // flat accessors can't see.
                std::vector<int> edge_ids;
                double           sb     = 0.0;
                bool             has_sb = false;

                int   count = 0;
                int*  items = nullptr;
                if (cvxDataGetItemList(idData, f.fld_id, &count, &items) == ZW_API_NO_ERROR && items != nullptr)
                {
                    for (int k = 0; k < count; ++k) {
                        CollectVDataTree(items[k], edge_ids, sb, has_sb, 0);
                    }
                    cvxMemFree(reinterpret_cast<void**>(&items));
                }
                d.list_count = count;   // diagnostic: top-level item count

                // Last-resort flat fallbacks (kept for non-tree list widgets).
                if (edge_ids.empty())
                {
                    int  cnt = 0;
                    int* ids = nullptr;
                    if (cvxDataGetEnts(idData, f.fld_id, &cnt, &ids) == ZW_API_NO_ERROR && ids != nullptr)
                    {
                        for (int k = 0; k < cnt; ++k) {
                            if (ids[k] > 0) { edge_ids.push_back(ids[k]); }
                        }
                        cvxMemFree(reinterpret_cast<void**>(&ids));
                    }
                }

                for (int eid : edge_ids) {
                    EntSig sig = EntitySig(eid);
                    if (has_sb) { sig.has_num = true; sig.num = sb; }
                    d.ents.push_back(sig);
                }
                if (has_sb) {
                    d.has_num = true;
                    d.num     = sb;
                }
            }
            else
            {
                d.type = "other";
            }

            // A "Direction" widget (a pattern's fld 2 "Direction" / fld 5
            // "Direction D", a revolve axis, ...) stores its resolved UNIT
            // DIRECTION in the field data's Dir member (isDirection=1), NOT in
            // Pnt -- cvxDataGetPnt returns zeros for it (that is why fld 5 came
            // out [0,0,0]). The reader's edge-derived pattern.dir takes the
            // referenced edge's PARAMETRIC orientation, whose sign (and even
            // axis) is arbitrary, so the reader prefers this field Dir: it is
            // ZW3D's true signed pattern direction. Emit it for any field that
            // carries one. (cvxDataGetAll fills fld_data for scalar/point/dir
            // fields; it is the nested VX_FLD_DATA *list* that needed the
            // per-item tree walk, not this.)
            if (f.fld_data != nullptr) {
                bool dir_done = false;
                for (int k = 0; k < f.count; ++k) {
                    const svxData& fd = f.fld_data[k];
                    if (!dir_done && fd.isDirection) {
                        const svxVector& dv = fd.Dir;
                        if (dv.x != 0.0 || dv.y != 0.0 || dv.z != 0.0) {
                            d.dir[0] = dv.x;
                            d.dir[1] = dv.y;
                            d.dir[2] = dv.z;
                            d.has_dir = true;
                            dir_done  = true;
                        }
                    }
                    // Every point this field carries (a PNT_TO_PNT pattern's
                    // "To points" holds one per instance) -- cvxDataGetPnt
                    // above only saw the first.
                    if (fd.isPoint) {
                        d.pts.push_back({ fd.Pnt.x, fd.Pnt.y, fd.Pnt.z });
                    }
                }
            }

            out.push_back(std::move(d));
        }
        cvxFldDataFree(numFld, &fldData);
    }

    cvxDataFree(idData);
    return out;
}

// ---- History stop-line positioning -----------------------------------------
// The export walks the model through history states so each feature is read in
// the state it actually operated in. ZwHistoryReplay moves the model stop-line
// (the real roll -- cvxPartHistScrollTo is a no-op for the body: proven on
// test.Z3PRT, faces stayed 7->7, only ZwHistoryReplay moved it 7->6) and is
// RELATIVE to the current position (SDK contract: "if the entity is played it
// rolls back to it; if not played it plays forward to it"). So a single forward
// sweep -- roll to BEGIN once, then ask for each feature's BEFORE / AFTER
// position in history order -- only ever plays forward; the body is regenerated
// once end-to-end, never per-feature (no O(N^2) replay).
//
// We read a feature's INPUTS in its BEFORE state (its picked faces/edges are
// still live -- a dressup hasn't consumed its edge yet, so the input resolves to
// the ORIGINAL, e.g. box top edge @ (5,0,5) not the chamfer boundary @ (-3,0,5))
// and its OUTPUTS in its AFTER state (its result shapes exist). This matters
// because the reader rebuilds incrementally and matches each feature's refs
// against THAT same per-feature state; reading at the final body would hand it
// post-consumption / post-modification geometry. The old code did this roll for
// dressup edges only; here it is uniform for every feature, in one sweep.

void RollBodyToBegin()
{
    ZwHistoryReplay(nullptr, ZW_HISTORY_REPLAY_TO_BEGIN);   // the single rollback
}

void RollBodyToEnd()
{
    ZwHistoryReplay(nullptr, ZW_HISTORY_REPLAY_TO_THE_END);
}

// Play the body to just BEFORE / AFTER a feature. Called in history order from
// the begin, so each call only plays FORWARD (for BEFORE the target feature
// isn't played yet; AFTER plays exactly this one feature) -- never a rollback.
// Returns true on success.
bool RollBody(int idFtr, ezwHistoryModelStopLinePosition pos)
{
    szwEntityHandle h;
    bool ok = false;
    if (ZwEntityIdTransfer(1, &idFtr, &h) == ZW_API_NO_ERROR) {
        ok = (ZwHistoryReplay(&h, pos) == ZW_API_NO_ERROR);
        ZwEntityHandleFree(&h);
    }
    return ok;
}

bool RollBodyBefore(int idFtr) { return RollBody(idFtr, ZW_HISTORY_REPLAY_BEFORE_FEATURE); }
bool RollBodyAfter (int idFtr) { return RollBody(idFtr, ZW_HISTORY_REPLAY_AFTER_FEATURE);  }

// The model EDGES a feature consumes as input, read AT THE CURRENT stop-line --
// the caller has already rolled the body to this feature's BEFORE state, where a
// dressup's picked edge is still the original (not the post-chamfer boundary).
// No rolling here: that is the caller's single forward sweep. Each edge becomes a
// geometric signature; the ZW3D id is meaningless after the OCCT rebuild, only
// the geometry the loader matches on survives. (cvxPartFtrInqInpEnts allocates
// its list for ZwMemoryFree, not the cvxMemFree the older cvxPart* inquiries
// use -- same as ReadProfile.)
std::vector<EntSig> InputEdgesAtCurrentState(int idFtr)
{
    std::vector<EntSig> out;
    int  cnt = 0;
    int* ids = nullptr;
    if (cvxPartFtrInqInpEnts(idFtr, VX_ENT_EDGE, &cnt, &ids) == ZW_API_NO_ERROR && ids != nullptr)
    {
        for (int i = 0; i < cnt; ++i) {
            if (ids[i] > 0) {
                out.push_back(EntitySig(ids[i]));
            }
        }
        ZwMemoryFree(reinterpret_cast<void**>(&ids));
    }
    return out;
}

// One curvilinear entity captured as world-mm geometry. Analytic when
// ZW3D classifies it (line / arc / circle), raw NURBS otherwise.
struct CurveGeo
{
    int    type = 0;                  // evxCurveType
    double p0[3] = { 0, 0, 0 };       // line start
    double p1[3] = { 0, 0, 0 };       // line end
    double origin[3] = { 0, 0, 0 };   // arc/circle centre (Frame origin)
    double xaxis[3]  = { 1, 0, 0 };   // Frame X axis
    double yaxis[3]  = { 0, 1, 0 };   // Frame Y axis
    double radius = 0.0;
    double a1 = 0.0, a2 = 0.0;        // arc start/end angles (degrees)
    int                 degree = 0;   // NURB
    bool                rational = false;
    int                 cp_dim = 0;   // coords per control point (1-4)
    std::vector<double> knots;
    std::vector<double> cps;          // num_cp * cp_dim raw coords
};

// Per-row probe of a field's entity picks; rides _diag so a path whose
// rows carry NEITHER idEntity nor entPath (R2900 Sweep2/4) is visible
// from the snapshot instead of just "n_path_curve=0".
struct FieldEntRow
{
    int idEntity   = 0;
    int idParent   = 0;
    int path_count = 0;
    int path_last  = 0;
    int isPntOnCrv = 0;
};

// Read every curvilinear entity referenced by field `fld_id` of feature
// `idFtr` -- e.g. a sweep's "Path P2" point-on-curve pick, whose svxData
// rows carry the picked curve in idEntity (the generic field dump keeps
// only the pick POINT, which is useless for rebuilding the spine). Must
// run in the feature's BEFORE state, where the path curve is live.
// cvxPartInqCurve accepts both wireframe curves and brep edges.
//
// A SKETCH-member curve comes back in the sketch's LOCAL frame (z == 0;
// R2900's Sweep1/5 lines landed at z=0 while their pins live at y~20):
// when the pick row names a parent, transform by the parent's world
// matrix (axes as rows, axis-major names: X=(xx,xy,xz), O=(xt,yt,zt) --
// same convention the datum-plane normal recovery uses).
std::vector<CurveGeo> ReadFieldCurves(int idFtr, int fld_id,
                                      std::vector<FieldEntRow>* rows = nullptr)
{
    std::vector<CurveGeo> out;
    int idData = 0;
    if (cvxPartInqFtrData(idFtr, 0, &idData) != ZW_API_NO_ERROR) {
        return out;
    }

    int         numFld = 0;
    svxFldData* flds   = nullptr;
    if (cvxDataGetAll(idData, &numFld, &flds) != ZW_API_NO_ERROR ||
        flds == nullptr)
    {
        return out;
    }
    std::vector<int> ids;
    std::vector<int> parents;
    for (int i = 0; i < numFld; ++i)
    {
        if (flds[i].fld_id != fld_id || flds[i].fld_data == nullptr) {
            continue;
        }
        for (int k = 0; k < flds[i].count; ++k)
        {
            const svxData& sd = flds[i].fld_data[k];
            if (rows != nullptr)
            {
                FieldEntRow r;
                r.idEntity   = sd.idEntity;
                r.idParent   = sd.idParent;
                r.path_count = sd.entPath.Count;
                r.path_last  = (sd.entPath.Count > 0 &&
                                sd.entPath.Count <= V_PP_LEN)
                                   ? sd.entPath.Id[sd.entPath.Count - 1]
                                   : 0;
                r.isPntOnCrv = sd.isPntOnCrv;
                rows->push_back(r);
            }
            int id = sd.idEntity;
            // A pick NESTED inside another object leaves idEntity 0 and
            // carries the route in entPath; the LAST path element is
            // the child entity to act upon (svxEntPath doc).
            if (id <= 0 && sd.entPath.Count > 0 &&
                sd.entPath.Count <= V_PP_LEN) {
                id = sd.entPath.Id[sd.entPath.Count - 1];
            }
            if (id <= 0) { continue; }
            bool dup = false;
            for (int e : ids) { if (e == id) { dup = true; break; } }
            if (!dup) { ids.push_back(id); parents.push_back(sd.idParent); }
        }
    }
    cvxFldDataFree(numFld, &flds);

    // Nested VDATA tree fallback: an extrude's "Profile P" (fld 1) keeps
    // its picks one or more levels down a sub-container tree, invisible
    // to the flat fld_data rows above -- the same shape the chamfer /
    // fillet edge lists have. Walk it with the per-item API.
    if (ids.empty())
    {
        int  count = 0;
        int* items = nullptr;
        if (cvxDataGetItemList(idData, fld_id, &count, &items) ==
                ZW_API_NO_ERROR && items != nullptr)
        {
            std::vector<int> tree_ids;
            double           sb     = 0.0;
            bool             has_sb = false;
            for (int k = 0; k < count; ++k) {
                CollectVDataTree(items[k], tree_ids, sb, has_sb, 0);
            }
            cvxMemFree(reinterpret_cast<void**>(&items));
            for (int tid : tree_ids)
            {
                if (tid <= 0) { continue; }
                bool dup = false;
                for (int e : ids) { if (e == tid) { dup = true; break; } }
                if (dup) { continue; }
                ids.push_back(tid);
                parents.push_back(0);
                if (rows != nullptr)
                {
                    FieldEntRow r;
                    r.idEntity = tid;
                    rows->push_back(r);
                }
            }
        }
    }

    for (size_t idx = 0; idx < ids.size(); ++idx)
    {
        const int id     = ids[idx];
        const int parent = parents[idx];
        svxCurve crv = {};
        if (cvxPartInqCurve(id, 0, &crv) != ZW_API_NO_ERROR) {
            continue;
        }
        CurveGeo g;
        g.type = static_cast<int>(crv.Type);
        if (crv.Type == VX_CRV_LINE)
        {
            g.p0[0] = crv.P1.x; g.p0[1] = crv.P1.y; g.p0[2] = crv.P1.z;
            g.p1[0] = crv.P2.x; g.p1[1] = crv.P2.y; g.p1[2] = crv.P2.z;
        }
        else if (crv.Type == VX_CRV_ARC || crv.Type == VX_CRV_CIRCLE)
        {
            // Frame axes as ROWS (see the SnapEnt edge-anchor note).
            const svxMatrix& m = crv.Frame;
            g.origin[0] = m.xt; g.origin[1] = m.yt; g.origin[2] = m.zt;
            g.xaxis[0]  = m.xx; g.xaxis[1]  = m.xy; g.xaxis[2]  = m.xz;
            g.yaxis[0]  = m.yx; g.yaxis[1]  = m.yy; g.yaxis[2]  = m.yz;
            g.radius    = crv.R;
            g.a1        = crv.A1;
            g.a2        = crv.A2;
        }
        else if (crv.Type == VX_CRV_NURB)
        {
            g.degree   = crv.T.degree;
            g.rational = (crv.P.rat != 0);
            g.cp_dim   = crv.P.dim;
            if (crv.T.knots != nullptr && crv.T.num_knots > 0) {
                g.knots.assign(crv.T.knots, crv.T.knots + crv.T.num_knots);
            }
            if (crv.P.coord != nullptr && crv.P.num_cp > 0 && crv.P.dim > 0) {
                g.cps.assign(crv.P.coord,
                             crv.P.coord + crv.P.num_cp * crv.P.dim);
            }
        }
        cvxCurveFree(&crv);

        // Sketch-local -> world. Only when the pick row named a parent
        // whose world matrix is available and non-identity; top-level 3D
        // curves (parent 0, e.g. Sweep3's elbow arc) are world already.
        if (parent > 0)
        {
            szwEntityHandle ph;
            if (ZwEntityIdTransfer(1, &parent, &ph) == ZW_API_NO_ERROR)
            {
                szwMatrix m;
                if (ZwEntityMatrixGet(ph, &m) == ZW_API_NO_ERROR &&
                    !m.identity)
                {
                    auto xpnt = [&](double p[3]) {
                        const double x = p[0], y = p[1], z = p[2];
                        p[0] = m.xt + x*m.xx + y*m.yx + z*m.zx;
                        p[1] = m.yt + x*m.xy + y*m.yy + z*m.zy;
                        p[2] = m.zt + x*m.xz + y*m.yz + z*m.zz;
                    };
                    auto xvec = [&](double v[3]) {
                        const double x = v[0], y = v[1], z = v[2];
                        v[0] = x*m.xx + y*m.yx + z*m.zx;
                        v[1] = x*m.xy + y*m.yy + z*m.zy;
                        v[2] = x*m.xz + y*m.yz + z*m.zz;
                    };
                    if (g.type == 1) {
                        xpnt(g.p0);
                        xpnt(g.p1);
                    } else if (g.type == 2 || g.type == 3) {
                        xpnt(g.origin);
                        xvec(g.xaxis);
                        xvec(g.yaxis);
                    } else if (g.type == 4 && g.cp_dim >= 3) {
                        const int n = static_cast<int>(g.cps.size()) / g.cp_dim;
                        for (int c = 0; c < n; ++c) {
                            double* cp = &g.cps[(size_t)c * g.cp_dim];
                            double  w  = (g.rational && g.cp_dim >= 4)
                                           ? cp[3] : 1.0;
                            if (w == 0.0) { w = 1.0; }
                            double p[3] = { cp[0]/w, cp[1]/w, cp[2]/w };
                            xpnt(p);
                            cp[0] = p[0]*w; cp[1] = p[1]*w; cp[2] = p[2]*w;
                        }
                    }
                }
                ZwEntityHandleFree(&ph);
            }
        }
        out.push_back(std::move(g));
    }
    return out;
}

// Decode one curvilinear entity into a CurveGeo, NO transform: region-
// boundary members come back in the profile sketch's LOCAL frame, which
// is exactly the space profile.curves already live in.
bool CurveGeoFromId(int id, CurveGeo& g)
{
    svxCurve crv = {};
    if (cvxPartInqCurve(id, 0, &crv) != ZW_API_NO_ERROR) {
        return false;
    }
    g.type = static_cast<int>(crv.Type);
    if (crv.Type == VX_CRV_LINE)
    {
        g.p0[0] = crv.P1.x; g.p0[1] = crv.P1.y; g.p0[2] = crv.P1.z;
        g.p1[0] = crv.P2.x; g.p1[1] = crv.P2.y; g.p1[2] = crv.P2.z;
    }
    else if (crv.Type == VX_CRV_ARC || crv.Type == VX_CRV_CIRCLE)
    {
        const svxMatrix& m = crv.Frame;   // axes as ROWS
        g.origin[0] = m.xt; g.origin[1] = m.yt; g.origin[2] = m.zt;
        g.xaxis[0]  = m.xx; g.xaxis[1]  = m.xy; g.xaxis[2]  = m.xz;
        g.yaxis[0]  = m.yx; g.yaxis[1]  = m.yy; g.yaxis[2]  = m.yz;
        g.radius    = crv.R;
        g.a1        = crv.A1;
        g.a2        = crv.A2;
    }
    else if (crv.Type == VX_CRV_NURB)
    {
        g.degree   = crv.T.degree;
        g.rational = (crv.P.rat != 0);
        g.cp_dim   = crv.P.dim;
        if (crv.T.knots != nullptr && crv.T.num_knots > 0) {
            g.knots.assign(crv.T.knots, crv.T.knots + crv.T.num_knots);
        }
        if (crv.P.coord != nullptr && crv.P.num_cp > 0 && crv.P.dim > 0) {
            g.cps.assign(crv.P.coord,
                         crv.P.coord + crv.P.num_cp * crv.P.dim);
        }
    }
    cvxCurveFree(&crv);
    return true;
}

// REGION-pick profile decode. An extrude's "Profile P" (fld 1) is a
// region pick whose entity is a CURVE LIST: the EVALUATED boundary of
// the picked region(s) -- outer loop AND island cutouts that exist in
// NEITHER the drawn-2D nor the reference curve set (R2900's Extrude48
// rect carries its tower / tab cutouts only here; without them the
// replay extruded a solid 13k mm^3 block over a +2.4k truth). Resolve
// fld 1's entity rows, try cvxPartInqCrvList on each, and read every
// member curve (sketch-local coords, same space as profile.curves).
std::vector<CurveGeo> ReadProfileRegionCurves(int idFtr, int* n_list)
{
    std::vector<CurveGeo> out;
    if (n_list != nullptr) { *n_list = 0; }

    std::vector<FieldEntRow> rows;
    std::vector<CurveGeo> direct = ReadFieldCurves(idFtr, 1, &rows);
    for (const FieldEntRow& r : rows)
    {
        int id = (r.idEntity > 0) ? r.idEntity : r.path_last;
        if (id <= 0) { continue; }
        int         cnt   = 0;
        svxEntPick* picks = nullptr;
        if (cvxPartInqCrvList(id, &cnt, &picks) != ZW_API_NO_ERROR ||
            picks == nullptr)
        {
            continue;
        }
        if (n_list != nullptr) { ++(*n_list); }
        for (int k = 0; k < cnt; ++k)
        {
            if (picks[k].idEntity <= 0) { continue; }
            CurveGeo g;
            if (CurveGeoFromId(picks[k].idEntity, g)) {
                out.push_back(std::move(g));
            }
        }
        cvxMemFree(reinterpret_cast<void**>(&picks));
    }
    // The picks may BE the boundary curves directly (no list entity):
    // ReadFieldCurves already decoded them (sketch-local, parent 0 from
    // the tree walk -> no transform); let the reader's closed-loop gate
    // judge usability.
    if (out.empty() && !direct.empty()) {
        out = std::move(direct);
    }
    return out;
}

// Export the active part's final solid to a STEP file. This is the
// universal geometry baseline (correct for ALL features regardless of
// whether we parametrise them) and the "truth" geometry used downstream
// for match anchoring / boolean compensation. Whole-part, all objects.
struct StepExportResult
{
    bool ok = false;
    int  init_rc   = -999;
    int  export_rc = -999;
    long bytes     = -1;     // written file size (post-export stat)
    bool empty     = false;  // rc==0 but no face/solid payload in the file
};

// UTF-8 first, ANSI fallback -- same dual decode OpenWriteBinary uses for
// the JSON; needed because the path handed in here may already be ACP.
std::wstring WidenAnyPath(const std::string& path)
{
    UINT cp  = CP_UTF8;
    int  len = MultiByteToWideChar(cp, MB_ERR_INVALID_CHARS,
                                   path.c_str(), -1, nullptr, 0);
    if (len <= 0)
    {
        cp  = CP_ACP;
        len = MultiByteToWideChar(cp, 0, path.c_str(), -1, nullptr, 0);
    }
    if (len <= 0) {
        return std::wstring();
    }
    std::wstring w(static_cast<size_t>(len), L'\0');
    MultiByteToWideChar(cp, (cp == CP_UTF8) ? MB_ERR_INVALID_CHARS : 0,
                        path.c_str(), -1, &w[0], len);
    while (!w.empty() && w.back() == L'\0') {
        w.pop_back();
    }
    return w;
}

// cvxFileExport reports rc=0 even when the translator dropped every
// entity: pre-ExcludeGeom-fix, blanked bodies produced 1.6KB header-only
// skeletons that read back as null shapes -- and the json pointed the
// reader at them as authored truth. Trust the file, not the rc: stat it
// and require an actual face/solid entity in the DATA section before the
// caller may record it as geometry.
void VerifyStepPayload(const std::string& api_path, StepExportResult& r)
{
    const std::wstring w = WidenAnyPath(api_path);
    if (w.empty()) {
        return;
    }
    FILE* f = _wfopen(w.c_str(), L"rb");
    if (!f) {
        return;
    }
    std::fseek(f, 0, SEEK_END);
    const long sz = std::ftell(f);
    r.bytes = sz;
    // Big files always carry payload; only sniff the small ones.
    if (sz >= 0 && sz < 262144)
    {
        std::string buf(static_cast<size_t>(sz), '\0');
        std::fseek(f, 0, SEEK_SET);
        const size_t got = std::fread(&buf[0], 1, buf.size(), f);
        buf.resize(got);
        const bool has_payload =
            buf.find("ADVANCED_FACE") != std::string::npos ||
            buf.find("MANIFOLD_SOLID_BREP") != std::string::npos;
        if (!has_payload)
        {
            r.empty = true;
            r.ok    = false;
        }
    }
    std::fclose(f);
}

// cvxFileExport, like cvxFileOpen (ToAcp in ZwCaxPlugin.cpp), decodes its
// path argument in the system ANSI code page: handing it UTF-8 makes a
// Chinese DIRECTORY fail with -242 (the misread directory doesn't exist;
// every HW主要案例 truth STEP was silently dropped this way) and a Chinese
// file NAME land as a mojibake-named file even when the directory is
// ASCII (the R2900 .state<K>.step family). Re-encode to ACP when the path
// is valid UTF-8; pass it through unchanged when it already isn't (older
// SDKs hand back GBK) or a character has no ACP representation.
std::string StepPathForApi(const std::string& path)
{
#ifdef _WIN32
    int wlen = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS,
                                   path.c_str(), -1, nullptr, 0);
    if (wlen <= 0) {
        return path;   // not valid UTF-8 -> assume already ANSI
    }
    std::wstring w(static_cast<size_t>(wlen), L'\0');
    MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS,
                        path.c_str(), -1, &w[0], wlen);
    BOOL lost = FALSE;
    int alen = WideCharToMultiByte(CP_ACP, 0, w.c_str(), -1,
                                   nullptr, 0, nullptr, &lost);
    if (alen <= 0 || lost) {
        return path;   // no ACP form -- let ZW3D try the raw bytes
    }
    std::string acp(static_cast<size_t>(alen), '\0');
    WideCharToMultiByte(CP_ACP, 0, w.c_str(), -1, &acp[0], alen,
                        nullptr, nullptr);
    acp.resize(strlen(acp.c_str()));
    return acp;
#else
    return path;
#endif
}

StepExportResult ExportPartStep(const std::string& path)
{
    StepExportResult r;
    svxSTEPData data;
    r.init_rc = static_cast<int>(cvxFileExportInit(VX_EXPORT_TYPE_STEP, 0, &data));
    if (r.init_rc != ZW_API_NO_ERROR) {
        return r;
    }
    data.ExportType = 0;   // 0 = all objects
    const std::string api_path = StepPathForApi(path);
    r.export_rc = static_cast<int>(cvxFileExport(VX_EXPORT_TYPE_STEP, api_path.c_str(), &data));
    r.ok = (r.export_rc == ZW_API_NO_ERROR);
    if (r.ok) {
        VerifyStepPayload(api_path, r);
    }
    return r;
}

// Export ONLY the b-rep shapes a feature produced (svxSTEPData's
// "specified entities" mode). This is the per-feature authored geometry:
// e.g. CdGeomCopy's imported base body, captured on its own so the
// downstream parametric features (extrude/fillet) can be replayed on top
// of it -- as opposed to the whole-part STEP, which bakes everything in.
// Assumes the history is rolled to this feature (the caller's loop does
// that), so the shapes are the feature's own result.
StepExportResult ExportFeatureShapesStep(int idFtr, const std::string& path)
{
    StepExportResult r;

    int  cnt  = 0;
    int* ents = nullptr;
    if (cvxPartInqFtrEnts(idFtr, VX_ENT_SHAPE, &cnt, &ents) != ZW_API_NO_ERROR ||
        ents == nullptr || cnt < 1)
    {
        if (ents != nullptr) {
            cvxMemFree(reinterpret_cast<void**>(&ents));
        }
        return r;   // nothing to export
    }

    svxSTEPData data;
    r.init_rc = static_cast<int>(cvxFileExportInit(VX_EXPORT_TYPE_STEP, 0, &data));
    if (r.init_rc == ZW_API_NO_ERROR)
    {
        data.ExportType = 1;        // specified entities
        data.EntCnt     = cnt;
        data.EntList    = ents;     // the feature's result shape ids
        // Init defaults to VX_EXCLUDE_BLANKED -- which silently drops the
        // very bodies this export exists to capture (surface-modeling
        // parts blank consumed construction skins; 02-ear wrote 16
        // header-only featN.steps this way). We listed the ids explicitly:
        // export them all, visible or not.
        data.ExcludeGeom = 0;
        const std::string api_path = StepPathForApi(path);
        r.export_rc = static_cast<int>(cvxFileExport(VX_EXPORT_TYPE_STEP, api_path.c_str(), &data));
        r.ok = (r.export_rc == ZW_API_NO_ERROR);
        if (r.ok) {
            VerifyStepPayload(api_path, r);
        }
    }

    cvxMemFree(reinterpret_cast<void**>(&ents));
    return r;
}

// Cumulative whole-part state at the CURRENT history stop-line: shape /
// face / edge counts, world AABB, and (optionally) summed mass props.
// Captured right after each feature executes, this is the per-step truth
// the zw_verify bisect replays against -- the converted prefix 1..K must
// reproduce exactly this state. Counts and bbox are id-list inquiries
// (cheap); mass properties recompute integrals per shape, so they hide
// behind `with_mass` (CAX_FEAT_STATE=2). Units are the part's native mm.
struct StateSnap
{
    bool   ok       = false;
    int    n_shape  = 0, n_face = 0, n_edge = 0;
    int    n_blanked = 0;   // of n_shape, how many are blanked (hidden)
    bool   has_box  = false;
    double bmin[3]  = {0, 0, 0}, bmax[3] = {0, 0, 0};
    bool   has_mass = false;
    double area     = 0.0, volume = 0.0;
};

StateSnap CaptureStateSnap(bool with_mass)
{
    StateSnap s;
    int  cnt = 0;
    int* ids = nullptr;
    // NULL file / part = active part. A part whose first features haven't
    // produced a body yet legitimately reports zero shapes.
    if (cvxPartInqShapes(nullptr, nullptr, &cnt, &ids) != ZW_API_NO_ERROR) {
        return s;
    }
    s.ok      = true;
    s.n_shape = cnt;
    for (int i = 0; i < cnt && ids != nullptr; ++i)
    {
        if (cvxEntIsBlanked(ids[i]) == 1) {
            ++s.n_blanked;
        }
        int  n   = 0;
        int* sub = nullptr;
        if (cvxPartInqShapeFaces(ids[i], &n, &sub) == ZW_API_NO_ERROR && sub != nullptr)
        {
            s.n_face += n;
            cvxMemFree(reinterpret_cast<void**>(&sub));
        }
        n   = 0;
        sub = nullptr;
        if (cvxPartInqShapeEdges(ids[i], &n, &sub) == ZW_API_NO_ERROR && sub != nullptr)
        {
            s.n_edge += n;
            cvxMemFree(reinterpret_cast<void**>(&sub));
        }

        svxBndBox bb;
        if (cvxEntBndBox(ids[i], &bb) == ZW_API_NO_ERROR)
        {
            if (!s.has_box)
            {
                s.bmin[0] = bb.X.min; s.bmin[1] = bb.Y.min; s.bmin[2] = bb.Z.min;
                s.bmax[0] = bb.X.max; s.bmax[1] = bb.Y.max; s.bmax[2] = bb.Z.max;
                s.has_box = true;
            }
            else
            {
                s.bmin[0] = std::min(s.bmin[0], bb.X.min);
                s.bmin[1] = std::min(s.bmin[1], bb.Y.min);
                s.bmin[2] = std::min(s.bmin[2], bb.Z.min);
                s.bmax[0] = std::max(s.bmax[0], bb.X.max);
                s.bmax[1] = std::max(s.bmax[1], bb.Y.max);
                s.bmax[2] = std::max(s.bmax[2], bb.Z.max);
            }
        }

        if (with_mass)
        {
            svxMassProp mp;
            // density 0 = the shape's default; Area / Volume don't depend on it.
            if (cvxPartInqShapeMass(ids[i], 0.0, &mp) == ZW_API_NO_ERROR)
            {
                s.area    += mp.Area;
                s.volume  += mp.Volume;
                s.has_mass = true;
            }
        }
    }
    if (ids != nullptr) {
        cvxMemFree(reinterpret_cast<void**>(&ids));
    }
    return s;
}

// Bodies BLANKED in the part's current (final) state. A blanked body is
// a real product of the history -- the converter's replay builds it --
// but ZW3D's visible part and its "all objects" STEP export exclude it,
// so the snapshot must say which bodies those are or the replay emits
// them as phantom extra solids (R2900: the Pattern17 plate+funnel
// composite, ~46k mm^3 = a +19% volume error). Captured at the end of
// the forward sweep, after RollBodyToEnd(). bbox in part-native mm.
struct HiddenBody
{
    double bbox[6] = {0, 0, 0, 0, 0, 0};
    int    n_face  = 0;
};

std::vector<HiddenBody> CaptureHiddenBodies()
{
    std::vector<HiddenBody> out;
    int  cnt = 0;
    int* ids = nullptr;
    if (cvxPartInqShapes(nullptr, nullptr, &cnt, &ids) != ZW_API_NO_ERROR) {
        return out;
    }
    for (int i = 0; i < cnt && ids != nullptr; ++i)
    {
        if (!cvxEntIsBlanked(ids[i])) {
            continue;
        }
        svxBndBox bb;
        if (cvxEntBndBox(ids[i], &bb) != ZW_API_NO_ERROR) {
            continue;
        }
        HiddenBody h;
        h.bbox[0] = bb.X.min; h.bbox[1] = bb.Y.min; h.bbox[2] = bb.Z.min;
        h.bbox[3] = bb.X.max; h.bbox[4] = bb.Y.max; h.bbox[5] = bb.Z.max;
        int  n   = 0;
        int* sub = nullptr;
        if (cvxPartInqShapeFaces(ids[i], &n, &sub) == ZW_API_NO_ERROR &&
            sub != nullptr)
        {
            h.n_face = n;
            cvxMemFree(reinterpret_cast<void**>(&sub));
        }
        out.push_back(h);
    }
    if (ids != nullptr) {
        cvxMemFree(reinterpret_cast<void**>(&ids));
    }
    return out;
}

// Read an external-geometry-copy feature (CdGeomCopy) via its dedicated
// typed inquiry -- its source reference lives there, not in the generic
// field container. Bridges the Vx int feature id to the new-API
// szwEntityHandle with ZwEntityIdTransfer.
void ReadGeomCopy(int idFtr, GeomCopyData& out)
{
    szwEntityHandle handle;
    out.idxfer_rc = static_cast<int>(ZwEntityIdTransfer(1, &idFtr, &handle));
    if (out.idxfer_rc != ZW_API_NO_ERROR) {
        return;
    }

    szwExternalGeometryCopyData data;
    out.get_rc = static_cast<int>(ZwExternalGeometryCopyDataGet(handle, &data));
    if (out.get_rc == ZW_API_NO_ERROR)
    {
        out.ok          = true;
        out.associative = static_cast<int>(data.associativeCopyType);
        if (data.copyType == ZW_GEOMETRY_COPY_FROM_EXTERNAL_FILE)
        {
            out.source       = "external";
            out.file         = data.copyData.externalData.fileName;
            out.root         = data.copyData.externalData.rootName;
            out.entity_count = data.copyData.externalData.entityCount;
        }
        else
        {
            out.source       = "local";
            out.entity_count = data.copyData.localData.entityCount;
        }
        ZwExternalGeometryCopyDataFree(&data);
    }

    ZwEntityHandleFree(&handle);
}

// Entities a feature produced, by type, with geometric signatures. The
// fallback for features whose parameters the API won't give us: read what
// they emitted instead. Curves are the interesting case (extrude profiles
// brought in by a geometry copy); shapes carry a bbox-centre signature.
ResultEnts FeatureResultEnts(int idFtr)
{
    ResultEnts r;

    auto collect = [&](evxEntType type, int& counter, std::vector<EntSig>* sink)
    {
        int  cnt  = 0;
        int* ents = nullptr;
        if (cvxPartInqFtrEnts(idFtr, type, &cnt, &ents) == ZW_API_NO_ERROR && ents != nullptr)
        {
            counter = cnt;
            if (sink != nullptr) {
                for (int i = 0; i < cnt; ++i) {
                    sink->push_back(EntitySig(ents[i]));
                }
            }
            cvxMemFree(reinterpret_cast<void**>(&ents));
        }
    };

    collect(VX_ENT_SHAPE, r.n_shape, &r.shapes);
    collect(VX_ENT_FACE,  r.n_face,  nullptr);   // count only -- faces can be many
    collect(VX_ENT_WIRE,  r.n_curve, &r.curves); // wireframe = profile curves

    return r;
}

// Read one curve (by handle) into `sink`. ZwCurveNURBSDataGet returns
// the geometry in the sketch's LOCAL 2D frame (z==0); the reader uses the
// x/y components plus the plane block to place it in world. Shared by the
// drawn-2D-curve path, the reference-geometry fallback, and the sweep
// path-sketch read.
void ReadCurveByHandle(szwEntityHandle& ch, std::vector<ProfileCurve>& sink)
{
    szwCurve cv;
    if (ZwCurveNURBSDataGet(ch, 0, &cv) != ZW_API_NO_ERROR) {
        return;
    }
    ProfileCurve pc;
    const auto& I = cv.curveInformation;
    switch (cv.type)
    {
        case ZW_CURVE_LINE:
            pc.kind = "line";
            pc.p0[0] = I.line.startPoint.x; pc.p0[1] = I.line.startPoint.y; pc.p0[2] = I.line.startPoint.z;
            pc.p1[0] = I.line.endPoint.x;   pc.p1[1] = I.line.endPoint.y;   pc.p1[2] = I.line.endPoint.z;
            break;
        case ZW_CURVE_ARC:
            pc.kind = "arc";
            pc.radius = I.arc.radius;
            pc.a0 = I.arc.startAngle; pc.a1 = I.arc.endAngle;
            pc.p0[0] = I.arc.startPoint.x; pc.p0[1] = I.arc.startPoint.y; pc.p0[2] = I.arc.startPoint.z;
            pc.p1[0] = I.arc.endPoint.x;   pc.p1[1] = I.arc.endPoint.y;   pc.p1[2] = I.arc.endPoint.z;
            pc.center[0] = I.arc.centerPoint.x; pc.center[1] = I.arc.centerPoint.y; pc.center[2] = I.arc.centerPoint.z;
            break;
        case ZW_CURVE_CIRCLE:
            pc.kind = "circle";
            pc.radius = I.circle.radius;
            pc.center[0] = I.circle.centerPoint.x; pc.center[1] = I.circle.centerPoint.y; pc.center[2] = I.circle.centerPoint.z;
            break;
        case ZW_CURVE_ELLIPSE2:
            pc.kind = "ellipse";
            pc.radius = I.ellipse2.majorAxis;
            pc.center[0] = I.ellipse2.centerPoint.x; pc.center[1] = I.ellipse2.centerPoint.y; pc.center[2] = I.ellipse2.centerPoint.z;
            break;
        default:
            pc.kind = "nurb";
            pc.p0[0] = I.nurb.startPoint.x; pc.p0[1] = I.nurb.startPoint.y; pc.p0[2] = I.nurb.startPoint.z;
            pc.p1[0] = I.nurb.endPoint.x;   pc.p1[1] = I.nurb.endPoint.y;   pc.p1[2] = I.nurb.endPoint.z;
            break;
    }
    sink.push_back(pc);
}

// Read an extrude/revolve feature's PROFILE: the curves of its built-in
// sketch(es). cvxPartFtrInqAuxFtrs gives the feature's auxiliary
// (built-in) sketches; ZwSketch2DCurveListGet enumerates each sketch's
// curves; ZwCurveNURBSDataGet reads each curve's geometry. Points come
// back in world 3D, so the profile is already positioned -- no separate
// plane resolution needed to see its shape.
void ReadProfile(int idFtr, Profile& out)
{
    auto read_one_curve = [&](szwEntityHandle& ch)
    {
        ReadCurveByHandle(ch, out.curves);
    };

    // Read every curve of one sketch (id + handle) into out.curves.
    auto read_sketch_curves = [&](int sketchId, szwEntityHandle& skh)
    {
        // Capture the sketch's world insertion plane once. The curves come back
        // in the sketch's LOCAL 2D frame (z==0), so this matrix carries their
        // true world placement. szwMatrix columns: 1st = x-axis, 3rd = z-axis
        // (normal), 4th = origin/offset.
        if (!out.has_plane)
        {
            szwMatrix m;
            if (ZwEntityMatrixGet(skh, &m) == ZW_API_NO_ERROR)
            {
                out.origin[0] = m.xt; out.origin[1] = m.yt; out.origin[2] = m.zt;
                out.x_dir[0]  = m.xx; out.x_dir[1]  = m.xy; out.x_dir[2]  = m.xz;
                out.normal[0] = m.zx; out.normal[1] = m.zy; out.normal[2] = m.zz;
                out.has_plane = true;
            }
        }

        // 1) Drawn 2D curves -- the normal sketch profile.
        int              cn      = 0;
        szwEntityHandle* curves  = nullptr;
        int              curveRc = ZwSketch2DCurveListGet(&skh, &cn, &curves);
        out.curvelist_rc = curveRc;       // diag: surface the otherwise-swallowed
        out.curvelist_cn = cn;            // rc/count for the "0 curves" extrudes
        if (curveRc == ZW_API_NO_ERROR && curves != nullptr)
        {
            for (int c = 0; c < cn; ++c) {
                read_one_curve(curves[c]);
            }
            ZwEntityHandleListFree(cn, &curves);
        }

        // 2) REFERENCE / projected geometry. Two distinct roles:
        //    - a profile with NO drawn 2D curves at all is built entirely
        //      from reference geometry (R2900_100's Extrude21/26/30/31);
        //    - a profile WITH drawn curves can still use reference curves
        //      as its INNER loops -- R2900's Extrude48 draws only the
        //      outer rect, its tower/hex cutouts are projected edges, and
        //      dropping them made the replay extrude a solid block
        //      (+8.9 cm^3, the part's largest error). So the reference
        //      set is now read ALWAYS and the curves it contributes are
        //      TAGGED is_ref: the reader uses untagged curves as the
        //      authoritative profile and decides per-loop whether a ref
        //      loop is a real cutout.
        if (sketchId > 0)
        {
            const size_t n_drawn = out.curves.size();
            auto read_by_ids = [&](int rcn, int* rids)
            {
                for (int i = 0; i < rcn; ++i)
                {
                    if (rids[i] <= 0) { continue; }
                    szwEntityHandle ch;
                    if (ZwEntityIdTransfer(1, &rids[i], &ch) != ZW_API_NO_ERROR) {
                        continue;
                    }
                    read_one_curve(ch);
                    ZwEntityHandleFree(&ch);
                }
            };

            int  rcn  = 0;
            int* rids = nullptr;
            if (cvxSkInqRefById(sketchId, &rcn, &rids) == ZW_API_NO_ERROR && rids != nullptr)
            {
                out.ref_cn = rcn;
                read_by_ids(rcn, rids);
                cvxMemFree(reinterpret_cast<void**>(&rids));
            }
            // Extended geometry is read UNCONDITIONALLY too (it used to be
            // gated on "ref found nothing"): a region-pick sketch keeps its
            // island boundaries split across BOTH sets -- E48's ref set has
            // only E47's projected outline, the cutout loops live in the
            // extended set.
            {
                int  gcn  = 0;
                int* gids = nullptr;
                if (cvxSkInqGeomXById(sketchId, &gcn, &gids) == ZW_API_NO_ERROR && gids != nullptr)
                {
                    out.geomx_cn = gcn;
                    read_by_ids(gcn, gids);
                    cvxMemFree(reinterpret_cast<void**>(&gids));
                }
            }
            for (size_t i = n_drawn; i < out.curves.size(); ++i) {
                out.curves[i].is_ref = true;
            }
        }
    };

    // 1) Built-in / auxiliary sketches: an extrude that carries its own
    //    inline profile (the dkba case). cvxPartFtrInqAuxFtrs returns those
    //    aux features; each transfers to a sketch handle.
    int  n   = 0;
    int* aux = nullptr;
    if (cvxPartFtrInqAuxFtrs(idFtr, &n, &aux) == ZW_API_NO_ERROR && aux != nullptr && n > 0)
    {
        out.n_sketch = n;
        for (int i = 0; i < n; ++i)
        {
            szwEntityHandle skh;
            if (ZwEntityIdTransfer(1, &aux[i], &skh) != ZW_API_NO_ERROR) {
                continue;
            }
            read_sketch_curves(aux[i], skh);
            ZwEntityHandleFree(&skh);
        }
    }
    if (aux != nullptr) {
        cvxMemFree(reinterpret_cast<void**>(&aux));
    }

    // 2) Fallback: a STANDALONE sketch feature consumed as the profile (the
    //    高压泡沫混合 case -- Sketch1/2/... are their OWN history features).
    //    cvxPartInqFtrList omits them by design ("Non-feature type history
    //    operations will not be output, such as ... sketch"), and they are
    //    not auxiliary, so cvxPartFtrInqAuxFtrs above finds nothing. But the
    //    extrude still REFERENCES that sketch in its data container -- pull
    //    it by entity type (VX_ENT_SKETCH) via cvxPartFtrInqInpEnts and read
    //    its curves the same way. The caller has already rolled history to
    //    idFtr, the in-context state cvxPartFtrInqInpEnts needs.
    if (out.curves.empty())
    {
        int  cnt = 0;
        int* ids = nullptr;
        if (cvxPartFtrInqInpEnts(idFtr, VX_ENT_SKETCH, &cnt, &ids) == ZW_API_NO_ERROR && ids != nullptr)
        {
            int got = 0;
            for (int i = 0; i < cnt; ++i)
            {
                if (ids[i] <= 0) {
                    continue;
                }
                szwEntityHandle skh;
                if (ZwEntityIdTransfer(1, &ids[i], &skh) != ZW_API_NO_ERROR) {
                    continue;
                }
                read_sketch_curves(ids[i], skh);
                ZwEntityHandleFree(&skh);
                ++got;
            }
            out.n_sketch = got;
            // cvxPartFtrInqInpEnts's list is freed with ZwMemoryFree (per its
            // own docs), NOT cvxMemFree like the older cvxPart* inquiries.
            ZwMemoryFree(reinterpret_cast<void**>(&ids));
        }
    }
}

// Read one SKETCH entity (by id) as a Profile: insertion plane + 2D
// curves. A sweep whose path is a whole SKETCH pick (R2900 Sweep2/4:
// fld 2's entity is the path sketch itself, so cvxPartInqCurve fails on
// it) gets its spine geometry from here. Drawn 2D curves only -- a path
// sketch is hand-drawn, not reference geometry.
bool ReadSketchProfileById(int sketchId, Profile& out)
{
    if (sketchId <= 0) {
        return false;
    }
    szwEntityHandle skh;
    if (ZwEntityIdTransfer(1, &sketchId, &skh) != ZW_API_NO_ERROR) {
        return false;
    }
    szwMatrix m;
    if (ZwEntityMatrixGet(skh, &m) == ZW_API_NO_ERROR)
    {
        out.origin[0] = m.xt; out.origin[1] = m.yt; out.origin[2] = m.zt;
        out.x_dir[0]  = m.xx; out.x_dir[1]  = m.xy; out.x_dir[2]  = m.xz;
        out.normal[0] = m.zx; out.normal[1] = m.zy; out.normal[2] = m.zz;
        out.has_plane = true;
    }
    int              cn     = 0;
    szwEntityHandle* curves = nullptr;
    int rc = ZwSketch2DCurveListGet(&skh, &cn, &curves);
    out.curvelist_rc = rc;
    out.curvelist_cn = cn;
    if (rc == ZW_API_NO_ERROR && curves != nullptr)
    {
        for (int c = 0; c < cn; ++c) {
            ReadCurveByHandle(curves[c], out.curves);
        }
        ZwEntityHandleListFree(cn, &curves);
    }
    // Reference / extended-geometry fallback, same as ReadProfile's: a
    // path sketch built from PROJECTED geometry has no drawn 2D curves
    // (R2900 Sweep2's path sketch came back empty here).
    if (out.curves.empty())
    {
        auto read_by_ids = [&](int rcn, int* rids)
        {
            for (int i = 0; i < rcn; ++i)
            {
                if (rids[i] <= 0) { continue; }
                szwEntityHandle ch;
                if (ZwEntityIdTransfer(1, &rids[i], &ch) != ZW_API_NO_ERROR) {
                    continue;
                }
                ReadCurveByHandle(ch, out.curves);
                ZwEntityHandleFree(&ch);
            }
        };
        int  rcn  = 0;
        int* rids = nullptr;
        if (cvxSkInqRefById(sketchId, &rcn, &rids) == ZW_API_NO_ERROR &&
            rids != nullptr)
        {
            out.ref_cn = rcn;
            read_by_ids(rcn, rids);
            cvxMemFree(reinterpret_cast<void**>(&rids));
        }
        if (out.curves.empty())
        {
            int  gcn  = 0;
            int* gids = nullptr;
            if (cvxSkInqGeomXById(sketchId, &gcn, &gids) == ZW_API_NO_ERROR &&
                gids != nullptr)
            {
                out.geomx_cn = gcn;
                read_by_ids(gcn, gids);
                cvxMemFree(reinterpret_cast<void**>(&gids));
            }
        }
    }
    if (!out.curves.empty()) {
        out.n_sketch = 1;
    }
    ZwEntityHandleFree(&skh);
    return !out.curves.empty();
}

// A pattern's DIRECTION comes from a referenced entity, NOT a scalar field
// (the feature's "Direction" field is just the cached pick point). It can be
// a datum axis OR -- as on 高压泡沫混合 -- a picked model EDGE. The first
// guess (axis/datum only) found nothing, so cast a wide net: query the
// pattern's input entities across every plausible direction-carrying type
// and resolve a unit direction for each (ZwDatumAxisDirectionGet for axes;
// end-start of the curve for edges/lines). Every candidate is emitted so the
// real +Y source is visible from one export even if several types match.
struct PatternDirCand
{
    int    type    = 0;      // evxEntType queried
    int    count   = 0;      // entities of that type the pattern references
    bool   has_dir = false;
    double dir[3]  = { 0.0, 0.0, 0.0 };
};

std::vector<PatternDirCand> ReadPatternDirCandidates(int idFtr)
{
    std::vector<PatternDirCand> out;

    const evxEntType kTypes[] = {
        VX_ENT_EDGE, VX_ENT_PART_LINE, VX_ENT_NRB_CRV,
        VX_ENT_INT_CRV, VX_ENT_AXIS, VX_ENT_DATUM,
    };

    for (evxEntType t : kTypes)
    {
        int  cnt = 0;
        int* ids = nullptr;
        if (cvxPartFtrInqInpEnts(idFtr, t, &cnt, &ids) != ZW_API_NO_ERROR || ids == nullptr) {
            continue;
        }

        PatternDirCand c;
        c.type  = static_cast<int>(t);
        c.count = cnt;

        for (int i = 0; i < cnt && !c.has_dir; ++i)
        {
            if (ids[i] <= 0) {
                continue;
            }
            szwEntityHandle h;
            if (ZwEntityIdTransfer(1, &ids[i], &h) != ZW_API_NO_ERROR) {
                continue;
            }

            // Axis / datum: direction is read directly.
            szwPoint ap, ad;
            if (ZwDatumAxisDirectionGet(h, &ap, &ad) == ZW_API_NO_ERROR)
            {
                double L = std::sqrt(ad.x * ad.x + ad.y * ad.y + ad.z * ad.z);
                if (L > 1e-9) {
                    c.dir[0] = ad.x / L; c.dir[1] = ad.y / L; c.dir[2] = ad.z / L;
                    c.has_dir = true;
                }
            }

            // Edge / line / curve: direction = (end - start) of the curve.
            if (!c.has_dir)
            {
                szwCurve cv;
                if (ZwCurveNURBSDataGet(h, 0, &cv) == ZW_API_NO_ERROR)
                {
                    const auto& I = cv.curveInformation;
                    double sx, sy, sz, ex, ey, ez;
                    if (cv.type == ZW_CURVE_LINE) {
                        sx = I.line.startPoint.x; sy = I.line.startPoint.y; sz = I.line.startPoint.z;
                        ex = I.line.endPoint.x;   ey = I.line.endPoint.y;   ez = I.line.endPoint.z;
                    } else {
                        sx = I.nurb.startPoint.x; sy = I.nurb.startPoint.y; sz = I.nurb.startPoint.z;
                        ex = I.nurb.endPoint.x;   ey = I.nurb.endPoint.y;   ez = I.nurb.endPoint.z;
                    }
                    double dx = ex - sx, dy = ey - sy, dz = ez - sz;
                    double L = std::sqrt(dx * dx + dy * dy + dz * dz);
                    if (L > 1e-9) {
                        c.dir[0] = dx / L; c.dir[1] = dy / L; c.dir[2] = dz / L;
                        c.has_dir = true;
                    }
                }
            }

            ZwEntityHandleFree(&h);
        }

        out.push_back(c);
        ZwMemoryFree(reinterpret_cast<void**>(&ids));
    }
    return out;
}

} // namespace zwapi

// ============================================================
// Export logic. Pure: neutral structs -> JSON. No SDK types. Every
// token is sourced from the shared schema header.
// ============================================================

namespace
{

const char* GeoKindToken(GeoKind k)
{
    switch (k)
    {
        case GeoKind::Point:   return sc::geo::Point;
        case GeoKind::Line:    return sc::geo::Line;
        case GeoKind::Arc:     return sc::geo::Arc;
        case GeoKind::Circle:  return sc::geo::Circle;
        case GeoKind::Ellipse: return sc::geo::Ellipse;
        case GeoKind::Spline:  return sc::geo::Spline;
        default:               return sc::geo::Unknown;
    }
}

const char* ConsKindToken(ConsKind k)
{
    switch (k)
    {
        case ConsKind::Distance:       return sc::cons::Distance;
        case ConsKind::DistanceX:      return sc::cons::DistanceX;
        case ConsKind::DistanceY:      return sc::cons::DistanceY;
        case ConsKind::Angle:          return sc::cons::Angle;
        case ConsKind::Parallel:       return sc::cons::Parallel;
        case ConsKind::Perpendicular:  return sc::cons::Perpendicular;
        case ConsKind::Coincident:     return sc::cons::Coincident;
        case ConsKind::Horizontal:     return sc::cons::Horizontal;
        case ConsKind::Vertical:       return sc::cons::Vertical;
        case ConsKind::Equal:          return sc::cons::Equal;
        case ConsKind::Tangent:        return sc::cons::Tangent;
        case ConsKind::Concentric:     return sc::cons::Concentric;
        case ConsKind::Symmetric:      return sc::cons::Symmetric;
        case ConsKind::Colinear:       return sc::cons::Colinear;
        case ConsKind::Fix:            return sc::cons::Fix;
        case ConsKind::CircleRadius:   return sc::cons::CircleRadius;
        case ConsKind::CircleDiameter: return sc::cons::CircleDiameter;
        case ConsKind::ArcRadius:      return sc::cons::ArcRadius;
        case ConsKind::ArcDiameter:    return sc::cons::ArcDiameter;
        default:                       return sc::cons::None;
    }
}

const char* EndCondToken(EndCond e)
{
    switch (e)
    {
        case EndCond::Blind:             return sc::end_cond::Blind;
        case EndCond::ThroughAll:        return sc::end_cond::ThroughAll;
        case EndCond::UpToSurface:       return sc::end_cond::UpToSurface;
        case EndCond::UpToVertex:        return sc::end_cond::UpToVertex;
        case EndCond::MidPlane:          return sc::end_cond::MidPlane;
        case EndCond::OffsetFromSurface: return sc::end_cond::OffsetFromSurface;
        case EndCond::UpToFirst:         return sc::end_cond::UpToFirst;
        default:                         return sc::end_cond::Blind;
    }
}

const char* PointPosToken(PointPos p)
{
    switch (p)
    {
        case PointPos::Start:  return sc::pos::Start;
        case PointPos::Mid:    return sc::pos::Mid;
        case PointPos::End:    return sc::pos::End;
        case PointPos::Center: return sc::pos::Center;
        default:               return sc::pos::None;
    }
}

json Vec3(const double v[3])
{
    return json::array({ v[0], v[1], v[2] });
}

json RefToJson(const ConsRef& r)
{
    json j;
    j["geo"] = r.geo;
    j["pos"] = PointPosToken(r.pos);
    return j;
}

json GeomToJson(const GeomData& g)
{
    json j;
    j["geo_id"]       = g.geo_id;
    j["type"]         = GeoKindToken(g.kind);
    j["construction"] = g.construction;

    const auto& p = g.params;
    switch (g.kind)
    {
        case GeoKind::Point:
        {
            j["pt"] = json::array({ p[0], p[1] });
            break;
        }
        case GeoKind::Line:
        {
            j["p0"] = json::array({ p[0], p[1] });
            j["p1"] = json::array({ p[2], p[3] });
            break;
        }
        case GeoKind::Arc:
        {
            j["center"]    = json::array({ p[0], p[1] });
            j["radius"]    = p[2];
            j["start_ang"] = p[3];
            j["end_ang"]   = p[4];
            break;
        }
        case GeoKind::Circle:
        {
            j["center"] = json::array({ p[0], p[1] });
            j["radius"] = p[2];
            break;
        }
        case GeoKind::Ellipse:
        {
            j["center"]  = json::array({ p[0], p[1] });
            j["major_r"] = p[2];
            j["minor_r"] = p[3];
            break;
        }
        case GeoKind::Spline:
        {
            json ctrl = json::array();
            for (size_t i = 0; i + 1 < p.size(); i += 2) {
                ctrl.push_back(json::array({ p[i], p[i + 1] }));
            }
            j["ctrl"] = std::move(ctrl);
            break;
        }
        default:
            break;
    }
    return j;
}

json ConsToJson(const ConsData& c)
{
    json j;
    j["type"]    = ConsKindToken(c.kind);
    j["value"]   = c.value;
    j["driving"] = c.driving;
    j["a"]       = RefToJson(c.a);
    j["b"]       = RefToJson(c.b);
    return j;
}

json SketchToJson(const SketchData& sk)
{
    json plane;
    plane["origin"] = Vec3(sk.origin);
    plane["x_dir"]  = Vec3(sk.x_dir);
    plane["normal"] = Vec3(sk.normal);

    json geoms = json::array();
    for (const auto& g : sk.geoms) {
        geoms.push_back(GeomToJson(g));
    }

    json cons = json::array();
    for (const auto& c : sk.cons) {
        cons.push_back(ConsToJson(c));
    }

    json j;
    j["plane"]       = std::move(plane);
    j["geoms"]       = std::move(geoms);
    j["constraints"] = std::move(cons);
    return j;
}

// Build the extrude feature JSON. prev_solid_id is the running body
// tip: 0 means "fresh body / no upstream" (Replayer treats id 0 as a
// new body root), otherwise the previous solid-producing feature id.
json ExtrudeToJson(const ExtrudeData& ex, uint32_t prev_solid_id)
{
    json inputs = json::array();
    json base_in;
    base_in["id"]   = prev_solid_id;
    base_in["role"] = sc::role::Base;
    inputs.push_back(std::move(base_in));

    json j;
    j["subkind"]        = ex.boss ? sc::subkind::Boss : sc::subkind::Cut;
    j["profile_id"]     = ex.profile_id;
    j["inputs"]         = std::move(inputs);
    j["end_cond"]       = EndCondToken(ex.end_cond);
    j["end_cond2"]      = EndCondToken(ex.end_cond2);
    j["depth"]          = ex.depth;
    j["depth2"]         = ex.depth2;
    j["flip"]           = ex.flip;
    j["thin"]           = ex.thin;
    j["thin_thickness"] = ex.thin_thickness;
    return j;
}

json EntSigToJson(const EntSig& e)
{
    json j;
    j["kind"]   = e.kind;
    j["anchor"] = json::array({ e.anchor[0], e.anchor[1], e.anchor[2] });
    if (e.has_normal) {
        j["normal"] = json::array({ e.normal[0], e.normal[1], e.normal[2] });
    }
    if (e.has_num) {
        j["num"] = e.num;
    }
    if (e.has_feat) {
        j["feat"] = e.feat;   // JSON ordinal id of the OWNING feature
    }
    if (e.blanked) {
        j["blanked"] = true;  // hidden construction geometry in ZW3D
    }
    return j;
}

// One field -> JSON, keyed by stable fld id, carrying whatever the field
// holds (scalar / point / text / entity signatures).
json FieldsToJson(const std::vector<FieldDump>& fields)
{
    json arr = json::array();
    for (const auto& d : fields)
    {
        json j;
        j["id"]   = d.id;
        if (!d.name.empty()) {
            j["name"] = d.name;
        }
        j["type"] = d.type;
        if (d.has_num) {
            j["value"] = d.num;
        }
        if (d.has_pt) {
            j["pt"] = json::array({ d.pt[0], d.pt[1], d.pt[2] });
        }
        // Full instance list of a multi-point field (PNT_TO_PNT "To points").
        // Emitted only when it adds information beyond the single "pt".
        if (d.pts.size() > 1) {
            json pa = json::array();
            for (const auto& p : d.pts) {
                pa.push_back(json::array({ p[0], p[1], p[2] }));
            }
            j["pts"] = std::move(pa);
        }
        if (d.has_dir) {
            j["dir"] = json::array({ d.dir[0], d.dir[1], d.dir[2] });
        }
        if (d.has_text) {
            j["text"] = d.text;
        }
        if (!d.ents.empty())
        {
            json es = json::array();
            for (const auto& e : d.ents) {
                es.push_back(EntSigToJson(e));
            }
            j["ents"] = std::move(es);
        }
        if (d.list_count >= 0) {
            j["list_count"] = d.list_count;   // VX_FLD_DATA tree item count
        }
        arr.push_back(std::move(j));
    }
    return arr;
}

json ResultEntsToJson(const ResultEnts& r)
{
    json j;
    j["n_shape"] = r.n_shape;
    j["n_face"]  = r.n_face;
    j["n_curve"] = r.n_curve;
    if (!r.shapes.empty())
    {
        json a = json::array();
        for (const auto& e : r.shapes) { a.push_back(EntSigToJson(e)); }
        j["shapes"] = std::move(a);
    }
    if (!r.curves.empty())
    {
        json a = json::array();
        for (const auto& e : r.curves) { a.push_back(EntSigToJson(e)); }
        j["curves"] = std::move(a);
    }
    return j;
}

json ProfileToJson(const Profile& pr)
{
    json j;
    j["n_sketch"] = pr.n_sketch;
    if (pr.has_plane)
    {
        json pl;
        pl["origin"] = json::array({ pr.origin[0], pr.origin[1], pr.origin[2] });
        pl["x_dir"]  = json::array({ pr.x_dir[0],  pr.x_dir[1],  pr.x_dir[2]  });
        pl["normal"] = json::array({ pr.normal[0], pr.normal[1], pr.normal[2] });
        j["plane"] = std::move(pl);
    }
    json arr = json::array();
    for (const auto& c : pr.curves)
    {
        json jc;
        jc["kind"] = c.kind;
        if (c.kind == "line" || c.kind == "arc" || c.kind == "nurb")
        {
            jc["p0"] = json::array({ c.p0[0], c.p0[1], c.p0[2] });
            jc["p1"] = json::array({ c.p1[0], c.p1[1], c.p1[2] });
        }
        if (c.kind == "arc" || c.kind == "circle" || c.kind == "ellipse")
        {
            jc["center"] = json::array({ c.center[0], c.center[1], c.center[2] });
            jc["radius"] = c.radius;
        }
        if (c.kind == "arc")
        {
            jc["a0"] = c.a0;
            jc["a1"] = c.a1;
        }
        if (c.is_ref)
        {
            jc["ref"] = true;
        }
        arr.push_back(std::move(jc));
    }
    j["curves"] = std::move(arr);
    return j;
}

// World-mm 3D curve (a sweep path segment). Distinct from ProfileToJson's
// sketch-local 2D encoding: these are absolute-space curves with their own
// frame, consumed by the reader to synthesize a spine.
json CurveGeoToJson(const zwapi::CurveGeo& g)
{
    json j;
    switch (g.type)
    {
    case 1:   // VX_CRV_LINE
        j["kind"] = "line";
        j["p0"]   = json::array({ g.p0[0], g.p0[1], g.p0[2] });
        j["p1"]   = json::array({ g.p1[0], g.p1[1], g.p1[2] });
        break;
    case 2:   // VX_CRV_ARC
    case 3:   // VX_CRV_CIRCLE
        j["kind"]   = (g.type == 2) ? "arc" : "circle";
        j["center"] = json::array({ g.origin[0], g.origin[1], g.origin[2] });
        j["x_dir"]  = json::array({ g.xaxis[0], g.xaxis[1], g.xaxis[2] });
        j["y_dir"]  = json::array({ g.yaxis[0], g.yaxis[1], g.yaxis[2] });
        j["radius"] = g.radius;
        if (g.type == 2) {
            j["a0"] = g.a1;
            j["a1"] = g.a2;
        }
        break;
    case 4:   // VX_CRV_NURB
        j["kind"]     = "nurbs";
        j["degree"]   = g.degree;
        j["rational"] = g.rational;
        j["cp_dim"]   = g.cp_dim;
        j["knots"]    = g.knots;
        j["cps"]      = g.cps;
        break;
    default:
        j["kind"] = "unknown";
        break;
    }
    return j;
}

json GeomCopyToJson(const GeomCopyData& gc)
{
    json j;
    j["source"]       = gc.source;
    j["entity_count"] = gc.entity_count;
    j["associative"]  = gc.associative;
    if (!gc.file.empty()) {
        j["file"] = gc.file;
    }
    if (!gc.root.empty()) {
        j["root"] = gc.root;
    }
    return j;
}

// Trailing path component of p (handles both / and \ separators).
std::string BaseName(const std::string& p)
{
    const auto s = p.find_last_of("/\\");
    return (s == std::string::npos) ? p : p.substr(s + 1);
}

// Sibling path for a feature's own STEP: "<out without .json>.feat<id>.step".
std::string FeatStepPath(const std::string& out_path, uint32_t id)
{
    std::string p = out_path;
    const std::string js = ".json";
    if (p.size() >= js.size() &&
        p.compare(p.size() - js.size(), js.size(), js) == 0)
    {
        p = p.substr(0, p.size() - js.size());
    }
    return p + ".feat" + std::to_string(id) + ".step";
}

// Sibling path for a CUMULATIVE state STEP (the whole part right after
// feature <id>): "<out without .json>.state<id>.step". Distinct from
// FeatStepPath, which exports only the shapes one feature created.
std::string StateStepPath(const std::string& out_path, uint32_t id)
{
    std::string p = out_path;
    const std::string js = ".json";
    if (p.size() >= js.size() &&
        p.compare(p.size() - js.size(), js.size(), js) == 0)
    {
        p = p.substr(0, p.size() - js.size());
    }
    return p + ".state" + std::to_string(id) + ".step";
}

json StateSnapToJson(const zwapi::StateSnap& s)
{
    json j;
    j["n_shape"] = s.n_shape;
    j["n_face"]  = s.n_face;
    j["n_edge"]  = s.n_edge;
    if (s.n_blanked > 0) {
        j["n_blanked"] = s.n_blanked;   // hidden construction bodies, absent
    }                                   // from any STEP the translator writes
    if (s.has_box)
    {
        j["bbox"] = json::array({ s.bmin[0], s.bmin[1], s.bmin[2],
                                  s.bmax[0], s.bmax[1], s.bmax[2] });
    }
    if (s.has_mass)
    {
        j["area"]   = s.area;     // mm^2
        j["volume"] = s.volume;   // mm^3
    }
    return j;
}

// Per-feature state capture config, from the environment (read once per
// export -- ZW3D inherits the env of whoever launched it, e.g. drive.ps1):
//   CAX_FEAT_STATE       0 = off, 1 = counts + bbox (the default),
//                        2 = also mass properties (area / volume; heavier:
//                        ZW3D re-integrates each shape at every feature)
//   CAX_FEAT_STATE_STEP  "" (default, off), "all", or a comma list of
//                        1-based feature ordinals -- exports the CUMULATIVE
//                        part body as .state<K>.step at those stops, for
//                        face-level diffing of a bisected feature.
struct StateConfig
{
    int               mode = 1;
    bool              step_all = false;
    std::vector<long> step_ids;

    static StateConfig FromEnv()
    {
        StateConfig c;
        if (const char* m = std::getenv("CAX_FEAT_STATE"))
        {
            if (*m != '\0') { c.mode = std::atoi(m); }
        }
        if (const char* s = std::getenv("CAX_FEAT_STATE_STEP"))
        {
            std::string v(s);
            if (v == "all" || v == "ALL")
            {
                c.step_all = true;
            }
            else
            {
                std::string tok;
                for (char ch : v + ",")
                {
                    if (ch == ',')
                    {
                        if (!tok.empty()) { c.step_ids.push_back(std::atol(tok.c_str())); }
                        tok.clear();
                    }
                    else { tok += ch; }
                }
            }
        }
        return c;
    }

    bool WantStep(uint32_t id) const
    {
        if (step_all) { return true; }
        for (long v : step_ids)
        {
            if (v == static_cast<long>(id)) { return true; }
        }
        return false;
    }
};

// Open a file for binary writing through a Unicode-safe path. ZW3D's C API
// returns paths in UTF-8; the CRT's narrow std::fopen opens them in the
// ANSI code page, so a Chinese (or any non-ASCII) part name fails to open
// even though the sibling STEP -- written by ZW3D's own Unicode-aware
// cvxFileExport -- lands fine. Convert UTF-8 -> UTF-16 and use _wfopen so
// the JSON write matches cvxFileExport's behaviour. Falls back to the
// system ANSI code page if the bytes aren't valid UTF-8 (an older SDK that
// hands back GBK). On non-Windows, plain fopen already handles UTF-8.
FILE* OpenWriteBinary(const std::string& path)
{
#ifdef _WIN32
    UINT cp  = CP_UTF8;
    int  len = MultiByteToWideChar(cp, MB_ERR_INVALID_CHARS,
                                   path.c_str(), -1, nullptr, 0);
    if (len == 0)
    {
        cp  = CP_ACP;   // not valid UTF-8 -> treat as system ANSI (e.g. GBK)
        len = MultiByteToWideChar(cp, 0, path.c_str(), -1, nullptr, 0);
    }
    if (len == 0)
    {
        return nullptr;
    }
    std::wstring wpath(static_cast<size_t>(len), L'\0');
    MultiByteToWideChar(cp, (cp == CP_UTF8) ? MB_ERR_INVALID_CHARS : 0,
                        path.c_str(), -1, &wpath[0], len);
    return _wfopen(wpath.c_str(), L"wb");
#else
    return std::fopen(path.c_str(), "wb");
#endif
}

} // namespace

// ============================================================
// Public entry. The registered ZW3D command calls this with the
// output path (typically "<part>.cax.json" next to the source .Z3).
// ============================================================

bool ExportActivePartToCax(const std::string& out_path, std::string& err)
{
    json doc;
    doc["schema_version"] = sc::kSchemaVersion;
    doc["source"]         = sc::kSourceZw3d;
    doc["length_unit"]    = zwapi::LengthUnit();

    json features = json::array();

    std::vector<int> feats = zwapi::HistoryFeatures();
    if (feats.empty())
    {
        err = "ExportActivePartToCax: no features (no active part?)";
        return false;
    }

    // Build the ZW-feature-id -> JSON-ordinal map BEFORE the loop so every
    // EntitySig (incl. a pattern's Base picks) can stamp its owning feature's
    // JSON id (cvxPartInqEntFtr returns a ZW id; the JSON id is the ordinal).
    zwapi::SetJsonIdMap(feats);

    // Per-feature cumulative state capture (CAX_FEAT_STATE / _STEP env).
    const StateConfig state_cfg = StateConfig::FromEnv();

    // CAX_BAKE_CUMULATIVE=1: for opaque features that modify the body IN
    // PLACE but report no own result shapes (n_shape==0) -- sheet-metal
    // flanges/tabs/punches (CdSmd*/Smd*), the API-opaque ___凸包 bosses --
    // bake the WHOLE body at the feature's AFTER state as its geometry and
    // flag it "baked_cumulative". The reader loads it as the new running
    // body (replace); downstream reconstructable features (cuts, patterns)
    // replay on top. Gives exact geometry for feature classes cax can't
    // model parametrically. Off by default (heavier: a full-body STEP per
    // such feature); enable per-part for sheet-metal / opaque-feature parts.
    const bool bake_cumulative = [] {
        const char* e = std::getenv("CAX_BAKE_CUMULATIVE");
        return e && e[0] && e[0] != '0';
    }();

    // ONE forward sweep through history. Roll the body to the BEGIN once; from
    // here every per-feature roll only plays FORWARD (ZwHistoryReplay is relative
    // to the stop-line and we visit features in history order), so the body is
    // rebuilt once end-to-end -- no per-feature roll-back-and-replay (no O(N^2)).
    zwapi::RollBodyToBegin();

    // History is linear per part, so the running solid tip is just the
    // previous solid-producing feature's id. Sketches don't advance it.
    uint32_t prev_solid_id = 0;

    for (size_t i = 0; i < feats.size(); ++i)
    {
        int fid = feats[i];

        // INPUT STATE: play the body forward to just BEFORE this feature, where
        // its picked faces/edges are still live (a dressup hasn't consumed its
        // edge yet). Every INPUT-side read below -- params, entity picks,
        // profile, pattern direction -- happens in this state, so its geometric
        // anchors match the state the feature actually operated in (the same
        // state the reader rebuilds and matches against). OUTPUT-side reads later
        // roll forward to the AFTER state. No cvxPartHistScrollTo here: the body
        // roll itself sets the read context (mirrors the proven dressup recipe --
        // ZwHistoryReplay then read, no scroll).
        bool at_before = zwapi::RollBodyBefore(fid);

        // Set by any branch that already played the body to this feature's
        // AFTER state, so the state capture at the loop tail doesn't roll twice.
        bool rolled_after = false;

        FeatNode node;
        node.id   = static_cast<uint32_t>(i + 1);
        node.name = zwapi::FeatureName(fid);

        std::string type = zwapi::FeatureType(fid);
        node.kind        = zwapi::MapFeatKind(type);

        json jf;
        jf["id"]   = node.id;
        jf["name"] = node.name;

        if (node.kind == FeatKind::Sketch)
        {
            zwapi::ReadSketch(fid, node.sketch);
            jf["kind"] = sc::kind::Sketch;
            jf.update(SketchToJson(node.sketch));
        }
        else if (node.kind == FeatKind::Extrude)
        {
            zwapi::ReadExtrude(fid, node.extrude);
            jf["kind"] = sc::kind::Extrude;
            jf.update(ExtrudeToJson(node.extrude, prev_solid_id));
            prev_solid_id = node.id;
        }
        else if (node.kind == FeatKind::Box)
        {
            // OUTPUT STATE: a Box is a root primitive -- its result shape (read
            // by ReadShapeMinCorner) only exists once the feature executes.
            rolled_after = zwapi::RollBodyAfter(fid);
            zwapi::ReadBox(fid, node.box);
            jf["kind"]   = sc::kind::Box;
            jf["length"] = node.box.length;
            jf["width"]  = node.box.width;
            jf["height"] = node.box.height;
            if (node.box.has_place)
            {
                // World min-corner: where the box's (0,0,0) corner sits.
                jf["placement"] = json::array({
                    node.box.place[0], node.box.place[1], node.box.place[2] });
            }
            prev_solid_id = node.id;
        }
        else
        {
            // Unrecognised: keep the feature visible with its true ZW3D
            // template token, plus a FULL dump of its data container --
            // every field by type, including points, text, and entity
            // references with geometric signatures. That exposes what the
            // feature actually operates on (which faces/edges, where), not
            // just secondary scalars.
            jf["kind"]    = sc::kind::Opaque;
            jf["zw_type"] = type;

            // Diagnostic: report the data-access return codes so an empty
            // export tells us WHERE it failed (scroll / data re-eval /
            // field count) instead of leaving us to guess. ZW_API_NO_ERROR
            // is 0; -1 from data_rc means "feature data undefined".
            zwapi::FtrProbe probe = zwapi::ProbeFeature(fid);
            json diag;
            diag["roll_before"] = at_before;   // body played to this feat's BEFORE state
            diag["data_rc"]     = probe.data_rc;
            diag["field_count"] = probe.field_count;

            std::vector<FieldDump> fields = zwapi::DumpFields(fid);

            // Picked edges in their ORIGINAL geometry. The body is already at
            // this feature's BEFORE state, so a dressup's edge is still the
            // original (not yet consumed into a chamfer boundary) -- the nested-
            // list ents in `fields` above were read in this same state, so they
            // should already be original. We additionally read the feature's
            // input edges straight off its input set (the proven cvxPartFtrInq-
            // InpEnts path) and, when the counts line up, overwrite the list-
            // field ents' anchors with them, so the reader / TopoRefResolver
            // matches the true edge instead of any post-dressup boundary (which
            // on test.Z3PRT sat at the face centre, tied across 4 edges).
            std::vector<EntSig> in_edges = zwapi::InputEdgesAtCurrentState(fid);
            if (!in_edges.empty())
            {
                size_t n_list = 0;
                for (const auto& fd : fields) {
                    if (fd.list_count >= 0) { n_list += fd.ents.size(); }
                }
                if (n_list == in_edges.size())
                {
                    size_t k = 0;
                    for (auto& fd : fields) {
                        if (fd.list_count < 0) { continue; }
                        for (auto& e : fd.ents) {
                            e.anchor[0] = in_edges[k].anchor[0];
                            e.anchor[1] = in_edges[k].anchor[1];
                            e.anchor[2] = in_edges[k].anchor[2];
                            ++k;
                        }
                    }
                }
            }

            if (!fields.empty()) {
                jf["fields"] = FieldsToJson(fields);
            }

            // CdGeomCopy keeps its source reference outside the generic
            // field container -- pull it via the dedicated typed inquiry.
            if (type == "CdGeomCopy")
            {
                GeomCopyData gc;
                zwapi::ReadGeomCopy(fid, gc);
                if (gc.ok) {
                    jf["geom_copy"] = GeomCopyToJson(gc);
                }
                diag["idxfer_rc"]  = gc.idxfer_rc;
                diag["geomcopy_rc"] = gc.get_rc;
            }

            // Feature-level input edges (original geometry, read in this
            // feature's BEFORE state) -- also emitted standalone as a fallback
            // for the reader when it can't use the list-field ents.
            if (!in_edges.empty()) {
                json ea = json::array();
                for (const auto& e : in_edges) {
                    ea.push_back(EntSigToJson(e));
                }
                jf["input_edges"] = std::move(ea);
            }
            diag["n_input_edge"] = static_cast<int>(in_edges.size());

            // Profile: an extrude/revolve's built-in sketch curves. This is
            // the geometry its scalar params (End E, draft, ...) act on,
            // and what a parametric reconstruction extrudes.
            Profile pr;
            zwapi::ReadProfile(fid, pr);
            if (!pr.curves.empty()) {
                jf["profile"] = ProfileToJson(pr);
            }
            diag["n_profile_sketch"] = pr.n_sketch;
            diag["n_profile_curve"]  = static_cast<int>(pr.curves.size());
            // When a sketch was found but yielded no curves (the opaque-extrude
            // case), these say whether ZwSketch2DCurveListGet errored or simply
            // reported zero -- the discriminator for the eventual fix.
            diag["profile_curvelist_rc"] = pr.curvelist_rc;
            diag["profile_curvelist_cn"] = pr.curvelist_cn;
            // Fallback diagnostics: how many curves the reference- / extended-
            // geometry fallback (cvxSkInqRefById / cvxSkInqGeomXById) found
            // (-1 = the fallback did not run). For the reference-profile
            // extrudes these reveal which set actually supplied the profile.
            diag["profile_ref_cn"]   = pr.ref_cn;
            diag["profile_geomx_cn"] = pr.geomx_cn;

            // Region-pick profile (FtAllExt): fld 1's entity can be a curve
            // list = the EVALUATED region boundary, outer loop plus island
            // cutouts that no sketch curve set carries. Emitted separately
            // as "profile_region" (sketch-local coords, same space as
            // profile.curves); the reader prefers it when it forms closed
            // loops.
            if (type == "FtAllExt")
            {
                int n_list = 0;
                std::vector<zwapi::CurveGeo> region =
                    zwapi::ReadProfileRegionCurves(fid, &n_list);
                if (!region.empty())
                {
                    json ra = json::array();
                    for (const auto& g : region) {
                        ra.push_back(CurveGeoToJson(g));
                    }
                    jf["profile_region"] = std::move(ra);
                }
                diag["region_lists"]  = n_list;
                diag["region_curves"] = static_cast<int>(region.size());
            }

            // Sweep (FtAllSwp1): the path is a point-on-curve pick in fld 2
            // whose generic dump keeps only the pick POINT -- without the
            // curve geometry the reader cannot rebuild the spine and the
            // whole sweep is dropped (R2900 lost 5 sweeps plus every fillet
            // that blends a pin into the swept surface). Resolve fld 2's
            // entity rows to world-mm curve geometry here, still in the
            // BEFORE state where the path curve is live.
            if (type == "FtAllSwp1")
            {
                std::vector<zwapi::FieldEntRow> rows;
                std::vector<zwapi::CurveGeo>    path =
                    zwapi::ReadFieldCurves(fid, 2, &rows);
                if (!path.empty())
                {
                    json pa = json::array();
                    for (const auto& g : path) {
                        pa.push_back(CurveGeoToJson(g));
                    }
                    jf["path"] = std::move(pa);
                }
                else if (!rows.empty())
                {
                    // fld 2 references a whole SKETCH (cvxPartInqCurve
                    // fails on it): the path is that sketch's curve
                    // chain -- R2900's Sweep2 (one line) and Sweep4
                    // (4 lines + 2 arcs, an S-bend).
                    int sk_id = (rows[0].idEntity > 0) ? rows[0].idEntity
                                                       : rows[0].path_last;
                    Profile pp;
                    if (zwapi::ReadSketchProfileById(sk_id, pp)) {
                        jf["path_sketch"] = ProfileToJson(pp);
                    }
                }
                diag["n_path_curve"] = static_cast<int>(path.size());
                json ra = json::array();
                for (const auto& r : rows)
                {
                    json rj;
                    rj["ent"]    = r.idEntity;
                    rj["parent"] = r.idParent;
                    rj["pcnt"]   = r.path_count;
                    rj["plast"]  = r.path_last;
                    rj["on_crv"] = r.isPntOnCrv;
                    ra.push_back(std::move(rj));
                }
                diag["path_rows"] = std::move(ra);

                // Profile P1 (fld 1): the generic ReadProfile below merges
                // EVERY input sketch -- on Sweep4 that contaminated the
                // profile with the path sketch's 6 chain curves. When
                // fld 1's pick resolves to a sketch of its own, emit THAT
                // sketch alone as the profile (overrides the merged read).
                {
                    std::vector<zwapi::FieldEntRow> prows;
                    zwapi::ReadFieldCurves(fid, 1, &prows);
                    if (!prows.empty())
                    {
                        int pk_id = (prows[0].idEntity > 0)
                                        ? prows[0].idEntity
                                        : prows[0].path_last;
                        Profile pf;
                        if (zwapi::ReadSketchProfileById(pk_id, pf)) {
                            jf["profile"] = ProfileToJson(pf);
                            diag["profile_from_fld1"] = true;
                        }
                    }
                }
            }

            // Pattern (FtPtnFtr): count / spacing / patterned-target are
            // already in the field dump (fld 3 / 4 / 1); the one missing
            // piece is the DIRECTION, which lives in a referenced axis, not
            // a field. Resolve it here. (Reader uses this + the fields to
            // build a LinearPattern.) Diagnostics ride in _diag so a missed
            // direction-source is visible from the export.
            if (type == "FtPtnFtr")
            {
                std::vector<zwapi::PatternDirCand> cands =
                    zwapi::ReadPatternDirCandidates(fid);

                json cand_arr = json::array();
                bool picked = false;
                for (const auto& c : cands)
                {
                    json cj;
                    cj["type"]  = c.type;
                    cj["count"] = c.count;
                    if (c.has_dir)
                    {
                        cj["dir"] = json::array({ c.dir[0], c.dir[1], c.dir[2] });
                        if (!picked)   // first resolved direction -> pattern.dir
                        {
                            json pj;
                            pj["dir"] = json::array({ c.dir[0], c.dir[1], c.dir[2] });
                            jf["pattern"] = std::move(pj);
                            picked = true;
                        }
                    }
                    cand_arr.push_back(std::move(cj));
                }
                diag["pat_dir_ok"]     = picked;
                diag["pat_candidates"] = std::move(cand_arr);
            }

            // OUTPUT STATE: play the body forward to just AFTER this feature, so
            // its result shapes/faces exist. The input-side reads above ran in
            // the BEFORE state; this is the single forward step that actually
            // builds this feature. Everything below reads its OWN output.
            bool at_after = zwapi::RollBodyAfter(fid);
            diag["roll_after"] = at_after;
            rolled_after       = at_after;

            // PARAM FALLBACK (AFTER state): some features' data container is not
            // readable at the BEFORE state -- R2900_100's Extrude21 returns
            // cvxPartInqFtrData data_rc=-10000 / field_count=-1 there, so the
            // BEFORE dump above came back EMPTY and its Start S / End E / draft /
            // combine scalars were lost; the reader then saw depth 0 and could
            // only leave it opaque. The feature IS played in this AFTER state, so
            // its data container is now live -- re-read the fields here when the
            // BEFORE dump yielded nothing. Scalar params are state-independent;
            // such an extrude's only ents reference the unchanged base body, so
            // AFTER-state anchors are equally valid. Features whose BEFORE dump
            // succeeded keep it untouched (no regression -- a dressup's picked-
            // edge anchors must stay at the BEFORE state, and those dumps are
            // non-empty so this never runs for them).
            if (fields.empty())
            {
                std::vector<FieldDump> after_fields = zwapi::DumpFields(fid);
                zwapi::FtrProbe ap = zwapi::ProbeFeature(fid);
                diag["data_rc_after"]     = ap.data_rc;
                diag["field_count_after"] = ap.field_count;
                if (!after_fields.empty())
                {
                    jf["fields"] = FieldsToJson(after_fields);
                }
            }

            // What geometry did this feature actually produce? For features
            // whose parameters the API won't give us (CdGeomCopy general-
            // errors above), this is where the real content is.
            ResultEnts re = zwapi::FeatureResultEnts(fid);
            if (re.n_shape > 0 || re.n_face > 0 || re.n_curve > 0) {
                jf["result_ents"] = ResultEntsToJson(re);
            }
            diag["n_shape"] = re.n_shape;
            diag["n_face"]  = re.n_face;
            diag["n_curve"] = re.n_curve;

            // A shape-producing feature whose parameters the API won't give
            // us (e.g. CdGeomCopy, an imported base) needs its OWN geometry
            // as authored input -- export just its result shapes to a
            // per-feature STEP. The body is now rolled to AFTER this feature,
            // so these shapes are its own result. Downstream parametric
            // features replay on top of this base.
            if (re.n_shape > 0)
            {
                const std::string fstep = FeatStepPath(out_path, node.id);
                zwapi::StepExportResult fse = zwapi::ExportFeatureShapesStep(fid, fstep);
                if (fse.ok) {
                    // ToUtf8: out_path can be GBK (ZW3D file dialog / batch
                    // queue); raw GBK would hit dump()'s error_handler::replace
                    // and turn into unrecoverable U+FFFD mojibake -- the reader
                    // then can't resolve the sibling STEP by its recorded name.
                    jf["geometry"] = zwapi::ToUtf8(BaseName(fstep).c_str());
                }
                diag["feat_step_init_rc"]   = fse.init_rc;
                diag["feat_step_export_rc"] = fse.export_rc;
                diag["feat_step_bytes"]     = fse.bytes;
                if (fse.empty) {
                    // rc said OK but the translator wrote a hollow file
                    // (no face/solid payload); "geometry" stays unset so
                    // the reader does a clean opaque-skip, not BakedShape.
                    diag["feat_step_empty"] = true;
                }
            }

            // Cumulative-body bake -- ONLY for genuinely un-modelable opaque
            // features: sheet-metal forming (CdSmd*/Smd* flange/tab/punch) and
            // API-opaque features (empty type or unreadable params, e.g. the
            // ___凸包 bosses). NOT the in-place fillets/chamfers/cuts/drafts the
            // reader reconstructs -- baking those exploded DKBA80271607_119 to
            // 105 of 111 full-body STEPs and OOM'd the importer (死机). The
            // body is already rolled to this feature's AFTER state above.
            const bool opaque_geom_feat =
                type.rfind("CdSmd", 0) == 0 || type.rfind("Smd", 0) == 0 ||
                type.empty() || probe.data_rc == -1;
            if (bake_cumulative && opaque_geom_feat && re.n_shape == 0 &&
                at_after && jf.find("geometry") == jf.end())
            {
                const std::string cstep = FeatStepPath(out_path, node.id);
                zwapi::StepExportResult cse = zwapi::ExportPartStep(cstep);
                if (cse.ok && !cse.empty) {
                    jf["geometry"]         = zwapi::ToUtf8(BaseName(cstep).c_str());
                    jf["baked_cumulative"] = true;
                }
                diag["cumul_bake_rc"]    = cse.export_rc;
                diag["cumul_bake_bytes"] = cse.bytes;
            }

            jf["_diag"] = std::move(diag);
        }

        // Cumulative state truth at this feature's AFTER position: what the
        // WHOLE part looks like once features 1..i+1 have executed. This is
        // the per-step baseline zw_verify's --bisect replays against; without
        // it, localizing a divergence means exporting a STEP per probe. The
        // roll is forward-only (the sweep invariant holds: the next
        // iteration's RollBodyBefore still only plays forward).
        if (state_cfg.mode > 0)
        {
            if (!rolled_after)
            {
                rolled_after = zwapi::RollBodyAfter(fid);
            }
            zwapi::StateSnap ss = zwapi::CaptureStateSnap(state_cfg.mode >= 2);
            if (ss.ok)
            {
                json js = StateSnapToJson(ss);
                if (state_cfg.WantStep(node.id))
                {
                    const std::string spath = StateStepPath(out_path, node.id);
                    zwapi::StepExportResult se = zwapi::ExportPartStep(spath);
                    if (se.ok)
                    {
                        js["step"] = zwapi::ToUtf8(BaseName(spath).c_str());
                    }
                    js["step_rc"] = se.export_rc;
                }
                jf["_state"] = std::move(js);
            }
        }

        features.push_back(std::move(jf));
    }

    // Restore: play the body to the end once. Each feature above was read in its
    // own before/after state during the single forward sweep; this returns the
    // model to its final state for the whole-part STEP export below.
    zwapi::RollBodyToEnd();

    doc["document"]["name"]     = zwapi::ActivePartName();
    doc["document"]["features"] = std::move(features);

    // Bodies blanked in the final state: real history products that the
    // visible part (and the truth STEP below) excludes. The reader hands
    // these to the Replayer, which drops the matching solids at emission
    // -- without this list the replay emits them as phantom extra
    // bodies (R2900's blanked plate composite was +19% volume).
    {
        auto hidden = zwapi::CaptureHiddenBodies();
        if (!hidden.empty())
        {
            json arr = json::array();
            for (const auto& h : hidden)
            {
                json b;
                b["bbox"]   = { h.bbox[0], h.bbox[1], h.bbox[2],
                                h.bbox[3], h.bbox[4], h.bbox[5] };
                b["n_face"] = h.n_face;
                arr.push_back(std::move(b));
            }
            doc["hidden_bodies"] = std::move(arr);
        }
    }

    // Truth geometry: export the final solid to a sibling STEP file. This
    // is the universal baseline (correct for every feature) and the truth
    // used downstream for geometric matching / boolean compensation. Best
    // effort -- a failure just omits "geometry", the JSON still stands.
    {
        std::string step_path = out_path;
        const std::string suffix = ".json";
        if (step_path.size() >= suffix.size() &&
            step_path.compare(step_path.size() - suffix.size(), suffix.size(), suffix) == 0)
        {
            step_path = step_path.substr(0, step_path.size() - suffix.size()) + ".step";
        }
        else
        {
            step_path += ".step";
        }
        zwapi::StepExportResult se = zwapi::ExportPartStep(step_path);
        if (se.ok) {
            doc["geometry"] = zwapi::ToUtf8(BaseName(step_path).c_str());
        }
        // Diagnostic: surface why a STEP export failed (best-effort, so a
        // failure is otherwise silent). ZW_API_NO_ERROR is 0.
        json gdiag;
        gdiag["init_rc"]   = se.init_rc;
        gdiag["export_rc"] = se.export_rc;
        gdiag["bytes"]     = se.bytes;
        if (se.empty) {
            gdiag["empty"] = true;
        }
        gdiag["path"]      = zwapi::ToUtf8(step_path.c_str());
        doc["geometry_diag"] = std::move(gdiag);
    }

    // error_handler::replace, not the default strict: a feature label /
    // layer name / external-file path read off the part can carry bytes
    // that aren't valid UTF-8 (e.g. a GBK string from an older field), and
    // strict dump() THROWS on the first such byte -- which would abort the
    // whole export after the STEPs were already written, looking exactly
    // like a failed file open. Replace the offending bytes with U+FFFD so
    // a stray label never costs us the entire snapshot.
    std::string text = doc.dump(2, ' ', false, json::error_handler_t::replace);

    FILE* fp = OpenWriteBinary(out_path);
    if (!fp)
    {
        err = "ExportActivePartToCax: cannot open output: " + out_path;
        return false;
    }
    std::fwrite(text.data(), 1, text.size(), fp);
    std::fclose(fp);
    return true;
}

// ============================================================
// Plugin registration glue lives in ZwCaxPlugin.cpp (ZW3D-SDK / version
// specific). It registers a command whose callback derives out_path and
// calls ExportActivePartToCax above, surfacing `err` through the ZW3D
// message API on failure.
// ============================================================
