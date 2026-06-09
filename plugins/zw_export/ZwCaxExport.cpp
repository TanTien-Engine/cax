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

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

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
#include "zwapi_file.h"                    // cvxFileExportInit / cvxFileExport (STEP truth geometry)
#include "zwapi_entity.h"                  // ZwEntityIdTransfer / ZwEntityHandleFree (int id -> szwEntityHandle); ZwEntityMatrixGet
#include "zwapi_matrix_data.h"             // szwMatrix (sketch insertion-plane world transform)
#include "zwapi_datum.h"                   // ZwDatumAxisDirectionGet (pattern direction axis -> unit vector)
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
    // NOTE: for a curved edge the bbox centre is NOT on the curve (a circular
    // rim's centre is a full radius off it), and TopoRefResolver scores edges
    // by point-to-curve distance -- so this anchor is only "good enough"
    // within the resolver tolerance (~5x the dressup setback), not tight. A
    // tighter on-edge point would need a curve eval, but the picked edge of a
    // chamfer/fillet is CONSUMED at this (the feature's own) state, so it is
    // not reliably curve-evaluable here; rolling back to make it live does NOT
    // help either, because entity ids are scroll-state-local (the id would
    // then point at a different entity). bbox-centre via ZW3D's historical
    // resolution is the pragmatic, proven anchor.
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
                for (int k = 0; k < f.count; ++k) {
                    if (f.fld_data[k].isDirection) {
                        const svxVector& dv = f.fld_data[k].Dir;
                        if (dv.x != 0.0 || dv.y != 0.0 || dv.z != 0.0) {
                            d.dir[0] = dv.x;
                            d.dir[1] = dv.y;
                            d.dir[2] = dv.z;
                            d.has_dir = true;
                            break;
                        }
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

// Export the active part's final solid to a STEP file. This is the
// universal geometry baseline (correct for ALL features regardless of
// whether we parametrise them) and the "truth" geometry used downstream
// for match anchoring / boolean compensation. Whole-part, all objects.
struct StepExportResult
{
    bool ok = false;
    int  init_rc   = -999;
    int  export_rc = -999;
};

StepExportResult ExportPartStep(const std::string& path)
{
    StepExportResult r;
    svxSTEPData data;
    r.init_rc = static_cast<int>(cvxFileExportInit(VX_EXPORT_TYPE_STEP, 0, &data));
    if (r.init_rc != ZW_API_NO_ERROR) {
        return r;
    }
    data.ExportType = 0;   // 0 = all objects
    r.export_rc = static_cast<int>(cvxFileExport(VX_EXPORT_TYPE_STEP, path.c_str(), &data));
    r.ok = (r.export_rc == ZW_API_NO_ERROR);
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
        r.export_rc = static_cast<int>(cvxFileExport(VX_EXPORT_TYPE_STEP, path.c_str(), &data));
        r.ok = (r.export_rc == ZW_API_NO_ERROR);
    }

    cvxMemFree(reinterpret_cast<void**>(&ents));
    return r;
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

// Read an extrude/revolve feature's PROFILE: the curves of its built-in
// sketch(es). cvxPartFtrInqAuxFtrs gives the feature's auxiliary
// (built-in) sketches; ZwSketch2DCurveListGet enumerates each sketch's
// curves; ZwCurveNURBSDataGet reads each curve's geometry. Points come
// back in world 3D, so the profile is already positioned -- no separate
// plane resolution needed to see its shape.
void ReadProfile(int idFtr, Profile& out)
{
    // Read every 2D curve of one sketch (by handle) into out.curves. Shared
    // by the built-in-sketch path and the standalone-sketch fallback below.
    // Read one curve (by handle) into out.curves. ZwCurveNURBSDataGet returns
    // the geometry in the sketch's LOCAL 2D frame (z==0); the reader uses the
    // x/y components plus the plane block to place it in world. Shared by the
    // drawn-2D-curve path and the reference-geometry fallback below.
    auto read_one_curve = [&](szwEntityHandle& ch)
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
        out.curves.push_back(pc);
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

        // 2) Fallback: a profile built from REFERENCE / projected geometry has
        //    no drawn 2D curves (ZwSketch2DCurveListGet -> cn==0) -- R2900_100's
        //    Extrude21/26/30/31. Those edges live in the sketch's reference set
        //    (cvxSkInqRefById) and, failing that, its extended geometry
        //    (cvxSkInqGeomXById). Enumerate by sketch id, transfer each to a
        //    handle, and read it the same way. Runs ONLY when the drawn path
        //    found nothing, so the working drawn-sketch extrudes are untouched.
        if (out.curves.empty() && sketchId > 0)
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
            if (out.curves.empty())
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
        arr.push_back(std::move(jc));
    }
    j["curves"] = std::move(arr);
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
            zwapi::RollBodyAfter(fid);
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
                    jf["geometry"] = BaseName(fstep);
                }
                diag["feat_step_init_rc"]   = fse.init_rc;
                diag["feat_step_export_rc"] = fse.export_rc;
            }

            jf["_diag"] = std::move(diag);
        }

        features.push_back(std::move(jf));
    }

    // Restore: play the body to the end once. Each feature above was read in its
    // own before/after state during the single forward sweep; this returns the
    // model to its final state for the whole-part STEP export below.
    zwapi::RollBodyToEnd();

    doc["document"]["name"]     = zwapi::ActivePartName();
    doc["document"]["features"] = std::move(features);

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
            doc["geometry"] = BaseName(step_path);
        }
        // Diagnostic: surface why a STEP export failed (best-effort, so a
        // failure is otherwise silent). ZW_API_NO_ERROR is 0.
        json gdiag;
        gdiag["init_rc"]   = se.init_rc;
        gdiag["export_rc"] = se.export_rc;
        gdiag["path"]      = step_path;
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
