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
#include <vector>

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
    bool        has_text = false; std::string text;
    std::vector<EntSig> ents;
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
//   cvxMemFree(void**)                        [zwapi_memory.h]
//   vxName == char[32], ZW_API_NO_ERROR       [zwapi_util.h]
// ============================================================

#include "zwapi_part_history.h"            // cvxPartInqFtrList / cvxPartInqFtrTemplate / cvxPartInqFtrData / cvxPartInqFtrEnts
#include "zwapi_memory.h"                  // cvxMemFree
#include "zwapi_util.h"                    // vxName, evxErrors, ZW_API_NO_ERROR, svxBndBox, svxPoint/Vector, VX_ENT_*
#include "zwapi_general_ent.h"             // cvxEntBndBox / cvxEntExists
#include "zwapi_brep_face.h"               // cvxFaceParam / cvxFaceEval (face normal for matching signatures)
#include "zwapi_file.h"                    // cvxFileExportInit / cvxFileExport (STEP truth geometry)
#include "zwapi_entity.h"                  // ZwEntityIdTransfer / ZwEntityHandleFree (int id -> szwEntityHandle)
#include "zwapi_dataexchange.h"            // ZwExternalGeometryCopyDataGet/Free, szwExternalGeometryCopyData (CdGeomCopy)
#include "zwapi_part_opts.h"               // cvxPartHistScrollTo (roll the history bar to read a feature in context)
#include "zwapi_cmd_paramdefine_param.h"   // cvxDataGetAll / cvxDataGetNum / cvxDataGetPnt / cvxDataGetEnts /
                                           // cvxDataGetText / cvxFldDataFree / cvxDataFree;
                                           // svxFldData, evxFldType (VX_FLD_NUM / DST / ANG / PNT / ENT / TXT / DATA)
// Pending the per-feature reads (bind when wiring ReadSketch/ReadExtrude):
//   #include "zwapi_sk_data.h"        // sketch geometry entities
//   #include "zwapi_sk_cons.h"        // sketch constraints
//   #include "zwapi_sk_dim.h"         // sketch dimensions
//   #include "zwapi_cmd_shape_data.h" // extrude parameter struct

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

// Display name. No dedicated localized-name getter is bound yet, so the
// stable template token doubles as the label (name is round-trip-only and
// non-critical). TODO: bind a localized feature-name getter if wanted.
std::string FeatureName(int idFtr)
{
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

// Geometric signature of a referenced entity, for later matching in OCCT.
// Faces get an on-surface point (evaluated at mid-UV) + normal; anything
// else gets its bounding-box centre. The ZW3D entity id is NOT recorded
// -- it is meaningless once the part is rebuilt in OCCT; only geometry
// survives a cross-kernel rebuild.
EntSig EntitySig(int idEnt)
{
    EntSig s;
    if (cvxEntExists(idEnt, VX_ENT_FACE)) { s.kind = "face"; }
    else if (cvxEntExists(idEnt, VX_ENT_EDGE)) { s.kind = "edge"; }
    else { s.kind = "ent"; }

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
    return s;
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
                d.type = "list";   // nested container; deep dump is future work
            }
            else
            {
                d.type = "other";
            }

            out.push_back(std::move(d));
        }
        cvxFldDataFree(numFld, &fldData);
    }

    cvxDataFree(idData);
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

    // History is linear per part, so the running solid tip is just the
    // previous solid-producing feature's id. Sketches don't advance it.
    uint32_t prev_solid_id = 0;

    for (size_t i = 0; i < feats.size(); ++i)
    {
        int fid = feats[i];

        // Roll the history bar to this feature so its data reads in the
        // state where it executed (see ScrollHistoryTo). Restored to the
        // end after the loop.
        int scroll_rc = zwapi::ScrollHistoryTo(fid);

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
            diag["scroll_rc"]   = scroll_rc;
            diag["data_rc"]     = probe.data_rc;
            diag["field_count"] = probe.field_count;

            std::vector<FieldDump> fields = zwapi::DumpFields(fid);
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
            // per-feature STEP. The history is rolled to this feature, so
            // these shapes are its own result. Downstream parametric
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

    // Restore the history rollback bar to the end (we rolled it back per
    // feature above to read each in its own context).
    zwapi::ScrollHistoryTo(feats.back());

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

    std::string text = doc.dump(2);

    FILE* fp = std::fopen(out_path.c_str(), "wb");
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
