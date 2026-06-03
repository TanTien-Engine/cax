// ============================================================
// cadcvt/reader/ZwReader.cpp
//
// ZW3D neutral intermediate (.cax.json) -> cadapp::DocumentIR.
//
// No ZW3D SDK dependency: the feature-tree walk happened inside ZW3D
// (zw_export plugin, plugins/zw_export/ZwCaxExport.cpp); this file only
// parses the JSON snapshot. Mapping targets the real cadapp IR (FeatType
// / SkGeoType / SkConsType / ExtrudeEndType / FeatPayloadExtrude /
// SkGeoIR builders). Every wire token comes from the shared contract
// header interop/CaxIntermediateSchema.h, so the reader and the plugin
// writer can never drift on a token spelling.
//
// Build gate: the JSON parser (nlohmann, header-only) is the reader's
// only external dependency and is pulled in as the thirdparty/nlohmann
// submodule. When it is absent CAX_ZW_OK is undefined and ReadFile
// returns a clean "not built" error instead of failing to compile --
// the same shape as SwReader stubbing out without the SolidWorks libs.
// ============================================================

#include "cadcvt_c/reader/ZwReader.h"

#include "interop/CaxIntermediateSchema.h"

#ifdef CAX_ZW_OK

#include "cadapp_c/ir/FeatureIR.h"
#include "cadapp_c/ir/SketchIR.h"
#include "cadapp_c/ir/Enums.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <unordered_map>
#include <vector>

// For opening a UTF-8 path on Windows (the .cax.json may sit under a
// Chinese directory / part name). std::ifstream(const std::string&) goes
// through the narrow CRT (ANSI/GBK code page) there and fails to open it.
#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#endif

namespace cadcvt
{
namespace
{

using json = nlohmann::json;
namespace sc = cax_schema;

// Open a UTF-8 file path for binary reading. On Windows a non-ASCII path
// (e.g. a Chinese part name) won't open through std::ifstream's narrow
// constructor, which uses the ANSI code page -- the read-side mirror of
// the std::fopen trap the zw_export plugin hit writing the JSON. Convert
// UTF-8 -> UTF-16 and use the wide-path ifstream overload (MSVC
// extension). Mirrors SwReader's Widen(); elsewhere std::ifstream is fine.
std::ifstream OpenInputBinary(const std::string& path)
{
#ifdef _WIN32
    auto widen = [](const std::string& p, UINT cp, DWORD flags) -> std::wstring
    {
        int n = ::MultiByteToWideChar(cp, flags, p.data(), (int)p.size(),
                                      nullptr, 0);
        if (n <= 0) {
            return std::wstring();
        }
        std::wstring w((size_t)n, L'\0');
        ::MultiByteToWideChar(cp, flags, p.data(), (int)p.size(), w.data(), n);
        return w;
    };
    if (!path.empty())
    {
        // Prefer UTF-8 (the .ves scene files are UTF-8); fall back to the
        // system ANSI code page if the bytes aren't valid UTF-8 (a GBK path
        // -- ZW3D's own file API hands names back in the ANSI code page).
        std::wstring w = widen(path, CP_UTF8, MB_ERR_INVALID_CHARS);
        if (w.empty()) {
            w = widen(path, CP_ACP, 0);
        }
        if (!w.empty()) {
            std::ifstream in(w.c_str(), std::ios::binary);
            if (in) {
                return in;
            }
        }
    }
#endif
    return std::ifstream(path, std::ios::binary);
}

// ---- small JSON accessors ----------------------------------------------

// Numeric / bool field with a default when absent or null.
template <typename T>
T JGet(const json& j, const char* key, T def)
{
    auto it = j.find(key);
    if (it == j.end() || it->is_null()) {
        return def;
    }
    return it->get<T>();
}

// String field with a default when absent / null / not-a-string.
std::string JStr(const json& j, const char* key, const char* def)
{
    auto it = j.find(key);
    if (it == j.end() || it->is_null() || !it->is_string()) {
        return def;
    }
    return it->get<std::string>();
}

// Read a 3-element array into out, multiplying by scale (pass 1.0 for
// unit-vector fields like x_dir / normal; pass the unit scale for
// positional fields like origin).
void ReadVec3(const json& arr, double scale, double out[3])
{
    out[0] = arr.at(0).get<double>() * scale;
    out[1] = arr.at(1).get<double>() * scale;
    out[2] = arr.at(2).get<double>() * scale;
}

// ---- token -> enum maps -------------------------------------------------

cadapp::ExtrudeEndType MapEndCond(const std::string& s)
{
    using E = cadapp::ExtrudeEndType;
    if (s == sc::end_cond::ThroughAll) { return E::ThroughAll; }
    if (s == sc::end_cond::UpToSurface) { return E::UpToSurface; }
    if (s == sc::end_cond::UpToVertex) { return E::UpToVertex; }
    if (s == sc::end_cond::MidPlane) { return E::MidPlane; }
    if (s == sc::end_cond::OffsetFromSurface) { return E::OffsetFromSurface; }
    if (s == sc::end_cond::UpToFirst) { return E::UpToFirst; }
    return E::Blind;
}

cadapp::SkPointPos MapPointPos(const std::string& s)
{
    using P = cadapp::SkPointPos;
    if (s == sc::pos::Start) { return P::Start; }
    if (s == sc::pos::Mid) { return P::Mid; }
    if (s == sc::pos::End) { return P::End; }
    if (s == sc::pos::Center) { return P::Center; }
    return P::None;
}

cadapp::InputRole MapInputRole(const std::string& s)
{
    using R = cadapp::InputRole;
    if (s == sc::role::Operand) { return R::Operand; }
    if (s == sc::role::Tool) { return R::Tool; }
    if (s == sc::role::PatternTarget) { return R::PatternTarget; }
    if (s == sc::role::Reference) { return R::Reference; }
    return R::Base;
}

cadapp::SkConsType MapConsType(const std::string& s)
{
    using C = cadapp::SkConsType;
    static const std::unordered_map<std::string, C> kMap = {
        { sc::cons::Distance,       C::Distance },
        { sc::cons::DistanceX,      C::DistanceX },
        { sc::cons::DistanceY,      C::DistanceY },
        { sc::cons::Angle,          C::Angle },
        { sc::cons::Parallel,       C::Parallel },
        { sc::cons::Perpendicular,  C::Perpendicular },
        { sc::cons::Coincident,     C::Coincident },
        { sc::cons::Horizontal,     C::Horizontal },
        { sc::cons::Vertical,       C::Vertical },
        { sc::cons::Equal,          C::Equal },
        { sc::cons::Tangent,        C::Tangent },
        { sc::cons::Concentric,     C::Concentric },
        { sc::cons::Symmetric,      C::Symmetric },
        { sc::cons::Colinear,       C::Colinear },
        { sc::cons::Fix,            C::Fix },
        { sc::cons::CircleRadius,   C::CircleRadius },
        { sc::cons::CircleDiameter, C::CircleDiameter },
        { sc::cons::ArcRadius,      C::ArcRadius },
        { sc::cons::ArcDiameter,    C::ArcDiameter },
    };
    auto it = kMap.find(s);
    return (it == kMap.end()) ? C::None : it->second;
}

// Constraints whose value is a length (and so must be unit-scaled).
// Angle is radians and stays unscaled; geometric constraints carry 0.
bool IsLengthCons(cadapp::SkConsType t)
{
    using C = cadapp::SkConsType;
    switch (t)
    {
        case C::Distance:
        case C::DistanceX:
        case C::DistanceY:
        case C::CircleRadius:
        case C::CircleDiameter:
        case C::ArcRadius:
        case C::ArcDiameter:
            return true;
        default:
            return false;
    }
}

// ---- builders -----------------------------------------------------------

// Keep input_feature_ids and input_roles in lock-step (the FeatureIR
// invariant; same helper SwReader / FreeCadReader use).
void PushInput(cadapp::FeatureIR& feat, uint32_t id, cadapp::InputRole role)
{
    feat.input_feature_ids.push_back(id);
    feat.input_roles.push_back(role);
}

cadapp::SkGeoRef ReadRef(const json& jr)
{
    cadapp::SkGeoRef r;
    int geo = JGet<int>(jr, "geo", -1);
    r.geo_id    = (geo < 0) ? 0xFFFFFFFFu : static_cast<uint32_t>(geo);
    r.point_pos = MapPointPos(JStr(jr, "pos", sc::pos::None));
    return r;
}

// One sketch geometry. Positional values are scaled by s; angles
// (arc start/end) are left in radians.
cadapp::SkGeoIR BuildGeom(const json& jg, double s)
{
    using G = cadapp::SkGeoIR;

    uint32_t    id   = JGet<uint32_t>(jg, "geo_id", 0u);
    bool        cons = JGet<bool>(jg, "construction", false);
    std::string type = JStr(jg, "type", sc::geo::Unknown);

    if (type == sc::geo::Point)
    {
        const auto& p = jg.at("pt");
        return G::Point(id,
                        p.at(0).get<double>() * s,
                        p.at(1).get<double>() * s,
                        cons);
    }
    if (type == sc::geo::Line)
    {
        const auto& a = jg.at("p0");
        const auto& b = jg.at("p1");
        return G::Line(id,
                       a.at(0).get<double>() * s, a.at(1).get<double>() * s,
                       b.at(0).get<double>() * s, b.at(1).get<double>() * s,
                       cons);
    }
    if (type == sc::geo::Arc)
    {
        const auto& c  = jg.at("center");
        double      r  = jg.at("radius").get<double>() * s;
        double      a0 = jg.at("start_ang").get<double>();
        double      a1 = jg.at("end_ang").get<double>();
        return G::Arc(id,
                      c.at(0).get<double>() * s, c.at(1).get<double>() * s,
                      r, a0, a1,
                      cons);
    }
    if (type == sc::geo::Circle)
    {
        const auto& c = jg.at("center");
        double      r = jg.at("radius").get<double>() * s;
        return G::Circle(id,
                         c.at(0).get<double>() * s, c.at(1).get<double>() * s,
                         r,
                         cons);
    }
    if (type == sc::geo::Ellipse)
    {
        const auto& c  = jg.at("center");
        double      mr = jg.at("major_r").get<double>() * s;
        double      nr = jg.at("minor_r").get<double>() * s;
        return G::Ellipse(id,
                          c.at(0).get<double>() * s, c.at(1).get<double>() * s,
                          mr, nr,
                          cons);
    }
    if (type == sc::geo::Spline)
    {
        std::vector<double> xs;
        std::vector<double> ys;
        for (const auto& pt : jg.at("ctrl"))
        {
            xs.push_back(pt.at(0).get<double>() * s);
            ys.push_back(pt.at(1).get<double>() * s);
        }
        return G::Spline(id, xs, ys, cons);
    }

    // Unknown geometry kind: emit a None geo so its id still resolves
    // for any constraint that references it; the solver skips None.
    cadapp::SkGeoIR g;
    g.id           = id;
    g.type         = cadapp::SkGeoType::None;
    g.construction = cons;
    return g;
}

void BuildSketch(const json& jf, double s, uint32_t& next_cons_id, cadapp::SketchIR& sk)
{
    const auto& plane = jf.at("plane");
    ReadVec3(plane.at("origin"), s,   sk.plane_origin);
    ReadVec3(plane.at("x_dir"),  1.0, sk.plane_x_dir);
    ReadVec3(plane.at("normal"), 1.0, sk.plane_normal);

    auto git = jf.find("geoms");
    if (git != jf.end())
    {
        for (const auto& jg : *git) {
            sk.geos.push_back(BuildGeom(jg, s));
        }
    }

    auto cit = jf.find("constraints");
    if (cit != jf.end())
    {
        for (const auto& jc : *cit)
        {
            cadapp::SkConsType type = MapConsType(JStr(jc, "type", sc::cons::None));
            if (type == cadapp::SkConsType::None) {
                continue;
            }
            cadapp::SkConsIR c;
            c.id      = next_cons_id++;
            c.type    = type;
            c.a       = ReadRef(jc.at("a"));
            c.b       = ReadRef(jc.at("b"));
            c.value   = JGet<double>(jc, "value", 0.0);
            c.driving = JGet<bool>(jc, "driving", true);
            if (IsLengthCons(type)) {
                c.value *= s;
            }
            sk.cons.push_back(c);
        }
    }
}

void BuildExtrude(const json& jf, double s, cadapp::FeatureIR& feat)
{
    cadapp::FeatPayloadExtrude pl;
    pl.sketch_id      = JGet<uint32_t>(jf, "profile_id", 0xFFFFFFFFu);
    pl.distance       = JGet<double>(jf, "depth", 0.0) * s;
    pl.distance2      = JGet<double>(jf, "depth2", 0.0) * s;
    pl.end_type       = MapEndCond(JStr(jf, "end_cond", sc::end_cond::Blind));
    pl.end_type2      = MapEndCond(JStr(jf, "end_cond2", sc::end_cond::Blind));
    pl.flip_direction = JGet<bool>(jf, "flip", false);
    pl.is_thin        = JGet<bool>(jf, "thin", false);
    pl.thin_thickness = JGet<double>(jf, "thin_thickness", 0.0) * s;
    // direction stays at the payload default {0,0,1}; the Replayer
    // resolves the world-space extrude direction from the sketch plane,
    // the same convention SwReader relies on.

    std::string subkind = JStr(jf, "subkind", sc::subkind::Boss);
    feat.type = (subkind == sc::subkind::Cut) ? cadapp::FeatType::CutExtrude
                                              : cadapp::FeatType::BossExtrude;
    feat.data = std::move(pl);
}

void ApplyInputs(const json& jf, cadapp::FeatureIR& feat)
{
    auto it = jf.find("inputs");
    if (it == jf.end()) {
        return;
    }
    for (const auto& ji : *it)
    {
        uint32_t          id   = JGet<uint32_t>(ji, "id", 0u);
        cadapp::InputRole role = MapInputRole(JStr(ji, "role", sc::role::Base));
        PushInput(feat, id, role);
    }
}

// ---- ZW3D feature-style helpers (opaque features that carry richer data) -

// Value of a feature data-container field by its stable id, from the full
// "fields" dump the plugin emits on opaque features. Returns def if absent.
double FieldValueById(const json& jf, int fld_id, double def)
{
    auto fit = jf.find("fields");
    if (fit == jf.end() || !fit->is_array()) {
        return def;
    }
    for (const auto& fd : *fit)
    {
        auto idit = fd.find("id");
        if (idit != fd.end() && idit->is_number() && idit->get<int>() == fld_id)
        {
            auto vit = fd.find("value");
            if (vit != fd.end() && vit->is_number()) {
                return vit->get<double>();
            }
        }
    }
    return def;
}

// Build a SketchIR from a ZW3D extrude's built-in profile ("profile" block:
// a list of world-3D curves). First cut assumes the profile lies on the XY
// plane (z==0 for the test part), so the sketch plane is XY and 2D coords
// are the world (x,y). General plane fitting from the curve points is TODO.
void BuildSketchFromProfile(const json& profile, double s, uint32_t feature_id,
                            cadapp::SketchIR& sk)
{
    sk.feature_id = feature_id;
    sk.plane_origin[0] = 0.0; sk.plane_origin[1] = 0.0; sk.plane_origin[2] = 0.0;
    sk.plane_x_dir[0]  = 1.0; sk.plane_x_dir[1]  = 0.0; sk.plane_x_dir[2]  = 0.0;
    sk.plane_normal[0] = 0.0; sk.plane_normal[1] = 0.0; sk.plane_normal[2] = 1.0;

    auto cit = profile.find("curves");
    if (cit == profile.end() || !cit->is_array()) {
        return;
    }

    const double kDegToRad = 3.14159265358979323846 / 180.0;
    uint32_t gid = 1;
    for (const auto& c : *cit)
    {
        const std::string ck = JStr(c, "kind", "");
        if (ck == "line")
        {
            const auto& a = c.at("p0");
            const auto& b = c.at("p1");
            sk.geos.push_back(cadapp::SkGeoIR::Line(gid++,
                a.at(0).get<double>() * s, a.at(1).get<double>() * s,
                b.at(0).get<double>() * s, b.at(1).get<double>() * s));
        }
        else if (ck == "circle")
        {
            const auto& ctr = c.at("center");
            double r = c.at("radius").get<double>() * s;
            sk.geos.push_back(cadapp::SkGeoIR::Circle(gid++,
                ctr.at(0).get<double>() * s, ctr.at(1).get<double>() * s, r));
        }
        else if (ck == "arc")
        {
            const auto& ctr = c.at("center");
            double r  = c.at("radius").get<double>() * s;
            double a0 = JGet<double>(c, "a0", 0.0) * kDegToRad;  // szwCurve angles
            double a1 = JGet<double>(c, "a1", 0.0) * kDegToRad;  // are in degrees
            sk.geos.push_back(cadapp::SkGeoIR::Arc(gid++,
                ctr.at(0).get<double>() * s, ctr.at(1).get<double>() * s, r, a0, a1));
        }
        // nurb / ellipse: not reconstructed yet (TODO).
    }
}

} // namespace

ZwReader::ZwReader() = default;
ZwReader::~ZwReader() = default;

bool ZwReader::ReadFile(const std::string& path,
                        cadapp::DocumentIR& out,
                        std::string*       err_msg)
{
    auto fail = [&](const std::string& m) -> bool
    {
        if (err_msg) {
            *err_msg = m;
        }
        return false;
    };

    std::ifstream in = OpenInputBinary(path);
    if (!in) {
        return fail("ZwReader: cannot open " + path);
    }
    std::string text((std::istreambuf_iterator<char>(in)),
                     std::istreambuf_iterator<char>());

    json doc;
    try {
        doc = json::parse(text);
    } catch (const json::exception& e) {
        return fail(std::string("ZwReader: JSON parse error: ") + e.what());
    }

    // Version handshake: refuse a snapshot whose wire format this reader
    // does not understand, rather than silently building a wrong IR. A
    // missing version is tolerated (treated as the current version) so a
    // hand-authored test fixture need not carry the field.
    int ver = JGet<int>(doc, "schema_version", sc::kSchemaVersion);
    if (ver != sc::kSchemaVersion) {
        return fail("ZwReader: unsupported schema_version " + std::to_string(ver) +
                    " (this build understands " + std::to_string(sc::kSchemaVersion) +
                    "); re-export with a matching zw_export plugin");
    }

    out.source   = Name();
    out.doc_path = path;

    // length_unit -> scale; target is the project's metre convention.
    std::string unit = JStr(doc, "length_unit", sc::unit::Mm);
    if (unit == sc::unit::Mm) { m_unit_scale = 0.001; }
    else if (unit == sc::unit::Cm) { m_unit_scale = 0.01; }
    else if (unit == sc::unit::M) { m_unit_scale = 1.0; }
    else if (unit == sc::unit::In) { m_unit_scale = 0.0254; }
    // else: keep whatever SetUnitScale left.
    const double s = m_unit_scale;

    // Directory of the .cax.json, used to resolve the sibling per-feature
    // STEP refs (CdGeomCopy's authored base geometry).
    std::string doc_dir;
    {
        auto sl = path.find_last_of("/\\");
        if (sl != std::string::npos) {
            doc_dir = path.substr(0, sl + 1);
        }
    }

    // Running body tip: the last solid-producing feature's id, so a boss
    // extrude fuses onto the imported base instead of floating standalone.
    uint32_t running_solid_id = 0;

    try
    {
        const auto& features = doc.at("document").at("features");
        for (const auto& jf : features)
        {
            uint32_t    id   = JGet<uint32_t>(jf, "id", 0u);
            std::string name = JStr(jf, "name", "");
            std::string kind = JStr(jf, "kind", sc::kind::Opaque);

            if (kind == sc::kind::Sketch)
            {
                cadapp::SketchIR sk;
                sk.feature_id = id;
                sk.name       = name;
                BuildSketch(jf, s, m_next_sketch_cons_id, sk);
                out.sketches.push_back(std::move(sk));

                cadapp::FeatPayloadSketch pl;
                pl.sketch_id = id;
                cadapp::FeatureIR f;
                f.id   = id;
                f.type = cadapp::FeatType::Sketch;
                f.name = name;
                f.data = std::move(pl);
                out.features.push_back(std::move(f));
            }
            else if (kind == sc::kind::Extrude)
            {
                cadapp::FeatureIR f;
                f.id   = id;
                f.name = name;
                BuildExtrude(jf, s, f);
                ApplyInputs(jf, f);
                out.features.push_back(std::move(f));
            }
            else if (kind == sc::kind::Box)
            {
                // Sketch-less box primitive -> FeatPayloadPrimBox. Sizes
                // arrive positive (the plugin took magnitudes) and in the
                // file's length_unit, so scale to the IR's metre units. No
                // inputs: a primitive at the root of a body is a body root
                // (the Replayer fuses nothing for a missing base).
                cadapp::FeatPayloadPrimBox pl;
                pl.length = JGet<double>(jf, "length", 1.0) * s;
                pl.width  = JGet<double>(jf, "width",  1.0) * s;
                pl.height = JGet<double>(jf, "height", 1.0) * s;

                cadapp::FeatureIR f;
                f.id   = id;
                f.type = cadapp::FeatType::PrimBox;
                f.name = name;
                f.data = std::move(pl);

                // Placement: "placement" is the box's world min-corner --
                // where OCCT's origin-anchored box must be translated. The
                // Replayer's FinalizePrimitiveNode applies a translation
                // from ext_params["placement_px/py/pz"] to the primitive.
                auto plc = jf.find("placement");
                if (plc != jf.end() && plc->is_array() && plc->size() == 3)
                {
                    f.ext_params["placement_px"] = plc->at(0).get<double>() * s;
                    f.ext_params["placement_py"] = plc->at(1).get<double>() * s;
                    f.ext_params["placement_pz"] = plc->at(2).get<double>() * s;
                }

                out.features.push_back(std::move(f));
            }
            else
            {
                std::string zt = JStr(jf, "zw_type", "Unknown");

                // CdGeomCopy that imports a SOLID base -> FeatPayloadBakedShape
                // (an authored body root). The Replayer makes a const node
                // from doc.authored_shapes[id], which ZwLoader fills by
                // loading the per-feature STEP referenced in zw_geometry.
                // Heuristic: a closed solid has several faces (n_face >= 4);
                // a CdGeomCopy of a couple of reference surfaces stays opaque.
                {
                    auto geo = jf.find("geometry");
                    int  n_face = 0;
                    auto re = jf.find("result_ents");
                    if (re != jf.end()) {
                        n_face = JGet<int>(*re, "n_face", 0);
                    }
                    if (zt == "CdGeomCopy" && geo != jf.end() &&
                        geo->is_string() && n_face >= 4)
                    {
                        cadapp::FeatPayloadBakedShape pl;
                        cadapp::FeatureIR f;
                        f.id   = id;
                        f.type = cadapp::FeatType::BakedShape;
                        f.name = name;
                        f.data = std::move(pl);
                        f.ext_strings["zw_type"]     = zt;
                        f.ext_strings["zw_geometry"] = doc_dir + geo->get<std::string>();
                        out.features.push_back(std::move(f));
                        running_solid_id = id;
                        continue;
                    }
                }

                // ZW3D solid extrude (FtAllExt) with a closed built-in
                // profile -> reconstruct as Sketch + BossExtrude, fused onto
                // the running body (the imported base) when there is one.
                // A profile with < 3 curves (e.g. the single-line surface
                // extrude) is left opaque.
                auto prof = jf.find("profile");
                if (zt == "FtAllExt" && prof != jf.end() &&
                    prof->contains("curves") && prof->at("curves").is_array() &&
                    prof->at("curves").size() >= 3)
                {
                    const uint32_t sketch_fid = 1000000u + id;

                    cadapp::SketchIR sk;
                    sk.name = name + ":profile";
                    BuildSketchFromProfile(*prof, s, sketch_fid, sk);
                    out.sketches.push_back(std::move(sk));

                    cadapp::FeatPayloadSketch spl;
                    spl.sketch_id = sketch_fid;
                    cadapp::FeatureIR sf;
                    sf.id   = sketch_fid;
                    sf.type = cadapp::FeatType::Sketch;
                    sf.name = name + ":profile";
                    sf.data = std::move(spl);
                    out.features.push_back(std::move(sf));

                    // depth = End E (fld 3) - Start S (fld 2), from the field
                    // dump; magnitude (direction resolved from plane normal).
                    double endE   = FieldValueById(jf, 3, 0.0);
                    double startS = FieldValueById(jf, 2, 0.0);
                    double depth  = std::fabs(endE - startS) * s;

                    cadapp::FeatPayloadExtrude epl;
                    epl.sketch_id = sketch_fid;
                    epl.distance  = depth;
                    epl.end_type  = cadapp::ExtrudeEndType::Blind;
                    cadapp::FeatureIR ef;
                    ef.id   = id;
                    ef.type = cadapp::FeatType::BossExtrude;
                    ef.name = name;
                    ef.data = std::move(epl);

                    // Fuse onto the running body (the imported base) when
                    // present; the Replayer's boss-extrude arm boolean-adds
                    // the prism to the base input. Otherwise standalone.
                    if (running_solid_id != 0) {
                        PushInput(ef, running_solid_id, cadapp::InputRole::Base);
                    }
                    out.features.push_back(std::move(ef));
                    running_solid_id = id;
                    continue;
                }

                if (m_strict) {
                    return fail("ZwReader: unknown feature kind '" + kind +
                                "' for feature " + name);
                }
                // Opaque: visible in the IR, carries its raw ZW3D type,
                // and is NOT wired into the body chain (no inputs), so
                // the recognised features still replay around it.
                cadapp::FeatPayloadOpaque pl;
                pl.strings["zw_type"] = zt;

                // Full field dump (keyed by stable ZW3D field id). Scalars
                // land in params; field labels in strings. The richer parts
                // (points, entity signatures) stay in the JSON for now --
                // they feed the reconstruction phase, not the opaque IR.
                auto fit = jf.find("fields");
                if (fit != jf.end() && fit->is_array())
                {
                    for (const auto& fd : *fit)
                    {
                        auto idit = fd.find("id");
                        if (idit == fd.end() || !idit->is_number()) {
                            continue;
                        }
                        const std::string key = std::to_string(idit->get<int>());

                        auto vit = fd.find("value");
                        if (vit != fd.end() && vit->is_number()) {
                            pl.params[key] = vit->get<double>();
                        }
                        auto nm = fd.find("name");
                        if (nm != fd.end() && nm->is_string()) {
                            pl.strings["fld." + key] = nm->get<std::string>();
                        }
                    }
                }

                cadapp::FeatureIR f;
                f.id   = id;
                f.type = cadapp::FeatType::Unknown;
                f.name = name;
                f.data = std::move(pl);
                f.ext_strings["zw_type"] = zt;

                out.features.push_back(std::move(f));
            }
        }
    }
    catch (const json::exception& e)
    {
        return fail(std::string("ZwReader: malformed intermediate: ") + e.what());
    }

    return true;
}

} // namespace cadcvt

#else // !CAX_ZW_OK

namespace cadcvt
{

ZwReader::ZwReader() = default;
ZwReader::~ZwReader() = default;

// Stub build: the JSON parser (thirdparty/nlohmann) was not present at
// configure time, so the reader was compiled without its body. Mirrors
// SwReader's behaviour when the SolidWorks libraries are absent.
bool ZwReader::ReadFile(const std::string& path,
                        cadapp::DocumentIR& out,
                        std::string*       err_msg)
{
    (void)path;
    (void)out;
    if (err_msg) {
        *err_msg = "ZwReader: not built (nlohmann/json missing; run: "
                   "git submodule update --init thirdparty/nlohmann)";
    }
    return false;
}

} // namespace cadcvt

#endif // CAX_ZW_OK
    