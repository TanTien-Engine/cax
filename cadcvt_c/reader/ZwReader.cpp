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
#include "cadapp_c/ir/TopoRefIR.h"

#include <nlohmann/json.hpp>

#include <array>
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

// Point value (world mm) of a PNT-type field by its stable id -- e.g. a
// pattern's "Direction" (fld 2), whose cached pick "pt" lies ON the
// referenced axis/edge. Returns false (out untouched) when absent.
bool FieldPoint(const json& jf, int fld_id, double out[3])
{
    auto fit = jf.find("fields");
    if (fit == jf.end() || !fit->is_array()) {
        return false;
    }
    for (const auto& fd : *fit)
    {
        auto idit = fd.find("id");
        if (idit == fd.end() || !idit->is_number() || idit->get<int>() != fld_id) {
            continue;
        }
        auto pit = fd.find("pt");
        if (pit == fd.end() || !pit->is_array() || pit->size() < 3) {
            return false;
        }
        out[0] = pit->at(0).get<double>();
        out[1] = pit->at(1).get<double>();
        out[2] = pit->at(2).get<double>();
        return true;
    }
    return false;
}

// Unit DIRECTION of a field by its stable id -- a "Direction" widget (a
// pattern's fld 5 "Direction D" / fld 2 "Direction", a revolve axis, ...)
// stores its resolved direction in the field data's Dir member, which the
// plugin emits as "dir":[x,y,z]. This is ZW3D's TRUE signed direction; it is
// preferred over the edge-derived pattern.dir, whose sign -- and even axis --
// follow the referenced edge's arbitrary parametric orientation. Returns
// false (out untouched) when the field carries no (non-zero) direction.
bool FieldDir(const json& jf, int fld_id, double out[3])
{
    auto fit = jf.find("fields");
    if (fit == jf.end() || !fit->is_array()) {
        return false;
    }
    for (const auto& fd : *fit)
    {
        auto idit = fd.find("id");
        if (idit == fd.end() || !idit->is_number() || idit->get<int>() != fld_id) {
            continue;
        }
        auto dit = fd.find("dir");
        if (dit == fd.end() || !dit->is_array() || dit->size() < 3) {
            return false;
        }
        double d[3] = { dit->at(0).get<double>(),
                        dit->at(1).get<double>(),
                        dit->at(2).get<double>() };
        if (std::fabs(d[0]) < 1e-12 && std::fabs(d[1]) < 1e-12 &&
            std::fabs(d[2]) < 1e-12) {
            return false;
        }
        out[0] = d[0]; out[1] = d[1]; out[2] = d[2];
        return true;
    }
    return false;
}

// First entity-reference anchor (world mm) of field fld_id from the dump --
// e.g. a pattern's "Base" (fld 1), whose anchor is the patterned feature's
// location. Returns false (out untouched) when the field / anchor is absent.
bool FieldEntAnchor(const json& jf, int fld_id, double out[3])
{
    auto fit = jf.find("fields");
    if (fit == jf.end() || !fit->is_array()) {
        return false;
    }
    for (const auto& fd : *fit)
    {
        auto idit = fd.find("id");
        if (idit == fd.end() || !idit->is_number() || idit->get<int>() != fld_id) {
            continue;
        }
        auto eit = fd.find("ents");
        if (eit == fd.end() || !eit->is_array() || eit->empty()) {
            return false;
        }
        const auto& a = eit->at(0).find("anchor");
        if (a == eit->at(0).end() || !a->is_array() || a->size() < 3) {
            return false;
        }
        out[0] = a->at(0).get<double>();
        out[1] = a->at(1).get<double>();
        out[2] = a->at(2).get<double>();
        return true;
    }
    return false;
}

// Build a SketchIR from a ZW3D extrude's built-in profile ("profile" block).
// The curves are in the sketch's LOCAL 2D frame (u,v with z==0 on the plane),
// so they map straight onto the SketchIR's 2D plane coords; the "plane" block
// (origin/x_dir/normal in world, from the plugin's ZwEntityMatrixGet) is what
// positions that frame in space. Absent a plane block (older snapshots, or a
// sketch drawn on world XY), the default is XY at z==0 -- which is why the
// base extrude, on z==0, was already correct without it.
void BuildSketchFromProfile(const json& profile, double s, uint32_t feature_id,
                            cadapp::SketchIR& sk)
{
    sk.feature_id = feature_id;
    sk.plane_origin[0] = 0.0; sk.plane_origin[1] = 0.0; sk.plane_origin[2] = 0.0;
    sk.plane_x_dir[0]  = 1.0; sk.plane_x_dir[1]  = 0.0; sk.plane_x_dir[2]  = 0.0;
    sk.plane_normal[0] = 0.0; sk.plane_normal[1] = 0.0; sk.plane_normal[2] = 1.0;

    auto plit = profile.find("plane");
    if (plit != profile.end() && plit->is_object())
    {
        // origin is positional (scale mm->m); x_dir / normal are unit dirs.
        auto rd = [&](const char* key, double scale, double out3[3])
        {
            auto a = plit->find(key);
            if (a != plit->end() && a->is_array() && a->size() == 3) {
                out3[0] = a->at(0).get<double>() * scale;
                out3[1] = a->at(1).get<double>() * scale;
                out3[2] = a->at(2).get<double>() * scale;
            }
        };
        rd("origin", s,   sk.plane_origin);
        rd("x_dir",  1.0, sk.plane_x_dir);
        rd("normal", 1.0, sk.plane_normal);
    }

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

// A ZW3D extrude (FtAllExt) yields a SOLID only when its profile bounds a
// closed region. The historic guard "curves >= 3" rejected the single-line
// surface extrude correctly, but ALSO dropped perfectly good 1-2 curve solid
// profiles: a lone circle (a pin / hole), two concentric circles (a ring),
// or a 2-curve loop (arc + chord, a D-section). Accept those; keep every
// >= 3 profile exactly as before (zero risk to the base or existing bosses).
bool ProfileExtrudable(const json& curves)
{
    if (!curves.is_array()) {
        return false;
    }
    const size_t n = curves.size();
    if (n >= 3) {
        return true;            // unchanged: the existing multi-curve path
    }
    if (n == 0) {
        return false;
    }
    // n == 1 or 2: a solid only if the curves close. A circle / ellipse is
    // self-closing; a line / arc must chain endpoint-to-endpoint, no dangling
    // end. (A lone line or arc is the open surface-extrude case -> reject.)
    int closed_loops = 0;
    std::vector<std::array<double, 2>> ends;
    for (const auto& c : curves)
    {
        const std::string k = JStr(c, "kind", "");
        if (k == "circle" || k == "ellipse") {
            ++closed_loops;
            continue;
        }
        auto p0 = c.find("p0");
        auto p1 = c.find("p1");
        if (p0 == c.end() || p1 == c.end() ||
            !p0->is_array() || !p1->is_array() ||
            p0->size() < 2 || p1->size() < 2) {
            return false;       // open curve we can't verify -> not safe
        }
        ends.push_back({ p0->at(0).get<double>(), p0->at(1).get<double>() });
        ends.push_back({ p1->at(0).get<double>(), p1->at(1).get<double>() });
    }
    // Every open endpoint must coincide with exactly one other (a closed
    // chain). With only 1-2 curves this pairing is cheap and exact.
    const double tol = 1e-6;
    std::vector<bool> used(ends.size(), false);
    for (size_t i = 0; i < ends.size(); ++i)
    {
        if (used[i]) { continue; }
        bool matched = false;
        for (size_t j = i + 1; j < ends.size(); ++j)
        {
            if (used[j]) { continue; }
            if (std::fabs(ends[i][0] - ends[j][0]) < tol &&
                std::fabs(ends[i][1] - ends[j][1]) < tol) {
                used[i] = used[j] = true;
                matched = true;
                break;
            }
        }
        if (!matched) {
            return false;       // dangling end -> open profile (a surface)
        }
    }
    return closed_loops > 0 || !ends.empty();
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

    // Resolve the length scale; target is the project's metre convention.
    // A forced scale (SetUnitScale with s > 0) wins over the file's
    // declared unit -- the "set explicitly to force a scale" contract.
    // Otherwise derive it from length_unit.
    if (m_forced_scale > 0.0) {
        m_unit_scale = m_forced_scale;
    } else {
        std::string unit = JStr(doc, "length_unit", sc::unit::Mm);
        if (unit == sc::unit::Mm) { m_unit_scale = 0.001; }
        else if (unit == sc::unit::Cm) { m_unit_scale = 0.01; }
        else if (unit == sc::unit::M) { m_unit_scale = 1.0; }
        else if (unit == sc::unit::In) { m_unit_scale = 0.0254; }
        else { m_unit_scale = 0.001; }   // unknown unit -> mm convention
    }
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

    // World-placed footprint of each reconstructed solid extrude, by feature
    // id. A later pattern (FtPtnFtr) carries its target only as a world-space
    // anchor; this lets the reader map that anchor back to the feature the
    // pattern copies, so it patterns that feature's tool (not the whole body).
    //
    // Stored as the sketch plane (origin + u/v axes, world mm) plus the
    // profile's LOCAL (u,v) bounding box. Matching projects the anchor onto
    // the plane and tests the 2D box -- correct for ANY plane orientation.
    // (The old anchor.xy-vs-local-box test only worked when the sketch lay on
    // world XY, so a pattern of a boss on a side plane -- e.g. Pattern2's
    // -Y-plane pins -- mis-targeted whichever XY feature happened to be near.)
    struct ExtrudeFootprint {
        uint32_t id;
        double   origin[3];
        double   udir[3];
        double   vdir[3];
        double   umin, umax, vmin, vmax;
        double   wc[3];           // world-space centre of the footprint, for
                                  // grouping co-located seeds a pattern copies
                                  // together (concentric pins -> one stepped
                                  // pin patterned as a unit).
    };
    std::vector<ExtrudeFootprint> extrude_xy;

    // Records which prior pattern consumed a given seed feature, keyed by the
    // seed's feature id -> the pattern's feature id. Lets a LATER pattern that
    // copies the same seed nest the prior pattern's RESULT (e.g. Pattern4 over
    // Pattern3's ring) instead of just the bare seed: the seed id is mapped to
    // the pattern so the outer pattern's Tool becomes the inner pattern, which
    // the Replayer lowers as linear_pattern(circular_pattern(seed)). Overwritten
    // by the latest consumer, so a reference resolves to the most recent prior
    // pattern (processed in feature order).
    std::unordered_map<uint32_t, uint32_t> seed_to_pattern;

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
                // A box primitive is a solid body root, so it becomes the
                // running body tip -- a following feature that dresses / fuses
                // onto the current body (e.g. a chamfer, whose arm bails when
                // running_solid_id==0) must see it. (Extrude / baked / pattern
                // set this too; the box branch was the gap that left a
                // box+chamfer part dropping the chamfer to opaque.)
                running_solid_id = id;
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
                // extrude) is left opaque. So is a ~ZERO-thickness extrude:
                // its End E - Start S is ~0 (e.g. Extrude21, whose params fail
                // to read at the rolled history state -- _diag.data_rc=-10000,
                // field_count=-1 -- so fld 2/3 default to 0). A zero-distance
                // prism throws in OCCT (StdFail) and one throw aborts the whole
                // replay -- it crashed the editor before this guard. Staying
                // opaque is the safe outcome. (|E - S| is the RAW depth in the
                // file's length unit; 1e-4 mm is far below any real extrude.)
                auto prof = jf.find("profile");
                if (zt == "FtAllExt" && prof != jf.end() &&
                    prof->contains("curves") &&
                    ProfileExtrudable(prof->at("curves")) &&
                    std::fabs(FieldValueById(jf, 3, 0.0) -
                              FieldValueById(jf, 2, 0.0)) > 1e-4)
                {
                    const uint32_t sketch_fid = 1000000u + id;

                    // Start S (fld 2) / End E (fld 3): extrude limits measured
                    // ALONG the sketch normal from the sketch plane. The solid
                    // spans [S, E]; the profile curves sit ON the plane
                    // (offset 0), so a non-zero S means the extrude starts OFF
                    // the plane. Shift the sketch plane by S along its normal
                    // so the prism begins at S, then extrude the remaining
                    // |E - S|. (Ignoring S put every such boss |S| too low --
                    // Extrude3_Boss landed at z[0, 9.78] instead of its true
                    // [6.05, 15.83], the missing 4.55 being Start S.)
                    double startS = FieldValueById(jf, 2, 0.0);
                    double endE   = FieldValueById(jf, 3, 0.0);

                    cadapp::SketchIR sk;
                    sk.name = name + ":profile";
                    BuildSketchFromProfile(*prof, s, sketch_fid, sk);
                    const double startS_m = startS * s;
                    sk.plane_origin[0] += startS_m * sk.plane_normal[0];
                    sk.plane_origin[1] += startS_m * sk.plane_normal[1];
                    sk.plane_origin[2] += startS_m * sk.plane_normal[2];
                    out.sketches.push_back(std::move(sk));

                    cadapp::FeatPayloadSketch spl;
                    spl.sketch_id = sketch_fid;
                    cadapp::FeatureIR sf;
                    sf.id   = sketch_fid;
                    sf.type = cadapp::FeatType::Sketch;
                    sf.name = name + ":profile";
                    sf.data = std::move(spl);
                    out.features.push_back(std::move(sf));

                    // Extrude the remaining span; E below S grows the solid
                    // the other way along the normal (flip).
                    double depth  = std::fabs(endE - startS) * s;

                    cadapp::FeatPayloadExtrude epl;
                    epl.sketch_id      = sketch_fid;
                    epl.distance       = depth;
                    epl.end_type       = cadapp::ExtrudeEndType::Blind;
                    epl.flip_direction = (endE < startS);

                    // fld 14 = ZW3D boolean combine: 0 = new/base, 1 = add
                    // (boss), 2 = remove (cut). A cut rebuilt as a boss just
                    // fuses material already inside the body -> "no visible
                    // effect"; route fld14==2 to CutExtrude so it subtracts
                    // (the Replayer cuts the prism from the running body).
                    const bool is_cut =
                        std::fabs(FieldValueById(jf, 14, 0.0) - 2.0) < 0.5;

                    cadapp::FeatureIR ef;
                    ef.id   = id;
                    ef.type = is_cut ? cadapp::FeatType::CutExtrude
                                     : cadapp::FeatType::BossExtrude;
                    ef.name = name;
                    ef.data = std::move(epl);

                    // Wire the running body (imported / prior solid) as the
                    // Base input: a boss fuses the prism onto it, a cut
                    // subtracts the prism from it (the Replayer picks the
                    // boolean by feat type). A boss with no running body
                    // stands alone; a cut needs one (else the Replayer errs).
                    if (running_solid_id != 0) {
                        PushInput(ef, running_solid_id, cadapp::InputRole::Base);
                    }

                    // Record this extrude's world-placed footprint (plane +
                    // local u/v box, mm) so a later pattern can map its world
                    // anchor back to this feature -- see ExtrudeFootprint.
                    {
                        double umn = 1e300, umx = -1e300, vmn = 1e300, vmx = -1e300;
                        for (const auto& c : prof->at("curves")) {
                            for (const char* key : { "p0", "p1", "center" }) {
                                auto p = c.find(key);
                                if (p != c.end() && p->is_array() && p->size() >= 2) {
                                    double u = p->at(0).get<double>();
                                    double v = p->at(1).get<double>();
                                    if (u < umn) umn = u;
                                    if (u > umx) umx = u;
                                    if (v < vmn) vmn = v;
                                    if (v > vmx) vmx = v;
                                }
                            }
                        }
                        if (umn <= umx) {
                            ExtrudeFootprint fp;
                            fp.id = id;
                            // Plane frame in world mm (unscaled, to match the
                            // pattern anchor, which is world mm). v = normal x u
                            // (right-handed) -- the same frame the profile's
                            // (u,v) curve coords are authored in.
                            double o[3]  = { 0, 0, 0 };
                            double ud[3] = { 1, 0, 0 };
                            double nd[3] = { 0, 0, 1 };
                            auto pl = prof->find("plane");
                            if (pl != prof->end() && pl->is_object()) {
                                auto rd3 = [&](const char* key, double d[3]) {
                                    auto a = pl->find(key);
                                    if (a != pl->end() && a->is_array() && a->size() == 3) {
                                        d[0] = a->at(0).get<double>();
                                        d[1] = a->at(1).get<double>();
                                        d[2] = a->at(2).get<double>();
                                    }
                                };
                                rd3("origin", o);
                                rd3("x_dir",  ud);
                                rd3("normal", nd);
                            }
                            double vd[3] = {
                                nd[1] * ud[2] - nd[2] * ud[1],
                                nd[2] * ud[0] - nd[0] * ud[2],
                                nd[0] * ud[1] - nd[1] * ud[0],
                            };
                            for (int k = 0; k < 3; ++k) {
                                fp.origin[k] = o[k];
                                fp.udir[k]   = ud[k];
                                fp.vdir[k]   = vd[k];
                            }
                            fp.umin = umn; fp.umax = umx;
                            fp.vmin = vmn; fp.vmax = vmx;
                            // World centre of the footprint (origin + centre in
                            // the plane frame), for co-location grouping.
                            double cu = 0.5 * (umn + umx);
                            double cv = 0.5 * (vmn + vmx);
                            for (int k = 0; k < 3; ++k) {
                                fp.wc[k] = o[k] + cu * ud[k] + cv * vd[k];
                            }
                            extrude_xy.push_back(fp);
                        }
                    }

                    out.features.push_back(std::move(ef));
                    running_solid_id = id;
                    continue;
                }

                // ZW3D revolve (FtAllRev) -> Sketch + BossRevolve / CutRevolve.
                // The profile is captured exactly like an extrude's
                // (profile.curves on profile.plane); the difference is the boss
                // is swept around an AXIS instead of along the plane normal.
                // fld 2 "Axis A" gives only a POINT on the axis -- the plugin
                // does not export the axis direction. Recover it the way
                // SwReader does: the revolve axis is a LINE in the profile
                // sketch, so the profile line whose support passes through the
                // fld 2 point IS the axis, and its direction is the rotation
                // axis. (On R2900_50 the fld 2 point lies exactly on one profile
                // line, dir (0,1,0); the exported solid's surface placements all
                // share that axis -- STEP-cylinder, circular-pattern-orbit, and
                // profile-line fits independently agree.) The sketch is NOT
                // shifted (a revolve profile stays in its own plane). fld 3
                // Start / fld 4 End angle (deg) -> |E - S| radians; a 360 sweep
                // becomes exactly 2*PI so the Replayer closes the solid. fld 14
                // boolean combine: 2 = cut -> CutRevolve, else BossRevolve. When
                // no profile line carries the axis point (the axis is a datum we
                // cannot resolve reader-side), fall through to opaque.
                if (zt == "FtAllRev" && prof != jf.end() &&
                    prof->contains("curves") && prof->at("curves").is_array())
                {
                    double axpt[3] = { 0.0, 0.0, 0.0 };
                    if (FieldPoint(jf, 2, axpt))
                    {
                        // World plane frame (raw mm, to match axpt) from
                        // profile.plane; v = normal x x_dir (right-handed) is
                        // the frame the profile's (u,v) curve coords live in.
                        double o[3]  = { 0, 0, 0 };
                        double ud[3] = { 1, 0, 0 };
                        double nd[3] = { 0, 0, 1 };
                        auto pl = prof->find("plane");
                        if (pl != prof->end() && pl->is_object()) {
                            auto rd3 = [&](const char* key, double d[3]) {
                                auto a = pl->find(key);
                                if (a != pl->end() && a->is_array() &&
                                    a->size() == 3) {
                                    d[0] = a->at(0).get<double>();
                                    d[1] = a->at(1).get<double>();
                                    d[2] = a->at(2).get<double>();
                                }
                            };
                            rd3("origin", o);
                            rd3("x_dir",  ud);
                            rd3("normal", nd);
                        }
                        double vd[3] = {
                            nd[1] * ud[2] - nd[2] * ud[1],
                            nd[2] * ud[0] - nd[0] * ud[2],
                            nd[0] * ud[1] - nd[1] * ud[0],
                        };

                        // Axis direction = the profile line whose infinite
                        // support passes closest to the fld 2 axis point.
                        double best_d = 1e300;
                        double axis_dir[3] = { 0.0, 0.0, 0.0 };
                        for (const auto& c : prof->at("curves")) {
                            if (JStr(c, "kind", "") != "line") { continue; }
                            auto p0 = c.find("p0");
                            auto p1 = c.find("p1");
                            if (p0 == c.end() || p1 == c.end() ||
                                !p0->is_array() || !p1->is_array() ||
                                p0->size() < 2 || p1->size() < 2) { continue; }
                            double w0[3], w1[3];
                            for (int k = 0; k < 3; ++k) {
                                w0[k] = o[k] + p0->at(0).get<double>() * ud[k]
                                             + p0->at(1).get<double>() * vd[k];
                                w1[k] = o[k] + p1->at(0).get<double>() * ud[k]
                                             + p1->at(1).get<double>() * vd[k];
                            }
                            double dd[3] = { w1[0]-w0[0], w1[1]-w0[1], w1[2]-w0[2] };
                            double L = std::sqrt(dd[0]*dd[0] + dd[1]*dd[1] +
                                                 dd[2]*dd[2]);
                            if (L < 1e-9) { continue; }
                            double du[3] = { dd[0]/L, dd[1]/L, dd[2]/L };
                            double ap[3] = { axpt[0]-w0[0], axpt[1]-w0[1],
                                             axpt[2]-w0[2] };
                            double t = ap[0]*du[0] + ap[1]*du[1] + ap[2]*du[2];
                            double ft[3] = { w0[0]+t*du[0], w0[1]+t*du[1],
                                             w0[2]+t*du[2] };
                            double pd = std::sqrt(
                                (axpt[0]-ft[0])*(axpt[0]-ft[0]) +
                                (axpt[1]-ft[1])*(axpt[1]-ft[1]) +
                                (axpt[2]-ft[2])*(axpt[2]-ft[2]));
                            if (pd < best_d) {
                                best_d = pd;
                                axis_dir[0] = du[0];
                                axis_dir[1] = du[1];
                                axis_dir[2] = du[2];
                            }
                        }

                        double startA = FieldValueById(jf, 3, 0.0);
                        double endA   = FieldValueById(jf, 4, 0.0);
                        const double kPi = 3.14159265358979323846;
                        double angle  = std::fabs(endA - startA) * kPi / 180.0;

                        // Accept only when a profile line truly carries the axis
                        // point (~1e-3 mm) and the sweep is non-degenerate.
                        if (best_d < 1e-3 && angle > 1e-6)
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

                            cadapp::FeatPayloadRevolve rpl;
                            rpl.sketch_id      = sketch_fid;
                            rpl.axis_origin[0] = axpt[0] * s;
                            rpl.axis_origin[1] = axpt[1] * s;
                            rpl.axis_origin[2] = axpt[2] * s;
                            rpl.axis_dir[0]    = axis_dir[0];
                            rpl.axis_dir[1]    = axis_dir[1];
                            rpl.axis_dir[2]    = axis_dir[2];
                            rpl.angle          = angle;

                            const bool is_cut =
                                std::fabs(FieldValueById(jf, 14, 0.0) - 2.0) < 0.5;
                            cadapp::FeatureIR rf;
                            rf.id   = id;
                            rf.name = name;
                            rf.type = is_cut ? cadapp::FeatType::CutRevolve
                                             : cadapp::FeatType::BossRevolve;
                            rf.data = std::move(rpl);
                            if (running_solid_id != 0) {
                                PushInput(rf, running_solid_id,
                                          cadapp::InputRole::Base);
                            }
                            out.features.push_back(std::move(rf));
                            running_solid_id = id;
                            continue;
                        }
                    }
                }

                // ZW3D pattern (FtPtnFtr) -> LinearPattern. count (fld 3) /
                // spacing (fld 4) / target anchor (fld 1) are in the field
                // dump; the direction is the unit vector the plugin resolved
                // from the pattern's referenced edge ("pattern":{"dir":[...]}).
                // Map the target anchor to the reconstructed extrude it copies,
                // pattern that feature's tool, and combine the instances onto
                // the CURRENT body (ext_param pattern_onto_running; the
                // Replayer otherwise fuses onto the patterned feature's own
                // base, dropping any feature between it and the pattern, e.g.
                // Extrude4's cut).
                {
                    // ZW3D FtPtnFtr is a GENERIC pattern feature. fld 26 is NOT
                    // the method enum -- it is the pattern's creation ORDINAL
                    // (1,2,3,... across the part). On R2900_20/_30 the first
                    // patterns were linear,linear,circular, so the ordinals
                    // 1,2,3 happened to look like a method code; R2900_50 has
                    // seven patterns numbered 1..7, exposing fld 26 as a plain
                    // counter (there is no "method 7"). The real discriminator
                    // is fld 10 (0 = linear, 1 = circular); fld 28 "Diameter"
                    // (the pattern circle's diameter, > 0 only for circular)
                    // agrees and is used as a backstop. fld 12 carries the real
                    // per-step angle for a circular pattern; for a linear one it
                    // is a stale default.
                    //   LINEAR: pattern.dir is a TRANSLATION axis ->
                    //     LinearPattern; the co-located seed group is patterned
                    //     as a unit (concentric pins copied together).
                    //   CIRCULAR: the rotation axis is the seed's sketch-plane
                    //     NORMAL (the pattern lies in that plane) through the
                    //     fld 2 cached pick point (which sits ON the axis).
                    //     pattern.dir is only a parallel-offset reference edge
                    //     and is NOT used here. Only the single matched seed is
                    //     swept. fld 12 = per-instance angle, fld 3 = count
                    //     (incl. the seed). Verified against R2900_30/_50 by
                    //     circle-fitting the exported solid's pins.
                    // A circular pattern with no axis pick point stays OPAQUE.
                    auto patj = jf.find("pattern");
                    bool   is_circular     = FieldValueById(jf, 10, 0.0) > 0.5 ||
                                             FieldValueById(jf, 28, 0.0) > 1e-6;
                    bool   linear_method   = !is_circular;
                    bool   circular_method = is_circular;
                    double axis_pt[3] = { 0.0, 0.0, 0.0 };
                    bool   has_axis_pt = FieldPoint(jf, 2, axis_pt);
                    double fdir_tmp[3];
                    bool   has_dir = (patj != jf.end() &&
                                      patj->contains("dir") &&
                                      patj->at("dir").is_array() &&
                                      patj->at("dir").size() == 3) ||
                                     FieldDir(jf, 2, fdir_tmp);
                    if (zt == "FtPtnFtr" &&
                        ((linear_method && has_dir) ||
                         (circular_method && has_axis_pt)) &&
                        running_solid_id != 0 && !extrude_xy.empty())
                    {
                        // Match a world-mm anchor to a reconstructed extrude
                        // footprint: project onto each extrude's sketch plane and
                        // test the profile's local (u,v) box (the out-of-plane
                        // offset -- anchor at the feature's top face, profile at
                        // its base -- drops out, so a boss on ANY plane matches,
                        // not just world-XY). Returns 0 when the best hit is
                        // implausible (anchor far from every footprint CENTRE):
                        // the +1e6 keeps an outside hit last, and even an "inside"
                        // hit is trusted only within kSeedCentreTol of the centre.
                        // A pattern whose true seed is an UNreconstructed feature
                        // (a thin boss beside the opaque revolve) otherwise lands
                        // "inside" some large unrelated boss's box ~15 mm
                        // off-centre (R2900_50's Pattern6) and would replicate the
                        // WRONG body -- staying opaque (a clean warning) beats
                        // silently wrong geometry. Real seeds match within a
                        // fraction of a mm, so 5 mm is ample.
                        auto match_anchor = [&](const double a[3]) -> uint32_t {
                            double best = 1e300;
                            uint32_t hit = 0;
                            for (const auto& b : extrude_xy) {
                                double du[3] = { a[0] - b.origin[0],
                                                 a[1] - b.origin[1],
                                                 a[2] - b.origin[2] };
                                double u = du[0]*b.udir[0] + du[1]*b.udir[1] + du[2]*b.udir[2];
                                double v = du[0]*b.vdir[0] + du[1]*b.vdir[1] + du[2]*b.vdir[2];
                                bool inside = u >= b.umin - 1.0 && u <= b.umax + 1.0 &&
                                              v >= b.vmin - 1.0 && v <= b.vmax + 1.0;
                                double cu = 0.5 * (b.umin + b.umax);
                                double cv = 0.5 * (b.vmin + b.vmax);
                                double d2 = (u - cu) * (u - cu) + (v - cv) * (v - cv);
                                double score = inside ? d2 : (d2 + 1e6);
                                if (score < best) { best = score; hit = b.id; }
                            }
                            const double kSeedCentreTol = 5.0;   // mm
                            return (best > kSeedCentreTol * kSeedCentreTol) ? 0u : hit;
                        };

                        // First Base (fld 1) ent -> seed. Drives the gate and,
                        // for a circular pattern, the rotation axis (taken from
                        // this one footprint's normal).
                        double anc[3] = { 0.0, 0.0, 0.0 };
                        uint32_t target = 0;
                        if (FieldEntAnchor(jf, 1, anc)) {
                            target = match_anchor(anc);
                        }

                        // A LINEAR pattern's Base may list SEVERAL entities at
                        // different places (e.g. a boss and the cut that grooves
                        // it -- ~13 mm apart on R2900_50's Pattern7). Matching
                        // only the first ent + spatial grouping drops the rest, so
                        // the pattern replicates an incomplete seed (Pattern7
                        // patterned the lone cut -> nothing to carve -> no visible
                        // effect). Match EVERY fld 1 ent (guarded, deduped). This
                        // also lets a pattern whose first ent is unreconstructed
                        // (Pattern6's first ents sit on the revolve) still rebuild
                        // from its remaining ents. (Circular keeps the first-ent
                        // seed above -- its axis needs that one footprint.)
                        std::vector<uint32_t> seed_ids;
                        std::vector<uint32_t> consumed_seeds;   // raw seeds (pre-
                                                                // nesting) this
                                                                // pattern copies
                        if (linear_method) {
                            auto add_seed = [&](uint32_t t) {
                                if (t == 0) { return; }
                                for (uint32_t e : seed_ids) { if (e == t) return; }
                                seed_ids.push_back(t);
                            };
                            auto fit = jf.find("fields");
                            if (fit != jf.end() && fit->is_array()) {
                                for (const auto& fd : *fit) {
                                    auto idit = fd.find("id");
                                    if (idit == fd.end() || !idit->is_number() ||
                                        idit->get<int>() != 1) { continue; }
                                    auto eit = fd.find("ents");
                                    if (eit != fd.end() && eit->is_array()) {
                                        for (const auto& e : *eit) {
                                            // Prefer the OWNING feature id the
                                            // plugin resolved (cvxPartInqEntFtr,
                                            // emitted as "feat"): it names the
                                            // exact base child -- including a
                                            // prior PATTERN (Pattern4's Base lists
                                            // Pattern3, which a sub-mm cluster of
                                            // anchors can't disambiguate). Fall
                                            // back to geometric anchor matching
                                            // for older snapshots with no "feat".
                                            auto ft = e.find("feat");
                                            if (ft != e.end() &&
                                                ft->is_number_integer()) {
                                                add_seed(static_cast<uint32_t>(
                                                    ft->get<int>()));
                                                continue;
                                            }
                                            auto a = e.find("anchor");
                                            if (a == e.end() || !a->is_array() ||
                                                a->size() < 3) { continue; }
                                            double ea[3] = {
                                                a->at(0).get<double>(),
                                                a->at(1).get<double>(),
                                                a->at(2).get<double>() };
                                            add_seed(match_anchor(ea));
                                        }
                                    }
                                    break;
                                }
                            }

                            consumed_seeds = seed_ids;   // record raw, pre-nesting

                            // NEST a prior pattern: if a seed was itself patterned
                            // earlier (Pattern3 ringed Extrude11), copy that
                            // pattern's RESULT, not the bare seed -- map the seed
                            // to the pattern. The Replayer lowers the pattern-as-
                            // tool to linear_pattern(circular_pattern(seed)), so
                            // the whole ring replicates. Only maps to a pattern
                            // created BEFORE this one (its result is in the body).
                            for (uint32_t& sd : seed_ids) {
                                auto sp = seed_to_pattern.find(sd);
                                if (sp != seed_to_pattern.end() &&
                                    sp->second < id) {
                                    sd = sp->second;
                                }
                            }
                            // Re-dedup: two seeds may map to the same pattern.
                            {
                                std::vector<uint32_t> uniq;
                                for (uint32_t sd : seed_ids) {
                                    bool seen = false;
                                    for (uint32_t u : uniq) {
                                        if (u == sd) { seen = true; break; }
                                    }
                                    if (!seen) { uniq.push_back(sd); }
                                }
                                seed_ids.swap(uniq);
                            }

                            if (target == 0 && !seed_ids.empty()) {
                                target = seed_ids.front();
                            }
                        }

                        if (target != 0)
                        {
                            cadapp::FeatureIR pf;
                            pf.id   = id;
                            pf.name = name;

                            if (linear_method)
                            {
                                cadapp::FeatPayloadLinearPattern lp;
                                // Direction: prefer fld 2 "Direction"'s TRUE
                                // signed Dir over the edge-derived pattern.dir,
                                // whose sign -- and even axis -- follow the
                                // reference edge's arbitrary orientation (it sent
                                // R2900_50's Pattern6 +X->-X and Pattern7
                                // +Z->-Z). fld 2's Dir is correct for every
                                // pattern (P1 +Y, P2/P4 -X, P6 +X, P7 +Z); fld 5
                                // "Direction D" is a DIFFERENT, wrong vector --
                                // do NOT use it. Fall back to the edge dir only
                                // for older snapshots that carry no field Dir.
                                double pdir[3];
                                if (FieldDir(jf, 2, pdir)) {
                                    lp.dir1[0] = pdir[0];
                                    lp.dir1[1] = pdir[1];
                                    lp.dir1[2] = pdir[2];
                                } else {
                                    lp.dir1[0] = patj->at("dir").at(0).get<double>();
                                    lp.dir1[1] = patj->at("dir").at(1).get<double>();
                                    lp.dir1[2] = patj->at("dir").at(2).get<double>();
                                }
                                int count1  = static_cast<int>(FieldValueById(jf, 3, 2.0));
                                lp.count1   = (count1 >= 1) ? count1 : 2;
                                lp.spacing1 = FieldValueById(jf, 4, 0.0) * s;
                                int count2  = static_cast<int>(FieldValueById(jf, 6, 1.0));
                                lp.count2   = (count2 >= 1) ? count2 : 1;
                                lp.spacing2 = FieldValueById(jf, 7, 0.0) * s;
                                pf.type = cadapp::FeatType::LinearPattern;
                                pf.data = std::move(lp);

                                // Tools = every matched Base seed (seed_ids),
                                // each expanded to the seeds co-located with it
                                // (concentric pins copied as a unit), deduped so
                                // overlapping groups don't double-count. Base =
                                // the current running body.
                                std::vector<uint32_t> tool_ids;
                                auto add_tool = [&](uint32_t t) {
                                    for (uint32_t e : tool_ids) { if (e == t) return; }
                                    tool_ids.push_back(t);
                                };
                                const double kGroupTol = 1.5;   // mm (concentric
                                                                // seeds sit ~0 apart)
                                for (uint32_t seed : seed_ids) {
                                    const double* sw = nullptr;
                                    for (const auto& b : extrude_xy) {
                                        if (b.id == seed) { sw = b.wc; break; }
                                    }
                                    if (!sw) { add_tool(seed); continue; }
                                    for (const auto& b : extrude_xy) {
                                        double dx = b.wc[0] - sw[0];
                                        double dy = b.wc[1] - sw[1];
                                        double dz = b.wc[2] - sw[2];
                                        if (dx*dx + dy*dy + dz*dz <= kGroupTol*kGroupTol) {
                                            add_tool(b.id);
                                        }
                                    }
                                }
                                if (tool_ids.empty()) { add_tool(target); }
                                for (uint32_t t : tool_ids) {
                                    PushInput(pf, t, cadapp::InputRole::Tool);
                                }
                            }
                            else   // circular_method (gated above on has_axis_pt)
                            {
                                cadapp::FeatPayloadCircularPattern cp;
                                // Axis DIRECTION = the seed's sketch-plane normal
                                // (= udir x vdir of the matched footprint). The
                                // pattern lies IN that plane, so the plane normal
                                // is the rotation axis. pattern.dir (a reference
                                // edge, parallel-offset & oblique) is NOT used.
                                double nx = 0.0, ny = 0.0, nz = 1.0;
                                for (const auto& b : extrude_xy) {
                                    if (b.id == target) {
                                        nx = b.udir[1]*b.vdir[2] - b.udir[2]*b.vdir[1];
                                        ny = b.udir[2]*b.vdir[0] - b.udir[0]*b.vdir[2];
                                        nz = b.udir[0]*b.vdir[1] - b.udir[1]*b.vdir[0];
                                        break;
                                    }
                                }
                                double nl = std::sqrt(nx*nx + ny*ny + nz*nz);
                                if (nl < 1e-9) { nx = 0.0; ny = 0.0; nz = 1.0; nl = 1.0; }
                                cp.axis_dir[0] = nx / nl;
                                cp.axis_dir[1] = ny / nl;
                                cp.axis_dir[2] = nz / nl;
                                // fld 2 pick point lies ON the axis; it is a
                                // POSITION -> raw*s space (dir is unit, unscaled).
                                cp.axis_origin[0] = axis_pt[0] * s;
                                cp.axis_origin[1] = axis_pt[1] * s;
                                cp.axis_origin[2] = axis_pt[2] * s;
                                int cnt = static_cast<int>(FieldValueById(jf, 3, 2.0));
                                cp.count = (cnt >= 1) ? cnt : 2;
                                // fld 12 = per-instance angle (deg). TopoAlgo's
                                // CircularPattern uses step = total_angle / count,
                                // so feed step * count.
                                double step_deg = FieldValueById(jf, 12, 0.0);
                                cp.total_angle  = step_deg * cp.count
                                                * 3.14159265358979323846 / 180.0;
                                // Push each copy ~20um into the body so the thin
                                // coplanar boss base becomes a clean volumetric
                                // overlap (mm-scale OCCT will not merge a pure
                                // coplanar contact). 0.02 raw-mm in the geometry's
                                // raw*s units; well under a typical boss depth.
                                cp.penetration = 0.02 * s;
                                pf.type = cadapp::FeatType::CircularPattern;
                                pf.data = std::move(cp);

                                // Sweep ONLY the matched seed (one feature).
                                PushInput(pf, target, cadapp::InputRole::Tool);
                                consumed_seeds = { target };
                            }

                            PushInput(pf, running_solid_id, cadapp::InputRole::Base);
                            pf.ext_params["pattern_onto_running"] = 1.0;

                            // Record which pattern consumed each raw seed so a
                            // LATER pattern copying the same seed nests THIS
                            // pattern's result (the ring), not the bare seed.
                            for (uint32_t sd : consumed_seeds) {
                                if (sd != 0) { seed_to_pattern[sd] = id; }
                            }

                            out.features.push_back(std::move(pf));
                            running_solid_id = id;
                            continue;
                        }
                    }
                }

                // ZW3D edge dress-up: chamfer (FtChamfers2) -> FeatPayloadChamfer
                // and fillet/round (FtFillet2) -> FeatPayloadFillet. Both store
                // the picked edge(s) the same way: the plugin captured them from
                // the feature's nested VDATA edge-list as world-mm anchors with a
                // per-edge dimension (fld "ents":[{anchor,num}]) -- num is the
                // chamfer SETBACK for FtChamfers2 and the fillet RADIUS for
                // FtFillet2 (R2900_100: chamfer num=0.2 mm, fillet num=1.0/0.2 mm
                // -- it varies per fillet, confirming num is the radius) -- plus a
                // feature-level "input_edges" fallback (anchor only). The cax
                // Replayer applies the dress-up itself: a resolve_edge_ref per
                // anchor (matched to the running body's edge within ~5x the
                // dimension) feeding a "fillet" or "chamfer" op -- so there is no
                // loader/OCCT work here, only the IR mapping. The Replayer picks
                // the op purely from the payload variant (FeatPayloadFillet vs
                // FeatPayloadChamfer), so emitting the right one is all it takes.
                if ((zt == "FtChamfers2" || zt == "FtFillet2") &&
                    running_solid_id != 0)
                {
                    const bool is_fillet = (zt == "FtFillet2");

                    struct Pick { double anc[3] = { 0, 0, 0 };
                                  double num = 0.0; bool has_num = false; };
                    std::vector<Pick> picks;
                    auto read_ents = [&](const json& ents)
                    {
                        for (const auto& e : ents)
                        {
                            auto a = e.find("anchor");
                            if (a == e.end() || !a->is_array() || a->size() < 3) {
                                continue;
                            }
                            Pick pk;
                            pk.anc[0] = a->at(0).get<double>();
                            pk.anc[1] = a->at(1).get<double>();
                            pk.anc[2] = a->at(2).get<double>();
                            auto n = e.find("num");
                            if (n != e.end() && n->is_number()) {
                                pk.num     = n->get<double>();
                                pk.has_num = true;
                            }
                            picks.push_back(pk);
                        }
                    };

                    // Prefer the field ents (they carry the per-edge dimension);
                    // fall back to the feature-level input_edges.
                    auto fit2 = jf.find("fields");
                    if (fit2 != jf.end() && fit2->is_array()) {
                        for (const auto& fd : *fit2) {
                            auto eit = fd.find("ents");
                            if (eit != fd.end() && eit->is_array() && !eit->empty()) {
                                read_ents(*eit);
                            }
                        }
                    }
                    if (picks.empty()) {
                        auto iit = jf.find("input_edges");
                        if (iit != jf.end() && iit->is_array()) {
                            read_ents(*iit);
                        }
                    }

                    // Dimension (chamfer setback / fillet radius): first captured
                    // per-edge value, else the feature scalar -- chamfer distance
                    // is fld 3, fillet radius rides on the edge-list field fld 2
                    // (fld 3 "Relief" is 0 for a plain fillet). A 0-size dress-up
                    // would just error in OCCT, so fall through to opaque when
                    // neither source yields a positive value.
                    double dim = 0.0;
                    for (const auto& pk : picks) {
                        if (pk.has_num && pk.num > 0.0) { dim = pk.num; break; }
                    }
                    if (dim <= 0.0) {
                        dim = FieldValueById(jf, is_fillet ? 2 : 3, 0.0);
                    }

                    if (!picks.empty() && dim > 0.0)
                    {
                        std::vector<cadapp::TopoRefIR> edges;
                        for (const auto& pk : picks)
                        {
                            cadapp::TopoRefIR r;
                            r.kind     = cadapp::TopoRefIR::Kind::Edge;
                            r.point[0] = pk.anc[0] * s;
                            r.point[1] = pk.anc[1] * s;
                            r.point[2] = pk.anc[2] * s;
                            edges.push_back(r);
                        }

                        cadapp::FeatureIR cf;
                        cf.id   = id;
                        cf.name = name;
                        cf.ext_strings["zw_type"] = zt;
                        if (is_fillet)
                        {
                            cadapp::FeatPayloadFillet pl;
                            pl.radius = dim * s;
                            pl.edges  = std::move(edges);
                            cf.type   = cadapp::FeatType::Fillet;
                            cf.data   = std::move(pl);
                        }
                        else
                        {
                            cadapp::FeatPayloadChamfer pl;
                            pl.distance1 = dim * s;
                            pl.distance2 = 0.0;      // symmetric (fld 42 Type=0)
                            pl.edges     = std::move(edges);
                            cf.type      = cadapp::FeatType::Chamfer;
                            cf.data      = std::move(pl);
                        }
                        // Base = the running body the dress-up modifies; the
                        // Replayer resolves the picked edges against it.
                        PushInput(cf, running_solid_id, cadapp::InputRole::Base);
                        out.features.push_back(std::move(cf));
                        running_solid_id = id;
                        continue;
                    }
                }

                // ZW3D mirror (FtMirrorFtr) -> FeatPayloadMirror. fld 1
                // "Feature" lists the mirrored features -- each ent carries the
                // owning feature's JSON id in "feat", the SAME id the reader keys
                // features by, so they wire straight as Role::Tool originals. fld
                // 2 "Plane" is the mirror plane: its ent's "anchor" is a point ON
                // the plane and "normal" its orientation. The plugin emits that
                // normal for a datum-plane ent via ZwEntityMatrixGet; a snapshot
                // exported BEFORE that plugin change carries no normal and cannot
                // define the reflection -> the feature stays opaque. The Replayer
                // (AddMirroredOriginals) mirrors each built original's tool solid
                // across the plane and fuses onto the running body.
                //
                // GUARD -- avoid a silently-wrong whole-body mirror: when NONE of
                // a mirror's originals were reconstructed (e.g. R2900_100's
                // Mirror2 targets the two opaque face-derived extrudes 30/31),
                // ResolvePatternInputs returns no originals and the Replayer's
                // mirror arm falls back to mirroring the ENTIRE body across the
                // plane -- duplicating the whole part. So only emit a Mirror when
                // at least one mirrored feature actually built; otherwise leave it
                // opaque (an honest warning beats garbage geometry). NOTE: a
                // dressup (fillet/chamfer) among the originals is NOT re-applied by
                // AddMirroredOriginals (it mirrors tool solids, not dressup ops),
                // so a mirror over a filleted region is reconstructed without those
                // blends -- a known partial-fidelity limitation.
                if (zt == "FtMirrorFtr" && running_solid_id != 0)
                {
                    auto fields_it = jf.find("fields");
                    const bool has_fields =
                        (fields_it != jf.end() && fields_it->is_array());

                    // Mirror plane: first fld 2 ent carrying a non-zero normal.
                    double porg[3] = { 0.0, 0.0, 0.0 };
                    double pnrm[3] = { 0.0, 0.0, 0.0 };
                    bool   has_plane = false;
                    if (has_fields) {
                        for (const auto& fd : *fields_it) {
                            if (JGet<int>(fd, "id", -1) != 2) { continue; }
                            auto eit = fd.find("ents");
                            if (eit != fd.end() && eit->is_array()) {
                                for (const auto& e : *eit) {
                                    auto nm = e.find("normal");
                                    auto an = e.find("anchor");
                                    if (nm == e.end() || !nm->is_array() || nm->size() < 3 ||
                                        an == e.end() || !an->is_array() || an->size() < 3) {
                                        continue;
                                    }
                                    double nx = nm->at(0).get<double>();
                                    double ny = nm->at(1).get<double>();
                                    double nz = nm->at(2).get<double>();
                                    if (std::fabs(nx) + std::fabs(ny) + std::fabs(nz) < 1e-9) {
                                        continue;
                                    }
                                    porg[0] = an->at(0).get<double>();
                                    porg[1] = an->at(1).get<double>();
                                    porg[2] = an->at(2).get<double>();
                                    pnrm[0] = nx; pnrm[1] = ny; pnrm[2] = nz;
                                    has_plane = true;
                                    break;
                                }
                            }
                            break;
                        }
                    }

                    // Mirrored features: fld 1 ents' "feat" ids that the reader
                    // already reconstructed as a real (non-opaque) feature.
                    std::vector<uint32_t> mir_tools;
                    if (has_plane) {
                        auto is_built = [&](uint32_t fid) -> bool {
                            for (const auto& f : out.features) {
                                if (f.id == fid) {
                                    return f.type != cadapp::FeatType::Unknown;
                                }
                            }
                            return false;
                        };
                        for (const auto& fd : *fields_it) {
                            if (JGet<int>(fd, "id", -1) != 1) { continue; }
                            auto eit = fd.find("ents");
                            if (eit != fd.end() && eit->is_array()) {
                                for (const auto& e : *eit) {
                                    auto ft = e.find("feat");
                                    if (ft == e.end() || !ft->is_number_integer()) {
                                        continue;
                                    }
                                    uint32_t t = static_cast<uint32_t>(ft->get<int>());
                                    if (t == 0 || !is_built(t)) { continue; }
                                    bool dup = false;
                                    for (uint32_t x : mir_tools) { if (x == t) { dup = true; break; } }
                                    if (!dup) { mir_tools.push_back(t); }
                                }
                            }
                            break;
                        }
                    }

                    if (has_plane && !mir_tools.empty())
                    {
                        cadapp::FeatPayloadMirror pl;
                        // origin is positional (mm->IR units); normal is a unit
                        // direction and stays unscaled.
                        pl.plane_origin[0] = porg[0] * s;
                        pl.plane_origin[1] = porg[1] * s;
                        pl.plane_origin[2] = porg[2] * s;
                        double nl = std::sqrt(pnrm[0]*pnrm[0] + pnrm[1]*pnrm[1] +
                                              pnrm[2]*pnrm[2]);
                        if (nl < 1e-12) { nl = 1.0; }
                        pl.plane_normal[0] = pnrm[0] / nl;
                        pl.plane_normal[1] = pnrm[1] / nl;
                        pl.plane_normal[2] = pnrm[2] / nl;

                        cadapp::FeatureIR mf;
                        mf.id   = id;
                        mf.name = name;
                        mf.type = cadapp::FeatType::Mirror;
                        mf.data = std::move(pl);
                        mf.ext_strings["zw_type"] = zt;
                        // Tools = the mirrored originals; Base = running body the
                        // mirror images fuse onto (AddMirroredOriginals).
                        for (uint32_t t : mir_tools) {
                            PushInput(mf, t, cadapp::InputRole::Tool);
                        }
                        PushInput(mf, running_solid_id, cadapp::InputRole::Base);
                        out.features.push_back(std::move(mf));
                        running_solid_id = id;
                        continue;
                    }
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
    