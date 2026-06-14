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
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <set>
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

// Existence probe with the same dual-codepage decode OpenInputBinary uses
// (UTF-8 first, ANSI fallback), so a GBK path from argv and a UTF-8 path
// from a .ves scene string both resolve.
bool FileExists(const std::string& path)
{
    if (path.empty()) {
        return false;
    }
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
    std::wstring w = widen(path, CP_UTF8, MB_ERR_INVALID_CHARS);
    if (w.empty()) {
        w = widen(path, CP_ACP, 0);
    }
    if (w.empty()) {
        return false;
    }
    const DWORD attrs = ::GetFileAttributesW(w.c_str());
    return attrs != INVALID_FILE_ATTRIBUTES &&
           !(attrs & FILE_ATTRIBUTE_DIRECTORY);
#else
    std::ifstream probe(path, std::ios::binary);
    return (bool)probe;
#endif
}

// Resolve a feature's sibling STEP ("<part>.cax.feat<id>.step"). The
// exporter records it as a basename in the JSON, but a GBK part name that
// passed through the exporter's UTF-8 sanitiser comes back as U+FFFD
// mojibake -- the original bytes are unrecoverable and the recorded name
// matches no file. The sibling path is fully DERIVABLE from the snapshot's
// own path (same naming scheme, same directory), so use the recorded name
// when it resolves and fall back to the derived sibling otherwise.
std::string ResolveFeatStepPath(const std::string& doc_path,
                                const std::string& doc_dir,
                                const std::string& recorded,
                                uint32_t id)
{
    const std::string rec_path = doc_dir + recorded;
    if (FileExists(rec_path)) {
        return rec_path;
    }
    std::string base = doc_path;
    const std::string js = ".json";
    if (base.size() >= js.size())
    {
        std::string tail = base.substr(base.size() - js.size());
        for (auto& c : tail) {
            c = (char)std::tolower((unsigned char)c);
        }
        if (tail == js) {
            base.resize(base.size() - js.size());
        }
    }
    std::string derived = base + ".feat" + std::to_string(id) + ".step";
    if (FileExists(derived)) {
        return derived;
    }
    return rec_path;   // keep the recorded name for error reporting
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

// OWNING feature id ("feat", from cvxPartInqEntFtr) of the first entity of
// field fld_id. 0 when the field / ents / feat key is absent. The linear
// pattern branch already prefers this over geometric anchor matching; the
// circular branch needs it too -- R2900's Pattern13 carries an fld 1 anchor
// (a consumed-state bbox centre) 12 mm off its own seed's footprint, so
// match_anchor() rejects every candidate even though the dump names the
// seed feature outright.
uint32_t FieldEntFeat(const json& jf, int fld_id)
{
    auto fit = jf.find("fields");
    if (fit == jf.end() || !fit->is_array()) {
        return 0;
    }
    for (const auto& fd : *fit)
    {
        auto idit = fd.find("id");
        if (idit == fd.end() || !idit->is_number() || idit->get<int>() != fld_id) {
            continue;
        }
        auto eit = fd.find("ents");
        if (eit == fd.end() || !eit->is_array() || eit->empty()) {
            return 0;
        }
        auto ft = eit->at(0).find("feat");
        if (ft == eit->at(0).end() || !ft->is_number_integer()) {
            return 0;
        }
        int v = ft->get<int>();
        return (v > 0) ? static_cast<uint32_t>(v) : 0;
    }
    return 0;
}

// Build a SketchIR from a ZW3D extrude's built-in profile ("profile" block).
// The curves are in the sketch's LOCAL 2D frame (u,v with z==0 on the plane),
// so they map straight onto the SketchIR's 2D plane coords; the "plane" block
// (origin/x_dir/normal in world, from the plugin's ZwEntityMatrixGet) is what
// positions that frame in space. Absent a plane block (older snapshots, or a
// sketch drawn on world XY), the default is XY at z==0 -- which is why the
// base extrude, on z==0, was already correct without it.
// loops_only: build ONLY curves that lie on closed chains, dropping
// dangling strays. The extrude gate (ProfileHasClosedLoop) accepts a
// profile as soon as ONE closed region exists -- but an all-reference
// profile (02-ear 拉伸17: a region-picked r=0.9 circle next to two
// ±150 mm construction AXIS LINES) would otherwise feed the strays into
// the sketch, and WiresToFace fabricates faces over the phantom region
// (a 250 mm solid out of a 1.8 mm pin). Sweep SPINES are legitimately
// open chains -- their call site keeps loops_only=false.
void BuildSketchFromProfile(const json& profile, double s, uint32_t feature_id,
                            cadapp::SketchIR& sk, bool loops_only = false)
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
    // Reference / projected curves ("ref":true, from cvxSkInqRefById) are
    // tagged by the exporter and SKIPPED whenever drawn curves exist: a
    // sketch's reference set can hold construction junk alongside real
    // inner loops, and consuming it blindly is riskier than the known gap
    // (Extrude48's cutouts) it would close. A profile with NO drawn curves
    // at all (R2900_100 Extrude21-style reference-only profiles) keeps
    // using the ref curves -- they are the whole profile there.
    bool has_drawn = false;
    for (const auto& c : *cit)
    {
        if (!JGet<bool>(c, "ref", false)) { has_drawn = true; break; }
    }

    // Closed-chain mask (loops_only). Mirrors ProfileHasClosedLoop's
    // peel: circles/ellipses and self-closing segments are loops on
    // their own; line/arc segments survive only while both endpoints
    // touch another surviving segment. 1e-2 mm pairing tolerance --
    // same calibration as the gate (drawn arc-chains jitter ~3e-3 mm,
    // structural gaps are millimetres).
    const size_t      n_curves = cit->size();
    std::vector<bool> keep(n_curves, true);
    if (loops_only)
    {
        const double tol = 1e-2;
        auto near2 = [tol](const double* p, const double* q) {
            return std::fabs(p[0] - q[0]) < tol &&
                   std::fabs(p[1] - q[1]) < tol;
        };
        struct Seg { double a[2], b[2]; size_t idx; bool alive; };
        std::vector<Seg> segs;
        bool any_loop = false;
        for (size_t i = 0; i < n_curves; ++i)
        {
            const auto& c = cit->at(i);
            if (has_drawn && JGet<bool>(c, "ref", false)) { continue; }
            const std::string ck = JStr(c, "kind", "");
            if (ck == "circle" || ck == "ellipse") {
                any_loop = true;
                continue;                       // self-closed: keep
            }
            auto p0 = c.find("p0");
            auto p1 = c.find("p1");
            if (p0 == c.end() || p1 == c.end() ||
                !p0->is_array() || !p1->is_array() ||
                p0->size() < 2 || p1->size() < 2) {
                continue;                       // endpoint-less: leave as-is
            }
            Seg sg;
            sg.a[0] = p0->at(0).get<double>();
            sg.a[1] = p0->at(1).get<double>();
            sg.b[0] = p1->at(0).get<double>();
            sg.b[1] = p1->at(1).get<double>();
            sg.idx  = i;
            if (near2(sg.a, sg.b)) {            // full circle as one arc
                any_loop = true;
                continue;
            }
            sg.alive = true;
            segs.push_back(sg);
        }
        bool pruned = true;
        while (pruned)
        {
            pruned = false;
            for (auto& sg : segs)
            {
                if (!sg.alive) { continue; }
                int deg_a = 0, deg_b = 0;
                for (const auto& t : segs)
                {
                    if (!t.alive || &t == &sg) { continue; }
                    if (near2(sg.a, t.a) || near2(sg.a, t.b)) { ++deg_a; }
                    if (near2(sg.b, t.a) || near2(sg.b, t.b)) { ++deg_b; }
                }
                if (deg_a == 0 || deg_b == 0)
                {
                    sg.alive = false;
                    pruned   = true;
                }
            }
        }
        size_t survivors = 0;
        for (const auto& sg : segs)
        {
            if (sg.alive) { ++survivors; }
        }
        // Apply only when a loop actually remains; an all-stray profile
        // (shouldn't pass the gate) builds unfiltered rather than empty.
        if (any_loop || survivors >= 2)
        {
            for (const auto& sg : segs)
            {
                if (!sg.alive) { keep[sg.idx] = false; }
            }
        }
    }

    uint32_t gid = 1;
    for (size_t ci = 0; ci < n_curves; ++ci)
    {
        const auto& c = cit->at(ci);
        if (!keep[ci]) { continue; }
        if (has_drawn && JGet<bool>(c, "ref", false)) { continue; }
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

// ---- Region-pick hole import ("projection donor") ----------------------
//
// A ZW3D region-pick extrude (R2900 Extrude48) draws only its OUTER loop;
// the island cutouts are regions of ANOTHER feature's profile projected
// into the sketch view. The only static trace is the donor's outer loop
// in the reference set ("ref":true curves). Recover the holes by finding
// the PRECEDING profile whose outer loop is congruent to a ref loop and
// importing its inner closed loops (reprojected into this sketch's frame)
// as holes. Old snapshots carry no ref tags -> all of this is a no-op.

struct ProfFrame
{
    double o[3] = { 0, 0, 0 };
    double x[3] = { 1, 0, 0 };
    double n[3] = { 0, 0, 1 };
    double v[3] = { 0, 1, 0 };   // n cross x
};

ProfFrame FrameFromProfile(const json& prof)
{
    ProfFrame f;
    auto pl = prof.find("plane");
    if (pl != prof.end() && pl->is_object())
    {
        auto rd3 = [&](const char* key, double d[3]) {
            auto a = pl->find(key);
            if (a != pl->end() && a->is_array() && a->size() == 3) {
                d[0] = a->at(0).get<double>();
                d[1] = a->at(1).get<double>();
                d[2] = a->at(2).get<double>();
            }
        };
        rd3("origin", f.o);
        rd3("x_dir",  f.x);
        rd3("normal", f.n);
    }
    f.v[0] = f.n[1]*f.x[2] - f.n[2]*f.x[1];
    f.v[1] = f.n[2]*f.x[0] - f.n[0]*f.x[2];
    f.v[2] = f.n[0]*f.x[1] - f.n[1]*f.x[0];
    return f;
}

// 2D endpoints of a profile curve in its own sketch frame. Circles /
// ellipses are self-closed (returns false; caller treats them as loops).
bool CurveEnds2D(const json& c, double e0[2], double e1[2])
{
    const std::string k = JStr(c, "kind", "");
    if (k == "line" || k == "nurb")
    {
        auto p0 = c.find("p0");
        auto p1 = c.find("p1");
        if (p0 == c.end() || p1 == c.end() ||
            !p0->is_array() || !p1->is_array() ||
            p0->size() < 2 || p1->size() < 2) {
            return false;
        }
        e0[0] = p0->at(0).get<double>(); e0[1] = p0->at(1).get<double>();
        e1[0] = p1->at(0).get<double>(); e1[1] = p1->at(1).get<double>();
        return true;
    }
    if (k == "arc")
    {
        auto ctr = c.find("center");
        if (ctr == c.end() || !ctr->is_array() || ctr->size() < 2) {
            return false;
        }
        const double cx = ctr->at(0).get<double>();
        const double cy = ctr->at(1).get<double>();
        const double r  = JGet<double>(c, "radius", 0.0);
        const double d  = 3.14159265358979323846 / 180.0;
        const double a0 = JGet<double>(c, "a0", 0.0) * d;
        const double a1 = JGet<double>(c, "a1", 0.0) * d;
        e0[0] = cx + r * std::cos(a0); e0[1] = cy + r * std::sin(a0);
        e1[0] = cx + r * std::cos(a1); e1[1] = cy + r * std::sin(a1);
        return true;
    }
    return false;   // circle / ellipse: self-closed
}

// Chain a subset of curves (by index) into closed loops by greedy
// endpoint matching. Self-closed curves are their own loops. Open
// chains are DROPPED (construction junk like datum axis lines).
std::vector<std::vector<int>> ChainClosedLoops(const json&            curves,
                                               const std::vector<int>& idxs,
                                               double                  tol)
{
    std::vector<std::vector<int>> loops;
    std::vector<int>              open;
    for (int i : idxs)
    {
        const std::string k = JStr(curves.at(i), "kind", "");
        if (k == "circle" || k == "ellipse") {
            loops.push_back({ i });
        } else {
            open.push_back(i);
        }
    }
    std::vector<bool> used(open.size(), false);
    for (size_t s = 0; s < open.size(); ++s)
    {
        if (used[s]) { continue; }
        double e0[2], e1[2];
        if (!CurveEnds2D(curves.at(open[s]), e0, e1)) { used[s] = true; continue; }
        std::vector<int> loop{ open[s] };
        used[s] = true;
        double head[2] = { e0[0], e0[1] };
        double tail[2] = { e1[0], e1[1] };
        bool grew = true;
        while (grew)
        {
            // closed?
            if (std::fabs(head[0]-tail[0]) < tol &&
                std::fabs(head[1]-tail[1]) < tol && loop.size() >= 2) {
                break;
            }
            grew = false;
            for (size_t t = 0; t < open.size(); ++t)
            {
                if (used[t]) { continue; }
                double f0[2], f1[2];
                if (!CurveEnds2D(curves.at(open[t]), f0, f1)) { used[t] = true; continue; }
                if (std::fabs(tail[0]-f0[0]) < tol && std::fabs(tail[1]-f0[1]) < tol) {
                    tail[0] = f1[0]; tail[1] = f1[1];
                } else if (std::fabs(tail[0]-f1[0]) < tol && std::fabs(tail[1]-f1[1]) < tol) {
                    tail[0] = f0[0]; tail[1] = f0[1];
                } else {
                    continue;
                }
                loop.push_back(open[t]);
                used[t] = true;
                grew = true;
                break;
            }
        }
        const bool closed = std::fabs(head[0]-tail[0]) < tol &&
                            std::fabs(head[1]-tail[1]) < tol &&
                            (loop.size() >= 2 || JStr(curves.at(loop[0]), "kind", "") == "arc");
        if (closed) {
            loops.push_back(std::move(loop));
        }
    }
    return loops;
}

// 2D bbox of a loop (curve endpoints + arc/circle extremes approximated
// by centre +- r -- exact enough for congruence / containment gating).
void LoopBBox2D(const json& curves, const std::vector<int>& loop,
                double mn[2], double mx[2])
{
    mn[0] = mn[1] = 1e300;
    mx[0] = mx[1] = -1e300;
    auto add = [&](double x, double y) {
        mn[0] = std::min(mn[0], x); mn[1] = std::min(mn[1], y);
        mx[0] = std::max(mx[0], x); mx[1] = std::max(mx[1], y);
    };
    for (int i : loop)
    {
        const json& c = curves.at(i);
        const std::string k = JStr(c, "kind", "");
        if (k == "circle" || k == "ellipse" || k == "arc")
        {
            auto ctr = c.find("center");
            if (ctr != c.end() && ctr->is_array() && ctr->size() >= 2)
            {
                const double r = JGet<double>(c, "radius", 0.0);
                if (k == "arc")
                {
                    // endpoints only (an arc's bbox needs quadrant checks;
                    // endpoints + the centre disc bound is good enough for
                    // gating, and full circles don't take this path).
                    double e0[2], e1[2];
                    if (CurveEnds2D(c, e0, e1)) { add(e0[0], e0[1]); add(e1[0], e1[1]); }
                }
                else
                {
                    add(ctr->at(0).get<double>() - r, ctr->at(1).get<double>() - r);
                    add(ctr->at(0).get<double>() + r, ctr->at(1).get<double>() + r);
                }
            }
        }
        else
        {
            double e0[2], e1[2];
            if (CurveEnds2D(c, e0, e1)) { add(e0[0], e0[1]); add(e1[0], e1[1]); }
        }
    }
}

// Reproject one profile curve from frame A's 2D coords into frame B's.
// Planes must be parallel; handles in-plane rotation AND mirroring
// (R2900: E47's frame is x_dir (1,0,0) normal -Y, E48's x_dir (-1,0,0)
// normal +Y -- a pure mirror in u). Arc sweeps are kept CCW by swapping
// the angle pair under a mirror.
json ReprojectCurve(const json& c, const ProfFrame& A, const ProfFrame& B)
{
    auto to3 = [&](const double uv[2], double p[3]) {
        for (int k = 0; k < 3; ++k) {
            p[k] = A.o[k] + uv[0]*A.x[k] + uv[1]*A.v[k];
        }
    };
    auto to2 = [&](const double p[3], double uv[2]) {
        double d[3] = { p[0]-B.o[0], p[1]-B.o[1], p[2]-B.o[2] };
        uv[0] = d[0]*B.x[0] + d[1]*B.x[1] + d[2]*B.x[2];
        uv[1] = d[0]*B.v[0] + d[1]*B.v[1] + d[2]*B.v[2];
    };
    // Images of A's axes in B's 2D frame (unit for parallel planes).
    double xa[2] = {
        A.x[0]*B.x[0] + A.x[1]*B.x[1] + A.x[2]*B.x[2],
        A.x[0]*B.v[0] + A.x[1]*B.v[1] + A.x[2]*B.v[2] };
    double va[2] = {
        A.v[0]*B.x[0] + A.v[1]*B.x[1] + A.v[2]*B.x[2],
        A.v[0]*B.v[0] + A.v[1]*B.v[1] + A.v[2]*B.v[2] };
    const bool   mirrored = (xa[0]*va[1] - xa[1]*va[0]) < 0.0;
    const double theta    = std::atan2(xa[1], xa[0]);
    const double kRad     = 3.14159265358979323846 / 180.0;

    json out = c;
    out.erase("ref");
    const std::string k = JStr(c, "kind", "");
    auto xpt = [&](const char* key) {
        auto it = c.find(key);
        if (it == c.end() || !it->is_array() || it->size() < 2) { return; }
        double uv[2] = { it->at(0).get<double>(), it->at(1).get<double>() };
        double p3[3], nuv[2];
        to3(uv, p3);
        to2(p3, nuv);
        out[key] = json::array({ nuv[0], nuv[1], 0.0 });
    };
    xpt("p0");
    xpt("p1");
    xpt("center");
    if (k == "arc")
    {
        const double a0 = JGet<double>(c, "a0", 0.0) * kRad;
        const double a1 = JGet<double>(c, "a1", 0.0) * kRad;
        double n0, n1;
        if (!mirrored) { n0 = a0 + theta;  n1 = a1 + theta; }
        else           { n0 = theta - a1;  n1 = theta - a0; }
        out["a0"] = n0 / kRad;
        out["a1"] = n1 / kRad;
    }
    return out;
}

// Planes parallel AND origins separated only along the normal -> the
// projection along the normal is the identity map of the part space.
bool FramesProjectionAligned(const ProfFrame& A, const ProfFrame& B)
{
    const double dot = A.n[0]*B.n[0] + A.n[1]*B.n[1] + A.n[2]*B.n[2];
    if (std::fabs(std::fabs(dot) - 1.0) > 1e-6) { return false; }
    double d[3] = { A.o[0]-B.o[0], A.o[1]-B.o[1], A.o[2]-B.o[2] };
    // remove the normal component; the rest must vanish
    const double along = d[0]*B.n[0] + d[1]*B.n[1] + d[2]*B.n[2];
    for (int k = 0; k < 3; ++k) { d[k] -= along * B.n[k]; }
    return (d[0]*d[0] + d[1]*d[1] + d[2]*d[2]) < 1e-6;
}

// A ZW3D extrude (FtAllExt) yields a SOLID only when its profile bounds a
// closed region -- and only when the SKETCH BUILDER can realize that
// closure. The historic guard accepted any >= 3 curve profile outright,
// which let two failure modes through (02-ear 拉伸2: phantom 300 mm solid
// out of a 20 mm sheet part): an OPEN multi-curve profile (ZW3D's
// surface-extrude idiom), and a closed profile containing a nurb/ellipse
// that BuildSketchFromProfile silently drops (gate counted 3 curves, the
// builder built 2, WiresToFace fabricated a face over the gap).
bool ProfileFormsClosedLoops(const json& curves, double tol);

// TRUE when the effective curves contain at least one CLOSED loop:
// circles / self-closed arcs count outright; line/arc chains are pruned
// of dangling segments (an endpoint with no partner within tol) until
// stable -- whatever survives sits on a cycle. Distinct from
// ProfileFormsClosedLoops (ALL endpoints must pair up): a region-pick
// profile can mix its real closed loops with leftover OPEN construction
// chains (R2900 Extrude21/26/30/31 carry 4 endpoints 157..490 mm apart
// next to perfectly closed loops -- the builder keeps what closes and
// drops the strays), while a profile that is ONLY open chains (02-ear's
// surface-extrude idiom) prunes to nothing and stays rejected.
// all_curves=true: include ref-tagged curves even when drawn curves exist.
// Needed for region-pick cuts where a drawn segment meets a ref arc to
// form a D-shaped closed boundary (R2900 Extrude56_Cut, Extrude61_Cut).
// Long construction reference lines are pruned out automatically by the
// dangling-endpoint algorithm.
bool ProfileHasClosedLoop(const json& curves, double tol,
                          bool all_curves = false)
{
    if (!curves.is_array() || curves.empty()) {
        return false;
    }
    bool has_drawn = false;
    for (const auto& c : curves)
    {
        if (!JGet<bool>(c, "ref", false)) { has_drawn = true; break; }
    }
    struct Seg { double a[2], b[2]; bool alive; };
    std::vector<Seg> segs;
    for (const auto& c : curves)
    {
        if (!all_curves && has_drawn && JGet<bool>(c, "ref", false)) { continue; }
        const std::string k = JStr(c, "kind", "");
        if (k == "circle" || k == "ellipse") {
            return true;
        }
        auto p0 = c.find("p0");
        auto p1 = c.find("p1");
        if (p0 == c.end() || p1 == c.end() ||
            !p0->is_array() || !p1->is_array() ||
            p0->size() < 2 || p1->size() < 2) {
            continue;   // endpoint-less curve: can't chain, ignore
        }
        Seg s;
        s.a[0]  = p0->at(0).get<double>();
        s.a[1]  = p0->at(1).get<double>();
        s.b[0]  = p1->at(0).get<double>();
        s.b[1]  = p1->at(1).get<double>();
        s.alive = true;
        segs.push_back(s);
    }
    auto near2 = [tol](const double* p, const double* q) {
        return std::fabs(p[0] - q[0]) < tol && std::fabs(p[1] - q[1]) < tol;
    };
    // A full circle exported as one arc closes on itself.
    for (const auto& s : segs)
    {
        if (near2(s.a, s.b)) {
            return true;
        }
    }
    // Peel dangling segments until stable; survivors lie on cycles.
    size_t alive  = segs.size();
    bool   pruned = true;
    while (pruned && alive > 0)
    {
        pruned = false;
        for (auto& s : segs)
        {
            if (!s.alive) { continue; }
            int deg_a = 0, deg_b = 0;
            for (const auto& t : segs)
            {
                if (!t.alive || &t == &s) { continue; }
                if (near2(s.a, t.a) || near2(s.a, t.b)) { ++deg_a; }
                if (near2(s.b, t.a) || near2(s.b, t.b)) { ++deg_b; }
            }
            if (deg_a == 0 || deg_b == 0)
            {
                s.alive = false;
                --alive;
                pruned  = true;
            }
        }
    }
    // Two segments suffice (a pair of half-circle arcs is a closed lens).
    return alive >= 2;
}

bool ProfileExtrudable(const json& curves)
{
    if (!curves.is_array()) {
        return false;
    }
    // Effective-curve rule (see BuildSketchFromProfile): tagged reference
    // curves don't count when drawn curves exist.
    bool   has_drawn = false;
    size_t n         = 0;
    for (const auto& c : curves)
    {
        if (!JGet<bool>(c, "ref", false)) { has_drawn = true; break; }
    }
    // Judge the wire the BUILDER will produce, not the wire the json
    // promises: any effective curve of a kind the builder drops makes the
    // built wire open no matter what the endpoints say.
    for (const auto& c : curves)
    {
        if (has_drawn && JGet<bool>(c, "ref", false)) { continue; }
        ++n;
        const std::string k = JStr(c, "kind", "");
        if (k != "line" && k != "circle" && k != "arc") {
            return false;       // builder can't realize this curve
        }
    }
    if (n == 0) {
        return false;
    }
    // At-least-one-closed-loop chaining at 1e-2 mm endpoint tolerance.
    // Both choices are R2900-calibrated against real exports:
    //   - 1e-2, not 1e-4: consecutive DRAWN arcs of an arc-chain ring
    //     (Extrude81, 40 arcs) carry ~3e-3 mm export jitter -- legit
    //     geometry, while a structurally open profile gapes by whole
    //     millimetres (02-ear's surface extrude: 150+ mm).
    //   - any closed loop, not ALL-curves-closed: region-pick profiles
    //     ship stray open construction chains next to their real loops
    //     (Extrude21/26/30/31); the sketch builder drops the strays.
    // 02-ear's failure modes stay rejected: its open profiles have NO
    // closed loop at any tolerance, and the nurb/ellipse kind gate
    // above is unchanged.
    if (ProfileHasClosedLoop(curves, 1e-2)) return true;
    // Fallback: allow ref curves to close the loop. Some region-pick
    // cuts export exactly one drawn segment (the region edge) and a ref
    // arc (the existing face boundary) that together bound a closed
    // D-shape (R2900 Extrude56_Cut, Extrude61_Cut). Long construction
    // reference lines are pruned out by the dangling-endpoint algorithm
    // so 02-ear's surface-extrude failure modes remain rejected.
    return has_drawn && ProfileHasClosedLoop(curves, 1e-2, /*all_curves=*/true);
}

// True when the profile's curves chain into closed loop(s): every open
// endpoint pairs with exactly one other within tol (raw profile units,
// mm). Self-closing circles/ellipses count on their own. Used by the
// revolve arm: an open profile cannot bound a revolved solid, and a
// near-closed one whose gaps exceed the sketch stitch tolerance
// fragments into slivers that poison the body chain (R2900's
// Revolve6_Base carries reference-curve gaps of ~5e-3 mm) -- falling
// through to the authored-STEP fallback is strictly better.
bool ProfileFormsClosedLoops(const json& curves, double tol)
{
    if (!curves.is_array() || curves.empty()) {
        return false;
    }
    // Same effective-curve rule as BuildSketchFromProfile: tagged
    // reference curves are skipped whenever drawn curves exist, so the
    // gate judges exactly the set the sketch will be built from.
    bool has_drawn = false;
    for (const auto& c : curves)
    {
        if (!JGet<bool>(c, "ref", false)) { has_drawn = true; break; }
    }
    int closed_loops = 0;
    std::vector<std::array<double, 2>> ends;
    for (const auto& c : curves)
    {
        if (has_drawn && JGet<bool>(c, "ref", false)) { continue; }
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
            return false;
        }
        ends.push_back({ p0->at(0).get<double>(), p0->at(1).get<double>() });
        ends.push_back({ p1->at(0).get<double>(), p1->at(1).get<double>() });
    }
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
            return false;
        }
    }
    return closed_loops > 0 || !ends.empty();
}

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

// Profiles of already-processed extrudes, for the region-pick hole
// import: a later extrude whose ref loop matches one of these outer
// loops pulls the donor's inner loops in as holes. Pointers into the
// parsed document json (stable for the lifetime of ReadFile).
struct PriorProfile
{
    const json* curves = nullptr;
    ProfFrame   frame;
};

// A reconstructed edge dress-up (fillet/chamfer), keyed by its feature id,
// kept so a later FtPtnFtr that PATTERNS this dress-up can re-apply it at
// the pattern-imaged edge positions (each picked edge anchor translated by
// the pattern offset). Points are stored in the SAME scaled (metre) space
// the dress-up arm emits into its TopoRefIR, so the pattern handler only
// adds a scaled offset -- no re-scaling.
struct ZwDressupSeed
{
    bool                           is_fillet = true;
    double                         size      = 0.0;   // radius / setback, *s
    std::vector<cadapp::TopoRefIR> edges;             // picked edges, points *s
};


// Shared mutable state threaded through the ZW-format feature builders
// below. ReadFile's else-branch fills one of these per feature and offers
// it to each TryBuild* handler in turn; the first that recognizes the
// feature's zw_type reconstructs it (returns true) and the dispatch stops.
struct ZwBuildCtx
{
    const json&                             jf;                   // current feature node
    uint32_t                                id;                   // feature id
    const std::string&                      name;                 // feature name
    const std::string&                      zt;                   // ZW3D zw_type
    double                                  s;                    // unit scale -> metres
    cadapp::DocumentIR&                     out;                  // doc under construction
    uint32_t&                               running_solid_id;     // body chain tip
    std::set<uint32_t>&                     standalone_ids;
    std::set<uint32_t>&                     standalone_sheet_ids;
    std::vector<ExtrudeFootprint>&          extrude_xy;
    std::vector<PriorProfile>&              prior_profiles;
    std::unordered_map<uint32_t, uint32_t>& seed_to_pattern;
    std::unordered_map<uint32_t, ZwDressupSeed>& feat_dressup;    // dress-up seeds, for pattern-of-dress-up
    // Body-lineage redirection. ZW3D mutates a body IN PLACE: 缝合/修剪/组合
    // (sew/trim/combine) all keep the SAME body entity, and their Base /
    // Operator fields reference it by its ROOT feature id no matter how
    // many in-place ops have run since. The cax IR, by contrast, gives
    // every feature its own node, so a raw ref to the root reaches the
    // STALE pre-op geometry AND leaves the intervening op un-consumed --
    // it re-emits as a phantom body. lineage_tip[root] = the latest feature
    // that superseded that lineage; ResolveTip follows the chain so a ref to
    // the root (or any intermediate) lands on the current tip. Updated by
    // each in-place op below, after it wires its (already-resolved) inputs.
    std::unordered_map<uint32_t, uint32_t>& lineage_tip;
    const std::string&                      doc_dir;              // dir of the .cax.json
    const std::string&                      path;                 // path of the .cax.json
};


// Follow the in-place body-lineage chain so a ref to a body's ROOT feature
// (or any intermediate in-place op) resolves to the current tip. See the
// lineage_tip note in ZwBuildCtx.
static uint32_t ResolveTip(const std::unordered_map<uint32_t, uint32_t>& lineage_tip,
                           uint32_t fid)
{
    uint32_t           cur = fid;
    std::set<uint32_t> seen;
    auto               it = lineage_tip.find(cur);
    while (it != lineage_tip.end() && seen.insert(cur).second) {
        cur = it->second;
        it  = lineage_tip.find(cur);
    }
    return cur;
}


static bool TryBuildCdGeomCopy(ZwBuildCtx& ctx)
{
    const json&                             jf                   = ctx.jf;
    const uint32_t                          id                   = ctx.id;
    const std::string&                      name                 = ctx.name;
    const std::string&                      zt                   = ctx.zt;
    const double                            s                    = ctx.s;
    cadapp::DocumentIR&                     out                  = ctx.out;
    uint32_t&                               running_solid_id     = ctx.running_solid_id;
    std::set<uint32_t>&                     standalone_ids       = ctx.standalone_ids;
    std::set<uint32_t>&                     standalone_sheet_ids = ctx.standalone_sheet_ids;
    std::vector<ExtrudeFootprint>&          extrude_xy           = ctx.extrude_xy;
    std::vector<PriorProfile>&              prior_profiles       = ctx.prior_profiles;
    std::unordered_map<uint32_t, uint32_t>& seed_to_pattern      = ctx.seed_to_pattern;
    const std::string&                      doc_dir              = ctx.doc_dir;
    const std::string&                      path                 = ctx.path;
    (void)id; (void)name; (void)zt; (void)s; (void)out; (void)jf;
    (void)running_solid_id; (void)standalone_ids; (void)standalone_sheet_ids;
    (void)extrude_xy; (void)prior_profiles; (void)seed_to_pattern;
    (void)doc_dir; (void)path;

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
            f.ext_strings["zw_geometry"] = ResolveFeatStepPath(
                path, doc_dir, geo->get<std::string>(), id);
            out.features.push_back(std::move(f));
            running_solid_id = id;
            return true;
        }
    }

    return false;
}

static bool TryBuildFtAllExt(ZwBuildCtx& ctx)
{
    const json&                             jf                   = ctx.jf;
    const uint32_t                          id                   = ctx.id;
    const std::string&                      name                 = ctx.name;
    const std::string&                      zt                   = ctx.zt;
    const double                            s                    = ctx.s;
    cadapp::DocumentIR&                     out                  = ctx.out;
    uint32_t&                               running_solid_id     = ctx.running_solid_id;
    std::set<uint32_t>&                     standalone_ids       = ctx.standalone_ids;
    std::set<uint32_t>&                     standalone_sheet_ids = ctx.standalone_sheet_ids;
    std::vector<ExtrudeFootprint>&          extrude_xy           = ctx.extrude_xy;
    std::vector<PriorProfile>&              prior_profiles       = ctx.prior_profiles;
    std::unordered_map<uint32_t, uint32_t>& seed_to_pattern      = ctx.seed_to_pattern;
    const std::string&                      doc_dir              = ctx.doc_dir;
    const std::string&                      path                 = ctx.path;
    (void)id; (void)name; (void)zt; (void)s; (void)out; (void)jf;
    (void)running_solid_id; (void)standalone_ids; (void)standalone_sheet_ids;
    (void)extrude_xy; (void)prior_profiles; (void)seed_to_pattern;
    (void)doc_dir; (void)path;

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
        // along the extrude DIRECTION from the sketch plane. The
        // solid spans offsets [min(S,E), max(S,E)] along that
        // direction; the profile curves sit ON the plane (offset 0).
        //
        // The direction is NOT simply +normal: ZW3D may grow the
        // extrude along -normal (Extrude21's fld 45 dir = -Z while
        // its sketch normal = +Z; Extrude20, same S/E, has dir +Z).
        // Deriving the flip from the S/E ORDER alone (the old
        // `endE < startS`) silently put such a boss on the WRONG
        // side -- Extrude21 landed above the body, too high and
        // detached. Take the true growth direction from fld 45's
        // cached "dir" and resolve its sign vs the sketch normal:
        // sign < 0 means the solid grows along -normal. Shift the
        // plane to the NEAR face (offset min(S,E) along that signed
        // direction), then extrude |E - S|. With sign = +1 (the
        // common case, and the fallback when fld 45 has no dir) this
        // reduces EXACTLY to the previous result, so every
        // already-correct boss is unchanged.
        double startS = FieldValueById(jf, 2, 0.0);
        double endE   = FieldValueById(jf, 3, 0.0);

        // Region-pick hole import. When the sketch carries ref
        // loops (projected geometry) and one of them is congruent
        // to a PRECEDING profile's outer loop, that profile is the
        // projection donor: its inner closed loops, reprojected
        // into this frame and lying strictly inside our drawn
        // outer loop, are this region's island cutouts (R2900
        // Extrude48: a plain rect whose tower / window cutouts
        // exist only in Extrude47's profile; without them the
        // replay extruded a solid 13k mm^3 block, +14% part
        // volume). Snapshots without ref tags skip all of this.
        json        eff_prof;
        const json* use_prof = &*prof;
        {
            const json& cv = prof->at("curves");
            std::vector<int> drawn_idx, ref_idx;
            for (int i = 0; i < (int)cv.size(); ++i) {
                if (JGet<bool>(cv.at(i), "ref", false)) {
                    ref_idx.push_back(i);
                } else {
                    drawn_idx.push_back(i);
                }
            }
            if (!ref_idx.empty() && !drawn_idx.empty())
            {
                const ProfFrame us = FrameFromProfile(*prof);
                double omn[2], omx[2];   // our drawn outer bbox
                LoopBBox2D(cv, drawn_idx, omn, omx);
                json holes = json::array();
                for (const auto& rl :
                     ChainClosedLoops(cv, ref_idx, 0.05))
                {
                    double rmn[2], rmx[2];
                    LoopBBox2D(cv, rl, rmn, rmx);
                    for (const auto& pp : prior_profiles)
                    {
                        if (!FramesProjectionAligned(pp.frame, us)) {
                            continue;
                        }
                        // donor loops, reprojected lazily
                        std::vector<int> all;
                        for (int i = 0; i < (int)pp.curves->size(); ++i) {
                            if (!JGet<bool>(pp.curves->at(i), "ref", false)) {
                                all.push_back(i);
                            }
                        }
                        auto dloops = ChainClosedLoops(*pp.curves, all, 0.05);
                        // find the donor loop congruent to the
                        // ref loop (bbox match in OUR frame)
                        int match = -1;
                        std::vector<json> proj(dloops.size());
                        for (int li = 0; li < (int)dloops.size(); ++li)
                        {
                            json pl = json::array();
                            for (int ci : dloops[li]) {
                                pl.push_back(ReprojectCurve(
                                    pp.curves->at(ci), pp.frame, us));
                            }
                            proj[li] = std::move(pl);
                            std::vector<int> pidx((size_t)proj[li].size());
                            for (int q = 0; q < (int)pidx.size(); ++q) pidx[q] = q;
                            double pmn[2], pmx[2];
                            LoopBBox2D(proj[li], pidx, pmn, pmx);
                            if (std::fabs(pmn[0]-rmn[0]) < 0.05 &&
                                std::fabs(pmn[1]-rmn[1]) < 0.05 &&
                                std::fabs(pmx[0]-rmx[0]) < 0.05 &&
                                std::fabs(pmx[1]-rmx[1]) < 0.05) {
                                match = li;
                            }
                        }
                        if (match < 0) { continue; }
                        // import every OTHER donor loop strictly
                        // inside our drawn outer bbox as a hole
                        for (int li = 0; li < (int)dloops.size(); ++li)
                        {
                            if (li == match) { continue; }
                            std::vector<int> pidx((size_t)proj[li].size());
                            for (int q = 0; q < (int)pidx.size(); ++q) pidx[q] = q;
                            double pmn[2], pmx[2];
                            LoopBBox2D(proj[li], pidx, pmn, pmx);
                            if (pmn[0] > omn[0] - 0.01 && pmx[0] < omx[0] + 0.01 &&
                                pmn[1] > omn[1] - 0.01 && pmx[1] < omx[1] + 0.01)
                            {
                                for (auto& pc : proj[li]) {
                                    holes.push_back(pc);
                                }
                            }
                        }
                        break;   // first matching donor wins
                    }
                }
                if (!holes.empty())
                {
                    // The region BETWEEN the imported loops (the
                    // tower disc bridged by the four window arcs,
                    // ~3.5k mm^3 on R2900) is excluded by ZW3D
                    // too: when several imported loops carry arcs
                    // of one shared support circle, emit the full
                    // circle as one more hole. It only SHARES
                    // BOUNDARY with the window loops; the face
                    // builder unions touching holes (face fuse +
                    // UnifySameDomain -- the fuse alone leaves
                    // coplanar fragments whose wires still
                    // overlap; installing those raw was the
                    // failure mode of the first two attempts).
                    {
                        struct ArcSup { double cx, cy, r; int n; };
                        std::vector<ArcSup> sups;
                        for (const auto& h : holes)
                        {
                            if (JStr(h, "kind", "") != "arc") { continue; }
                            auto ctr = h.find("center");
                            if (ctr == h.end() || !ctr->is_array() ||
                                ctr->size() < 2) { continue; }
                            const double cx = ctr->at(0).get<double>();
                            const double cy = ctr->at(1).get<double>();
                            const double r  = JGet<double>(h, "radius", 0.0);
                            bool found = false;
                            for (auto& sp : sups)
                            {
                                if (std::fabs(sp.cx-cx) < 0.01 &&
                                    std::fabs(sp.cy-cy) < 0.01 &&
                                    std::fabs(sp.r-r)   < 0.01)
                                {
                                    ++sp.n;
                                    found = true;
                                    break;
                                }
                            }
                            if (!found) { sups.push_back({cx, cy, r, 1}); }
                        }
                        for (const auto& sp : sups)
                        {
                            if (sp.n < 2) { continue; }
                            if (sp.cx - sp.r <= omn[0] || sp.cx + sp.r >= omx[0] ||
                                sp.cy - sp.r <= omn[1] || sp.cy + sp.r >= omx[1]) {
                                continue;
                            }
                            json circ;
                            circ["kind"]   = "circle";
                            circ["center"] = json::array({ sp.cx, sp.cy, 0.0 });
                            circ["radius"] = sp.r;
                            holes.push_back(std::move(circ));
                        }
                    }
                    eff_prof = *prof;
                    json eff = json::array();
                    for (int i : drawn_idx) { eff.push_back(cv.at(i)); }
                    for (auto& h : holes)   { eff.push_back(std::move(h)); }
                    eff_prof["curves"] = std::move(eff);
                    use_prof = &eff_prof;
                }
            }
        }

        cadapp::SketchIR sk;
        sk.name = name + ":profile";
        BuildSketchFromProfile(*use_prof, s, sketch_fid, sk,
                               /*loops_only=*/true);

        // Register this profile as a potential projection donor
        // for later region-pick extrudes (drawn curves only).
        prior_profiles.push_back({ &prof->at("curves"),
                                   FrameFromProfile(*prof) });

        double edir[3];
        double sign = 1.0;
        if (FieldDir(jf, 45, edir)) {
            double dot = edir[0] * sk.plane_normal[0] +
                         edir[1] * sk.plane_normal[1] +
                         edir[2] * sk.plane_normal[2];
            if (dot < 0.0) { sign = -1.0; }
        }
        double near_off = (startS < endE) ? startS : endE;
        // fld 31 'Extrude type' = 3: SYMMETRIC about the
        // profile plane. S/E are dialog residue then (02-ear
        // 拉伸23: S=0,E=5 yet the authored pin is centred on
        // the plane -- the one-sided replay landed exactly
        // E/2 = 2.498mm off along the growth direction).
        if (std::fabs(FieldValueById(jf, 31, 0.0) - 3.0) < 0.5)
        {
            near_off = -std::fabs(endE - startS) * 0.5;
        }
        const double near_m   = near_off * sign * s;
        sk.plane_origin[0] += near_m * sk.plane_normal[0];
        sk.plane_origin[1] += near_m * sk.plane_normal[1];
        sk.plane_origin[2] += near_m * sk.plane_normal[2];
        out.sketches.push_back(std::move(sk));

        cadapp::FeatPayloadSketch spl;
        spl.sketch_id = sketch_fid;
        cadapp::FeatureIR sf;
        sf.id   = sketch_fid;
        sf.type = cadapp::FeatType::Sketch;
        sf.name = name + ":profile";
        sf.data = std::move(spl);
        out.features.push_back(std::move(sf));

        // Extrude |E - S| along the signed direction (flip = grow
        // along -normal, from fld 45's true direction resolved above).
        double depth  = std::fabs(endE - startS) * s;

        cadapp::FeatPayloadExtrude epl;
        epl.sketch_id      = sketch_fid;
        epl.distance       = depth;
        epl.end_type       = cadapp::ExtrudeEndType::Blind;
        epl.flip_direction = (sign < 0.0);

        // Draft angle: fld 4 holds the LAST dialog value even
        // when drafting is off -- R2900_100 carries stale 45s
        // on four undrafted extrudes. fld 46 is the enable
        // toggle (1 only on Extrude21, whose truth walls tilt
        // by exactly sin(5 deg)); gate on it, never on the
        // angle alone.
        if (std::fabs(FieldValueById(jf, 46, 0.0)) > 0.5)
        {
            epl.draft = FieldValueById(jf, 4, 0.0)
                      * 3.14159265358979323846 / 180.0;
        }

        // fld 14 = ZW3D boolean combine: 0 = new/base, 1 = add
        // (boss), 2 = remove (cut). A cut rebuilt as a boss just
        // fuses material already inside the body -> "no visible
        // effect"; route fld14==2 to CutExtrude so it subtracts
        // (the Replayer cuts the prism from the running body).
        const double combine14 = FieldValueById(jf, 14, 0.0);
        const bool is_cut  = std::fabs(combine14 - 2.0) < 0.5;
        const bool is_base = std::fabs(combine14) < 0.5;

        cadapp::FeatureIR ef;
        ef.id   = id;
        ef.type = is_cut ? cadapp::FeatType::CutExtrude
                         : cadapp::FeatType::BossExtrude;
        ef.name = name;
        ef.data = std::move(epl);

        // Wire the running body (imported / prior solid) as the
        // Base input: a boss fuses the prism onto it, a cut
        // subtracts the prism from it (the Replayer picks the
        // boolean by feat type). EXCEPT fld14=0: that opens a
        // NEW body ZW3D keeps standalone -- 02-ear's pin forest
        // lost 5 solids to implicit fuses (拉伸14/17/18 absorbed
        // without trace, 拉伸9+19 and 拉伸11+12 pairwise merged).
        // The chain ROOT (no running body yet) still advances
        // the tip: a single-body part's first base extrude is
        // what later adds fuse onto.
        const bool standalone_base =
            is_base && running_solid_id != 0;
        if (running_solid_id != 0 && !standalone_base) {
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
        if (standalone_base)
        {
            // New independent body: the chain tip stays on the
            // body it was building; this extrude emits on its
            // own (and is consumable as a sew/trim operand).
            standalone_ids.insert(id);
        }
        else
        {
            running_solid_id = id;
        }
        return true;
    }

    return false;
}

static bool TryBuildFtAllCyl(ZwBuildCtx& ctx)
{
    const json&                             jf                   = ctx.jf;
    const uint32_t                          id                   = ctx.id;
    const std::string&                      name                 = ctx.name;
    const std::string&                      zt                   = ctx.zt;
    const double                            s                    = ctx.s;
    cadapp::DocumentIR&                     out                  = ctx.out;
    uint32_t&                               running_solid_id     = ctx.running_solid_id;
    std::set<uint32_t>&                     standalone_ids       = ctx.standalone_ids;
    std::set<uint32_t>&                     standalone_sheet_ids = ctx.standalone_sheet_ids;
    std::vector<ExtrudeFootprint>&          extrude_xy           = ctx.extrude_xy;
    std::vector<PriorProfile>&              prior_profiles       = ctx.prior_profiles;
    std::unordered_map<uint32_t, uint32_t>& seed_to_pattern      = ctx.seed_to_pattern;
    const std::string&                      doc_dir              = ctx.doc_dir;
    const std::string&                      path                 = ctx.path;
    (void)id; (void)name; (void)zt; (void)s; (void)out; (void)jf;
    (void)running_solid_id; (void)standalone_ids; (void)standalone_sheet_ids;
    (void)extrude_xy; (void)prior_profiles; (void)seed_to_pattern;
    (void)doc_dir; (void)path;

    // ZW3D cylinder primitive (FtAllCyl) -> circle-profile Sketch
    // + Boss/CutExtrude. fld 1 "Center" is the base circle's
    // centre (world mm), fld 2 "Radius" / fld 3 "Length" the
    // size, fld 11 carries the axis direction, and fld 14 the
    // boolean combine (0 = new/base, 1 = add, 2 = remove).
    // Lowered through the same Sketch+Extrude path as FtAllExt
    // (not a PrimCylinder payload) so the pattern machinery
    // keeps working: the footprint registered below is what
    // lets a later FtPtnFtr map its anchor back to this
    // cylinder (R2900's Pattern13/18 copy cylinder bosses).
    if (zt == "FtAllCyl")
    {
        double ctr[3]  = { 0.0, 0.0, 0.0 };
        double axis[3] = { 0.0, 0.0, 1.0 };
        const double radius = FieldValueById(jf, 2, 0.0);
        const double length = FieldValueById(jf, 3, 0.0);
        if (FieldPoint(jf, 1, ctr) && radius > 1e-9 &&
            std::fabs(length) > 1e-6)
        {
            FieldDir(jf, 11, axis);   // stays +Z when absent
            double al = std::sqrt(axis[0]*axis[0] +
                                  axis[1]*axis[1] +
                                  axis[2]*axis[2]);
            if (al < 1e-12) {
                axis[0] = 0.0; axis[1] = 0.0; axis[2] = 1.0;
                al = 1.0;
            }
            axis[0] /= al; axis[1] /= al; axis[2] /= al;

            // Any unit vector perpendicular to the axis works as
            // the sketch frame's x_dir -- the profile is a full
            // circle, so rotation about the axis is immaterial.
            double ux[3];
            if (std::fabs(axis[2]) < 0.9) {
                ux[0] = -axis[1]; ux[1] = axis[0]; ux[2] = 0.0;
            } else {
                ux[0] = 0.0; ux[1] = -axis[2]; ux[2] = axis[1];
            }
            double ul = std::sqrt(ux[0]*ux[0] + ux[1]*ux[1] +
                                  ux[2]*ux[2]);
            ux[0] /= ul; ux[1] /= ul; ux[2] /= ul;

            const uint32_t sketch_fid = 1000000u + id;
            cadapp::SketchIR sk;
            sk.feature_id = sketch_fid;
            sk.name = name + ":profile";
            sk.plane_origin[0] = ctr[0] * s;
            sk.plane_origin[1] = ctr[1] * s;
            sk.plane_origin[2] = ctr[2] * s;
            sk.plane_x_dir[0]  = ux[0];
            sk.plane_x_dir[1]  = ux[1];
            sk.plane_x_dir[2]  = ux[2];
            sk.plane_normal[0] = axis[0];
            sk.plane_normal[1] = axis[1];
            sk.plane_normal[2] = axis[2];
            sk.geos.push_back(cadapp::SkGeoIR::Circle(
                1, 0.0, 0.0, radius * s));
            out.sketches.push_back(std::move(sk));

            cadapp::FeatPayloadSketch spl;
            spl.sketch_id = sketch_fid;
            cadapp::FeatureIR sf;
            sf.id   = sketch_fid;
            sf.type = cadapp::FeatType::Sketch;
            sf.name = name + ":profile";
            sf.data = std::move(spl);
            out.features.push_back(std::move(sf));

            cadapp::FeatPayloadExtrude epl;
            epl.sketch_id      = sketch_fid;
            epl.distance       = std::fabs(length) * s;
            epl.end_type       = cadapp::ExtrudeEndType::Blind;
            epl.flip_direction = (length < 0.0);
            // fld 46 gates the draft; fld 4 alone is the
            // stale dialog value (see the profile branch).
            if (std::fabs(FieldValueById(jf, 46, 0.0)) > 0.5)
            {
                epl.draft = FieldValueById(jf, 4, 0.0)
                          * 3.14159265358979323846 / 180.0;
            }

            const bool is_cut =
                std::fabs(FieldValueById(jf, 14, 1.0) - 2.0) < 0.5;
            cadapp::FeatureIR ef;
            ef.id   = id;
            ef.type = is_cut ? cadapp::FeatType::CutExtrude
                             : cadapp::FeatType::BossExtrude;
            ef.name = name;
            ef.data = std::move(epl);
            if (running_solid_id != 0) {
                PushInput(ef, running_solid_id,
                          cadapp::InputRole::Base);
            }

            // Footprint (plane frame + circle box, raw mm) so a
            // later pattern's anchor maps back to this cylinder.
            {
                ExtrudeFootprint fp;
                fp.id = id;
                double vd[3] = {
                    axis[1]*ux[2] - axis[2]*ux[1],
                    axis[2]*ux[0] - axis[0]*ux[2],
                    axis[0]*ux[1] - axis[1]*ux[0],
                };
                for (int k = 0; k < 3; ++k) {
                    fp.origin[k] = ctr[k];
                    fp.udir[k]   = ux[k];
                    fp.vdir[k]   = vd[k];
                    fp.wc[k]     = ctr[k];
                }
                fp.umin = -radius; fp.umax = radius;
                fp.vmin = -radius; fp.vmax = radius;
                extrude_xy.push_back(fp);
            }

            out.features.push_back(std::move(ef));
            running_solid_id = id;
            return true;
        }
    }

    return false;
}

static bool TryBuildFtAllSwp1(ZwBuildCtx& ctx)
{
    const json&                             jf                   = ctx.jf;
    const uint32_t                          id                   = ctx.id;
    const std::string&                      name                 = ctx.name;
    const std::string&                      zt                   = ctx.zt;
    const double                            s                    = ctx.s;
    cadapp::DocumentIR&                     out                  = ctx.out;
    uint32_t&                               running_solid_id     = ctx.running_solid_id;
    std::set<uint32_t>&                     standalone_ids       = ctx.standalone_ids;
    std::set<uint32_t>&                     standalone_sheet_ids = ctx.standalone_sheet_ids;
    std::vector<ExtrudeFootprint>&          extrude_xy           = ctx.extrude_xy;
    std::vector<PriorProfile>&              prior_profiles       = ctx.prior_profiles;
    std::unordered_map<uint32_t, uint32_t>& seed_to_pattern      = ctx.seed_to_pattern;
    const std::string&                      doc_dir              = ctx.doc_dir;
    const std::string&                      path                 = ctx.path;
    (void)id; (void)name; (void)zt; (void)s; (void)out; (void)jf;
    (void)running_solid_id; (void)standalone_ids; (void)standalone_sheet_ids;
    (void)extrude_xy; (void)prior_profiles; (void)seed_to_pattern;
    (void)doc_dir; (void)path;
    auto prof = jf.find("profile");

    // ZW3D sweep (FtAllSwp1) -> profile Sketch + spine Sketch +
    // FeatPayloadSweep. The profile rides the same "profile"
    // block as an extrude's; the PATH comes from the exporter's
    // "path" dump (fld 2's point-on-curve pick resolved to
    // world-mm curve geometry via cvxPartInqCurve). V1 handles a
    // single-segment line or arc spine -- R2900's five sweeps
    // are exactly that (pin tubes and a 4 mm elbow). The spine
    // becomes its own synthetic planar sketch: a line along the
    // sketch X axis, or an arc centred in a plane spanned by the
    // exported frame axes; the Replayer's existing Sweep arm
    // (profile face swept along the spine wire, fused as a boss)
    // does the rest. Multi-segment / NURBS paths stay opaque.
    if (zt == "FtAllSwp1" && prof != jf.end() &&
        prof->contains("curves") &&
        ProfileFormsClosedLoops(prof->at("curves"), 1e-4) &&
        running_solid_id != 0)
    {
        auto pit = jf.find("path");
        const json* pc = (pit != jf.end() && pit->is_array() &&
                          pit->size() == 1)
                             ? &pit->at(0)
                             : nullptr;
        const std::string pk = pc ? JStr(*pc, "kind", "") : "";

        cadapp::SketchIR spine;
        bool spine_ok = false;
        if (pk == "line" && pc->contains("p0") && pc->contains("p1"))
        {
            double p0[3], p1[3];
            for (int k = 0; k < 3; ++k) {
                p0[k] = pc->at("p0").at(k).get<double>();
                p1[k] = pc->at("p1").at(k).get<double>();
            }
            double dx[3] = { p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2] };
            double len = std::sqrt(dx[0]*dx[0] + dx[1]*dx[1] +
                                   dx[2]*dx[2]);
            if (len > 1e-9)
            {
                for (int k = 0; k < 3; ++k) {
                    spine.plane_origin[k] = p0[k] * s;
                    spine.plane_x_dir[k]  = dx[k] / len;
                }
                // Any unit normal perpendicular to the line works:
                // the wire is 1D, the plane only orients the frame.
                double n[3];
                if (std::fabs(spine.plane_x_dir[2]) < 0.9) {
                    n[0] = -spine.plane_x_dir[1];
                    n[1] =  spine.plane_x_dir[0];
                    n[2] =  0.0;
                } else {
                    n[0] =  0.0;
                    n[1] = -spine.plane_x_dir[2];
                    n[2] =  spine.plane_x_dir[1];
                }
                double nl = std::sqrt(n[0]*n[0] + n[1]*n[1] +
                                      n[2]*n[2]);
                // Sketch normal = plane Z; x_dir is plane X. The
                // wire lives along X so this cross frame is valid.
                spine.plane_normal[0] = n[0] / nl;
                spine.plane_normal[1] = n[1] / nl;
                spine.plane_normal[2] = n[2] / nl;
                spine.geos.push_back(cadapp::SkGeoIR::Line(
                    1, 0.0, 0.0, len * s, 0.0));
                spine_ok = true;
            }
        }
        else if (pk == "arc" && pc->contains("center") &&
                 pc->contains("x_dir") && pc->contains("y_dir") &&
                 pc->contains("radius"))
        {
            double C[3], X[3], Y[3];
            for (int k = 0; k < 3; ++k) {
                C[k] = pc->at("center").at(k).get<double>();
                X[k] = pc->at("x_dir").at(k).get<double>();
                Y[k] = pc->at("y_dir").at(k).get<double>();
            }
            double R  = JGet<double>(*pc, "radius", 0.0);
            double a0 = JGet<double>(*pc, "a0", 0.0)
                      * 3.14159265358979323846 / 180.0;
            double a1 = JGet<double>(*pc, "a1", 0.0)
                      * 3.14159265358979323846 / 180.0;
            if (R > 1e-9 && std::fabs(a1 - a0) > 1e-9)
            {
                for (int k = 0; k < 3; ++k) {
                    spine.plane_origin[k] = C[k] * s;
                    spine.plane_x_dir[k]  = X[k];
                }
                spine.plane_normal[0] = X[1]*Y[2] - X[2]*Y[1];
                spine.plane_normal[1] = X[2]*Y[0] - X[0]*Y[2];
                spine.plane_normal[2] = X[0]*Y[1] - X[1]*Y[0];
                spine.geos.push_back(cadapp::SkGeoIR::Arc(
                    1, 0.0, 0.0, R * s, a0, a1));
                spine_ok = true;
            }
        }

        // Whole-sketch path ("path_sketch": insertion plane +
        // 2D curve chain; R2900 Sweep2 = one line, Sweep4 =
        // 4 lines + 2 arcs). Reuse the profile-sketch builder
        // -- the chain becomes the spine sketch's geometry.
        // Circles are rejected: a closed spine sweep is a
        // different animal and none of the corpus needs it.
        if (!spine_ok)
        {
            auto psit = jf.find("path_sketch");
            if (psit != jf.end() && psit->is_object() &&
                psit->contains("curves") &&
                psit->at("curves").is_array() &&
                !psit->at("curves").empty())
            {
                bool chain_ok = true;
                for (const auto& c : psit->at("curves"))
                {
                    const std::string k = JStr(c, "kind", "");
                    if (k != "line" && k != "arc") {
                        chain_ok = false;
                        break;
                    }
                }
                if (chain_ok)
                {
                    BuildSketchFromProfile(*psit, s, 0, spine);
                    spine_ok = !spine.geos.empty();
                }
            }
        }

        // Body-edge path: the plugin could not export the curve
        // (on_crv=0, parent=0 in _diag path_rows) and only kept
        // the pick POINT in field id=2 type="point".  Store the
        // pick coordinates so the Replayer can find the nearest
        // edge in the running body at eval time.
        bool   edge_pick_ok = false;
        double edge_pick[3] = { 0.0, 0.0, 0.0 };
        if (!spine_ok && !jf.contains("path") &&
            !jf.contains("path_sketch"))
        {
            if (FieldPoint(jf, 2, edge_pick)) {
                edge_pick_ok = true;
            }
        }

        if (spine_ok || edge_pick_ok)
        {
            const uint32_t prof_fid  = 1000000u + id;
            const uint32_t spine_fid = 1500000u + id;

            cadapp::SketchIR psk;
            psk.name = name + ":profile";
            BuildSketchFromProfile(*prof, s, prof_fid, psk);
            out.sketches.push_back(std::move(psk));
            cadapp::FeatPayloadSketch pspl;
            pspl.sketch_id = prof_fid;
            cadapp::FeatureIR psf;
            psf.id   = prof_fid;
            psf.type = cadapp::FeatType::Sketch;
            psf.name = name + ":profile";
            psf.data = std::move(pspl);
            out.features.push_back(std::move(psf));

            cadapp::FeatPayloadSweep swp;
            swp.profile_sketch_id = prof_fid;
            cadapp::FeatureIR sf;
            sf.id   = id;
            sf.name = name;
            sf.type = cadapp::FeatType::Sweep;
            sf.data = std::move(swp);
            sf.ext_strings["zw_type"] = zt;
            sf.ext_params["sweep_solid"] = 1.0;

            if (spine_ok)
            {
                spine.feature_id = spine_fid;
                spine.name       = name + ":spine";
                out.sketches.push_back(std::move(spine));
                cadapp::FeatPayloadSketch sspl;
                sspl.sketch_id = spine_fid;
                cadapp::FeatureIR ssf;
                ssf.id   = spine_fid;
                ssf.type = cadapp::FeatType::Sketch;
                ssf.name = name + ":spine";
                ssf.data = std::move(sspl);
                out.features.push_back(std::move(ssf));
                sf.ext_params["spine_sketch_id"] = (double)spine_fid;
            }
            else
            {
                // edge_pick_ok: path is a body edge; store the
                // world pick point (raw mm × s = metres) for the
                // Replayer's edge_pick_wire calc-graph node.
                sf.ext_params["edge_pick_x"] = edge_pick[0] * s;
                sf.ext_params["edge_pick_y"] = edge_pick[1] * s;
                sf.ext_params["edge_pick_z"] = edge_pick[2] * s;
            }

            PushInput(sf, running_solid_id, cadapp::InputRole::Base);
            out.features.push_back(std::move(sf));
            running_solid_id = id;
            return true;
        }
    }

    return false;
}

static bool TryBuildFtAllRev(ZwBuildCtx& ctx)
{
    const json&                             jf                   = ctx.jf;
    const uint32_t                          id                   = ctx.id;
    const std::string&                      name                 = ctx.name;
    const std::string&                      zt                   = ctx.zt;
    const double                            s                    = ctx.s;
    cadapp::DocumentIR&                     out                  = ctx.out;
    uint32_t&                               running_solid_id     = ctx.running_solid_id;
    std::set<uint32_t>&                     standalone_ids       = ctx.standalone_ids;
    std::set<uint32_t>&                     standalone_sheet_ids = ctx.standalone_sheet_ids;
    std::vector<ExtrudeFootprint>&          extrude_xy           = ctx.extrude_xy;
    std::vector<PriorProfile>&              prior_profiles       = ctx.prior_profiles;
    std::unordered_map<uint32_t, uint32_t>& seed_to_pattern      = ctx.seed_to_pattern;
    const std::string&                      doc_dir              = ctx.doc_dir;
    const std::string&                      path                 = ctx.path;
    (void)id; (void)name; (void)zt; (void)s; (void)out; (void)jf;
    (void)running_solid_id; (void)standalone_ids; (void)standalone_sheet_ids;
    (void)extrude_xy; (void)prior_profiles; (void)seed_to_pattern;
    (void)doc_dir; (void)path;
    auto prof = jf.find("profile");

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
        prof->contains("curves") &&
        ProfileFormsClosedLoops(prof->at("curves"), 1e-4))
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

            // No profile line carries the axis point -- the axis
            // is EXTERNAL to the profile (R2900's Revolve3 spins
            // a D-section about a remote -Y axis). The plugin
            // caches the true direction on fld 2 itself ("Axis A"
            // carries dir alongside pt); take it as fallback. The
            // line-fit stays primary so every snapshot that
            // resolved before resolves identically.
            if (best_d >= 1e-3)
            {
                double fdir[3];
                if (FieldDir(jf, 2, fdir)) {
                    double fl = std::sqrt(fdir[0]*fdir[0] +
                                          fdir[1]*fdir[1] +
                                          fdir[2]*fdir[2]);
                    if (fl > 1e-12) {
                        axis_dir[0] = fdir[0] / fl;
                        axis_dir[1] = fdir[1] / fl;
                        axis_dir[2] = fdir[2] / fl;
                        best_d = 0.0;
                    }
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
                return true;
            }
        }
    }

    return false;
}

static bool TryBuildFtPtnFtr(ZwBuildCtx& ctx)
{
    const json&                             jf                   = ctx.jf;
    const uint32_t                          id                   = ctx.id;
    const std::string&                      name                 = ctx.name;
    const std::string&                      zt                   = ctx.zt;
    const double                            s                    = ctx.s;
    cadapp::DocumentIR&                     out                  = ctx.out;
    uint32_t&                               running_solid_id     = ctx.running_solid_id;
    std::set<uint32_t>&                     standalone_ids       = ctx.standalone_ids;
    std::set<uint32_t>&                     standalone_sheet_ids = ctx.standalone_sheet_ids;
    std::vector<ExtrudeFootprint>&          extrude_xy           = ctx.extrude_xy;
    std::vector<PriorProfile>&              prior_profiles       = ctx.prior_profiles;
    std::unordered_map<uint32_t, uint32_t>& seed_to_pattern      = ctx.seed_to_pattern;
    const std::string&                      doc_dir              = ctx.doc_dir;
    const std::string&                      path                 = ctx.path;
    (void)id; (void)name; (void)zt; (void)s; (void)out; (void)jf;
    (void)running_solid_id; (void)standalone_ids; (void)standalone_sheet_ids;
    (void)extrude_xy; (void)prior_profiles; (void)seed_to_pattern;
    (void)doc_dir; (void)path;

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
        // fld 10 is the method enum (0 = linear, 1 = circular)
        // and is AUTHORITATIVE when the dump carries it. fld 28
        // "Diameter" is only a backstop for old snapshots with
        // no fld 10: like fld 4's stale draft angle and fld 12's
        // stale spacing angle, it holds the LAST dialog value
        // even on a linear pattern -- R2900's Pattern12 (a plain
        // 2 x 46 mm +Z linear copy of the pin tower) carries a
        // stale Diameter=8.2 and was silently rebuilt as a 24-deg
        // CIRCULAR pattern: copies 39 mm off bbox, +23% phantom
        // volume, the dominant error of the whole part.
        double f10 = FieldValueById(jf, 10, -1.0);
        bool   is_circular     = (f10 >= 0.0)
                                   ? (f10 > 0.5)
                                   : (FieldValueById(jf, 28, 0.0)
                                          > 1e-6);
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
        // Can fld 1 identify the seed by direct feature-id
        // reference ("feat" field), without needing anchor
        // matching in extrude_xy? This lets a pattern of a
        // box/BakedShape primitive be reconstructed even when
        // no FtAllExt extrude has been processed yet.
        auto has_fld1_feat_ref = [&]() -> bool {
            auto fit2 = jf.find("fields");
            if (fit2 == jf.end() || !fit2->is_array()) return false;
            for (const auto& fd : *fit2) {
                auto idit2 = fd.find("id");
                if (idit2 == fd.end() || !idit2->is_number() ||
                    idit2->get<int>() != 1) {
                    continue;
                }
                auto eit2 = fd.find("ents");
                if (eit2 == fd.end() || !eit2->is_array()) continue;
                for (const auto& e2 : *eit2) {
                    auto ft2 = e2.find("feat");
                    if (ft2 != e2.end() &&
                        ft2->is_number_integer() &&
                        ft2->get<int>() > 0) {
                        return true;
                    }
                }
            }
            return false;
        };
        // ZW3D "Pattern Feature" whose seed is an edge DRESS-UP
        // (fillet/chamfer), not a tool body -- fld 1 Base names the
        // dress-up feature (e.g. Fillet1), and the bodies its copies dress
        // ALREADY exist: an earlier "Pattern Geometry" (FtPtnGeom) deposited
        // them, and fld 62 "Boolean shapes" names the one this instance
        // lands on. A dress-up has no copyable tool solid, so the
        // LinearPattern arm below cannot replicate it -- it pushes the fillet
        // as a Tool that resolves to nothing AND consumes the fld-62 body
        // without fusing, so that body silently vanishes. Instead re-apply
        // the SAME dress-up at the pattern-IMAGED edges: translate each
        // picked edge anchor by the pattern offset and fillet/chamfer the
        // fld-62 body there, reusing the single-dress-up arm (no Replayer
        // change). The Replayer's resolver finds the moved edge (box2 =
        // box1 + offset, so the imaged anchor sits on a real edge); a miss
        // degrades to a clean no-op via its dead-feature fallback, keeping
        // the body. Base = the fld-62 body, so the Replayer consumes it and
        // the dressed body replaces the plain copy (no duplicate, no vanish).
        //
        // GUARD: only when the fld-62 body is a STANDALONE deposited body (an
        // FtPtnGeom copy in standalone_ids) -- that is the "geometry already
        // laid down, the pattern only dresses it" shape this models. A pattern
        // whose fld-62 body is the RUNNING chain root (R2900's Pattern2
        // chamfer, onto = the base extrude) is a different topology: its edges
        // live on the EVOLVED running body, not on feature_nodes[onto] (the
        // bare extrude), so it stays on the existing path. Handles a 1-D
        // pattern of count1 (>= 2) instances along fld 2; each non-seed image
        // dresses its copy. (2-D grids would need fld 5's second direction,
        // which is unreliable, so they fall through.) Correctness needs the
        // pattern transform to MATCH the one that built the fld-62 body (true
        // for ptn_ftr_1: both +Y/50); a mismatch lands the imaged anchor
        // off-body -> that instance's dress-up no-ops, body kept.
        if (zt == "FtPtnFtr" && linear_method && running_solid_id != 0)
        {
            uint32_t seed_dress = FieldEntFeat(jf, 1);
            auto     dsit       = ctx.feat_dressup.find(seed_dress);
            uint32_t onto       = FieldEntFeat(jf, 62);
            int      count1     = static_cast<int>(FieldValueById(jf, 3, 1.0));
            int      count2     = static_cast<int>(FieldValueById(jf, 6, 1.0));
            if (dsit != ctx.feat_dressup.end() && onto != 0 &&
                standalone_ids.count(onto) > 0 &&
                count2 == 1 && count1 >= 2 && count1 <= 400)
            {
                // Per-instance step (scaled to metres) along the pattern's
                // direction. ONLY fld 2 "Direction" carries the true signed
                // axis; fld 5 "Direction D" is a DIFFERENT, wrong vector (see
                // the LinearPattern arm below), so the 2-D grid case
                // (count2 > 1, which would need fld 5) is left to fall
                // through. fld 4 is the spacing.
                double d1[3] = { 0, 0, 0 };
                FieldDir(jf, 2, d1);
                double sp1 = FieldValueById(jf, 4, 0.0) * s;

                // Image every picked edge at each NON-seed instance
                // i = 1 .. count1-1: anchor + i*step. The seed copy (i = 0)
                // is already dressed by the seed dress-up feature.
                const ZwDressupSeed& ds = dsit->second;
                std::vector<cadapp::TopoRefIR> edges;
                edges.reserve(ds.edges.size() * (count1 - 1));
                for (int i = 1; i < count1; ++i) {
                    double off[3] = { i * sp1 * d1[0],
                                      i * sp1 * d1[1],
                                      i * sp1 * d1[2] };
                    for (const auto& se : ds.edges) {
                        cadapp::TopoRefIR r = se;
                        r.point[0] += off[0];
                        r.point[1] += off[1];
                        r.point[2] += off[2];
                        edges.push_back(r);
                    }
                }

                cadapp::FeatureIR cf;
                cf.id   = id;
                cf.name = name;
                cf.ext_strings["zw_type"] = zt;
                if (ds.is_fillet) {
                    cadapp::FeatPayloadFillet pl;
                    pl.radius = ds.size;
                    pl.edges  = std::move(edges);
                    cf.type   = cadapp::FeatType::Fillet;
                    cf.data   = std::move(pl);
                } else {
                    cadapp::FeatPayloadChamfer pl;
                    pl.distance1 = ds.size;
                    pl.distance2 = 0.0;
                    pl.edges     = std::move(edges);
                    cf.type      = cadapp::FeatType::Chamfer;
                    cf.data      = std::move(pl);
                }
                // Base = the fld-62 body this instance dresses (box2). The
                // Replayer resolves the moved edges against it and consumes
                // it, so the dressed body REPLACES the plain copy (no
                // duplicate, no vanish). The running chain stays untouched.
                PushInput(cf, onto, cadapp::InputRole::Base);
                out.features.push_back(std::move(cf));
                standalone_ids.insert(id);
                return true;
            }
        }

        // Hole-seeded pattern: the seed (fld 1 ent's owning feat)
        // is a drill hole, not an extrude, so the extrude-footprint
        // matcher below can't see it. Wire the hole's cut tool as a
        // Tool original and pattern it onto the running body -- the
        // Replayer cut-replicates an op_kind='c' tool. ZW3D linear
        // modes: fld 10 = 0 plain (count fld 3, spacing fld 4) or
        // = 2 "fill to point" (fld 13 base -> fld 14 end, stepped at
        // fld 4 spacing). DKBA81377750 阵列1/阵列2 are fill-to-point
        // M5 hole rows.
        if (zt == "FtPtnFtr" && running_solid_id != 0)
        {
            const uint32_t seed_feat = FieldEntFeat(jf, 1);
            bool seed_is_hole = false;
            for (const auto& f : out.features) {
                if (f.id == seed_feat &&
                    f.type == cadapp::FeatType::HoleWizard) {
                    seed_is_hole = true;
                    break;
                }
            }
            if (seed_is_hole)
            {
                cadapp::FeatPayloadLinearPattern lp;
                const double sp1_mm = FieldValueById(jf, 4, 0.0);

                // PNT_TO_PNT (fld 10 = 2): fld 14 "To points"
                // carries the EXACT instance locations -- the
                // plugin now emits the full "pts" list. These rows
                // are IRREGULAR (DKBA81377750 阵列1 hits x =
                // -121.5,-107.5,30,55,90,120), so use the points
                // verbatim as offsets from the seed (fld 13 base) --
                // far more reliable than a dir/count/spacing guess.
                // Fall back to the direction/count fields only when
                // no pts list is present (older export / a genuinely
                // regular linear hole pattern).
                double     base_pt[3] = { 0.0, 0.0, 0.0 };
                const bool have_base  = FieldPoint(jf, 13, base_pt);
                std::vector<std::array<double, 3>> pts;
                {
                    auto fit = jf.find("fields");
                    if (fit != jf.end() && fit->is_array()) {
                        for (const auto& fd : *fit) {
                            if (JGet<int>(fd, "id", -1) != 14) {
                                continue;
                            }
                            auto pit = fd.find("pts");
                            if (pit != fd.end() && pit->is_array()) {
                                for (const auto& q : *pit) {
                                    if (q.is_array() && q.size() >= 3) {
                                        pts.push_back({
                                            q.at(0).get<double>(),
                                            q.at(1).get<double>(),
                                            q.at(2).get<double>() });
                                    }
                                }
                            }
                            break;
                        }
                    }
                }

                if (have_base && pts.size() >= 2)
                {
                    for (const auto& q : pts) {
                        lp.instance_offsets.push_back({
                            (q[0] - base_pt[0]) * s,
                            (q[1] - base_pt[1]) * s,
                            (q[2] - base_pt[2]) * s });
                    }
                    lp.count1 = static_cast<int>(pts.size());
                }
                else
                {
                    double dir[3] = { 1.0, 0.0, 0.0 };
                    int    cnt1   = 2;
                    double bp[3], tp[3];
                    const double f10h = FieldValueById(jf, 10, 0.0);
                    if (f10h > 1.5 && FieldPoint(jf, 13, bp) &&
                        FieldPoint(jf, 14, tp) && sp1_mm > 1e-9)
                    {
                        // fill-to-point: step from base toward end.
                        double v[3] = { tp[0]-bp[0], tp[1]-bp[1],
                                        tp[2]-bp[2] };
                        double L = std::sqrt(v[0]*v[0] + v[1]*v[1] +
                                             v[2]*v[2]);
                        if (L > 1e-9) {
                            dir[0]=v[0]/L; dir[1]=v[1]/L; dir[2]=v[2]/L;
                            cnt1 = (int)std::floor(L/sp1_mm + 1e-6) + 1;
                        }
                    }
                    else
                    {
                        // plain linear: fld 2 Dir, fld 3 count.
                        double pdir[3];
                        if (FieldDir(jf, 2, pdir)) {
                            dir[0]=pdir[0]; dir[1]=pdir[1]; dir[2]=pdir[2];
                        } else if (patj != jf.end() &&
                                   patj->contains("dir") &&
                                   patj->at("dir").is_array() &&
                                   patj->at("dir").size() == 3) {
                            dir[0]=patj->at("dir").at(0).get<double>();
                            dir[1]=patj->at("dir").at(1).get<double>();
                            dir[2]=patj->at("dir").at(2).get<double>();
                        }
                        cnt1 = (int)FieldValueById(jf, 3, 2.0);
                    }
                    lp.dir1[0]=dir[0]; lp.dir1[1]=dir[1]; lp.dir1[2]=dir[2];
                    lp.count1   = (cnt1 >= 1) ? cnt1 : 2;
                    lp.spacing1 = sp1_mm * s;
                    int cnt2 = (int)FieldValueById(jf, 6, 1.0);
                    lp.count2   = (cnt2 >= 1) ? cnt2 : 1;
                    lp.spacing2 = FieldValueById(jf, 7, 0.0) * s;
                }
                lp.fuse = true;   // combine onto running; the seed
                                  // hole tool's op_kind='c' -> cut
                cadapp::FeatureIR pf;
                pf.id   = id;
                pf.name = name;
                pf.type = cadapp::FeatType::LinearPattern;
                pf.data = std::move(lp);
                pf.ext_strings["zw_type"] = zt;
                PushInput(pf, seed_feat, cadapp::InputRole::Tool);
                PushInput(pf, running_solid_id,
                          cadapp::InputRole::Base);
                pf.ext_params["pattern_onto_running"] = 1.0;
                out.features.push_back(std::move(pf));
                running_solid_id = id;
                return true;
            }
        }

        if (zt == "FtPtnFtr" &&
            ((linear_method && has_dir) ||
             (circular_method && has_axis_pt)) &&
            running_solid_id != 0 &&
            (!extrude_xy.empty() || has_fld1_feat_ref()))
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
            // this one footprint's normal). Prefer the OWNING
            // feature id the plugin resolved (same rationale as the
            // linear seed loop below): fld 1 anchors are consumed-
            // state bbox centres and can sit far from the seed's
            // own footprint (R2900 Pattern13: 12 mm off Extrude46,
            // so match_anchor() returned 0 and the pattern was
            // silently dropped as opaque type=0). The feat id is
            // only trusted when a footprint was registered for it:
            // the circular branch derives its rotation axis from
            // that footprint's plane normal.
            double anc[3] = { 0.0, 0.0, 0.0 };
            uint32_t target = 0;
            {
                uint32_t tf = FieldEntFeat(jf, 1);
                if (tf != 0) {
                    for (const auto& b : extrude_xy) {
                        if (b.id == tf) { target = tf; break; }
                    }
                }
            }
            if (target == 0 && FieldEntAnchor(jf, 1, anc)) {
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

                // Boolean=none patterns (copies stay standalone
                // bodies): no Base link, the running body is not
                // touched and the chain tip does not advance. Set
                // by the linear branch from result_ents.n_shape.
                bool standalone = false;

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

                    // Boolean = none: the pattern's copies came out
                    // as NEW standalone shapes (result_ents.n_shape
                    // counts bodies the feature created -- a fusing
                    // pattern only grows faces on the running body,
                    // n_shape stays 0). R2900_100 Pattern9: 4x1
                    // copies -> n_shape=3, ZW3D truth holds 4 free
                    // bodies until the next boolean absorbs them.
                    {
                        int re_nshape = 0;
                        auto reit = jf.find("result_ents");
                        if (reit != jf.end() && reit->is_object()) {
                            re_nshape = JGet<int>(*reit, "n_shape", 0);
                        }
                        lp.fuse = (re_nshape <= 0);
                    }
                    standalone = !lp.fuse;

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

                if (!standalone) {
                    PushInput(pf, running_solid_id, cadapp::InputRole::Base);
                    pf.ext_params["pattern_onto_running"] = 1.0;
                }

                // fld 62 "Boolean shapes" lists the EXISTING bodies
                // ZW3D merged the pattern's copies with. A
                // STANDALONE candidate named there ceases to exist
                // independently -- wire it as Role::Operand so the
                // Replayer's input-consumption pass drops it from
                // emission. R2900 Pattern17 merges [Extrude1_Base,
                // Mirror5]: Mirror5's three funnel sheets get
                // absorbed into the (blanked) plate composite and
                // must stop being emitted (count_sheets read 9 vs
                // truth 6). Gated to standalone_ids: a chain
                // feature in fld 62 is already consumed by its
                // successor's Base link, and wiring it again only
                // churns the IR.
                {
                    std::set<uint32_t> consumed62;   // fld 62 lists
                                                     // one ent per
                                                     // merged copy;
                                                     // dedup inputs
                    bool merges_sheets = false;
                    bool merges_chain  = false;
                    auto fit62 = jf.find("fields");
                    if (fit62 != jf.end() && fit62->is_array()) {
                        for (const auto& fd : *fit62) {
                            if (JGet<int>(fd, "id", -1) != 62) {
                                continue;
                            }
                            auto eit = fd.find("ents");
                            if (eit != fd.end() && eit->is_array()) {
                                for (const auto& e : *eit) {
                                    auto ft = e.find("feat");
                                    if (ft == e.end() ||
                                        !ft->is_number_integer()) {
                                        continue;
                                    }
                                    int v = ft->get<int>();
                                    if (v <= 0 ||
                                        static_cast<uint32_t>(v) == id) {
                                        continue;
                                    }
                                    const uint32_t fv =
                                        static_cast<uint32_t>(v);
                                    if (!standalone_ids.count(fv)) {
                                        merges_chain = true;
                                        continue;
                                    }
                                    if (standalone_sheet_ids.count(fv)) {
                                        merges_sheets = true;
                                    }
                                    if (consumed62.insert(fv).second) {
                                        PushInput(pf, fv,
                                            cadapp::InputRole::Operand);
                                    }
                                }
                            }
                            break;
                        }
                    }
                    // fld 62 operand consumption handled above.
                    // (Quilt-kill logic removed: bisect showed that
                    // truth _state keeps the plate at n_blanked=0
                    // through feat 153; drop was incorrect.)
                    (void)merges_sheets;
                    (void)merges_chain;
                }

                // Record which pattern consumed each raw seed so a
                // LATER pattern copying the same seed nests THIS
                // pattern's result (the ring), not the bare seed.
                for (uint32_t sd : consumed_seeds) {
                    if (sd != 0) { seed_to_pattern[sd] = id; }
                }

                out.features.push_back(std::move(pf));
                // A standalone pattern's copies live beside the
                // body; the next feature still chains onto the
                // pre-pattern tip.
                if (!standalone) {
                    running_solid_id = id;
                } else {
                    standalone_ids.insert(id);
                }
                return true;
            }
        }
    }

    return false;
}

static bool TryBuildFtDressup(ZwBuildCtx& ctx)
{
    const json&                             jf                   = ctx.jf;
    const uint32_t                          id                   = ctx.id;
    const std::string&                      name                 = ctx.name;
    const std::string&                      zt                   = ctx.zt;
    const double                            s                    = ctx.s;
    cadapp::DocumentIR&                     out                  = ctx.out;
    uint32_t&                               running_solid_id     = ctx.running_solid_id;
    std::set<uint32_t>&                     standalone_ids       = ctx.standalone_ids;
    std::set<uint32_t>&                     standalone_sheet_ids = ctx.standalone_sheet_ids;
    std::vector<ExtrudeFootprint>&          extrude_xy           = ctx.extrude_xy;
    std::vector<PriorProfile>&              prior_profiles       = ctx.prior_profiles;
    std::unordered_map<uint32_t, uint32_t>& seed_to_pattern      = ctx.seed_to_pattern;
    const std::string&                      doc_dir              = ctx.doc_dir;
    const std::string&                      path                 = ctx.path;
    (void)id; (void)name; (void)zt; (void)s; (void)out; (void)jf;
    (void)running_solid_id; (void)standalone_ids; (void)standalone_sheet_ids;
    (void)extrude_xy; (void)prior_profiles; (void)seed_to_pattern;
    (void)doc_dir; (void)path;

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

            // Remember this dress-up so a later FtPtnFtr that patterns it
            // can re-apply it at the pattern-imaged edge positions (the
            // dress-up itself has no copyable tool solid).
            {
                ZwDressupSeed ds;
                ds.is_fillet = is_fillet;
                ds.size      = dim * s;
                ds.edges     = edges;            // scaled points; copy before move
                ctx.feat_dressup[id] = std::move(ds);
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
            return true;
        }
    }

    return false;
}

static bool TryBuildFtHoleMain(ZwBuildCtx& ctx)
{
    const json&                             jf                   = ctx.jf;
    const uint32_t                          id                   = ctx.id;
    const std::string&                      name                 = ctx.name;
    const std::string&                      zt                   = ctx.zt;
    const double                            s                    = ctx.s;
    cadapp::DocumentIR&                     out                  = ctx.out;
    uint32_t&                               running_solid_id     = ctx.running_solid_id;
    std::set<uint32_t>&                     standalone_ids       = ctx.standalone_ids;
    std::set<uint32_t>&                     standalone_sheet_ids = ctx.standalone_sheet_ids;
    std::vector<ExtrudeFootprint>&          extrude_xy           = ctx.extrude_xy;
    std::vector<PriorProfile>&              prior_profiles       = ctx.prior_profiles;
    std::unordered_map<uint32_t, uint32_t>& seed_to_pattern      = ctx.seed_to_pattern;
    const std::string&                      doc_dir              = ctx.doc_dir;
    const std::string&                      path                 = ctx.path;
    (void)id; (void)name; (void)zt; (void)s; (void)out; (void)jf;
    (void)running_solid_id; (void)standalone_ids; (void)standalone_sheet_ids;
    (void)extrude_xy; (void)prior_profiles; (void)seed_to_pattern;
    (void)doc_dir; (void)path;

    // ZW3D simple drill hole (FtHoleMain 钻孔) -> HoleWizard.
    // fld 25 "Dia (D1)" = drill diameter, fld 27 "Depth (H1)" =
    // drill depth, fld 55 "Tip" = drill-point angle (118 deg
    // standard), fld 65 the placement point on a body face. The
    // hole cuts the RUNNING body in place (like a dressup); the
    // Replayer's HoleWizard arm derives the drill axis from that
    // body's face at the point and Cuts a cylinder + conical tip.
    // Verified on DKBA81377750 (孔1 removes exactly 172 mm^3 =
    // cylinder 166.3 + 118-deg tip 5.8, matching truth). No body
    // yet, or no diameter / point -> fall through to opaque.
    if (zt == "FtHoleMain" && running_solid_id != 0)
    {
        const double dia_mm  = FieldValueById(jf, 25, 0.0);
        const double dep_mm  = FieldValueById(jf, 27, 0.0);
        const double tip_deg = FieldValueById(jf, 55, 118.0);
        double       hpt[3]  = { 0.0, 0.0, 0.0 };
        const bool   has_pt  = FieldPoint(jf, 65, hpt) ||
                               FieldPoint(jf, 98, hpt);
        if (dia_mm > 1e-9 && has_pt)
        {
            cadapp::FeatPayloadHoleWizard pl;
            pl.diameter    = dia_mm * s;
            pl.depth       = dep_mm * s;
            pl.through_all = (dep_mm <= 1e-9);
            cadapp::FeatureIR hf;
            hf.id   = id;
            hf.name = name;
            hf.type = cadapp::FeatType::HoleWizard;
            hf.data = std::move(pl);
            hf.ext_strings["zw_type"]     = zt;
            hf.ext_params["hole_px"]      = hpt[0] * s;
            hf.ext_params["hole_py"]      = hpt[1] * s;
            hf.ext_params["hole_pz"]      = hpt[2] * s;
            hf.ext_params["hole_tip_deg"] = tip_deg;
            // Base = the running body the hole cuts in place.
            PushInput(hf, running_solid_id,
                      cadapp::InputRole::Base);
            out.features.push_back(std::move(hf));
            running_solid_id = id;
            return true;
        }
        // incomplete record -> fall through to opaque.
    }

    return false;
}

static bool TryBuildFtCrossTrim(ZwBuildCtx& ctx)
{
    const json&                             jf                   = ctx.jf;
    const uint32_t                          id                   = ctx.id;
    const std::string&                      name                 = ctx.name;
    const std::string&                      zt                   = ctx.zt;
    const double                            s                    = ctx.s;
    cadapp::DocumentIR&                     out                  = ctx.out;
    uint32_t&                               running_solid_id     = ctx.running_solid_id;
    std::set<uint32_t>&                     standalone_ids       = ctx.standalone_ids;
    std::set<uint32_t>&                     standalone_sheet_ids = ctx.standalone_sheet_ids;
    std::vector<ExtrudeFootprint>&          extrude_xy           = ctx.extrude_xy;
    std::vector<PriorProfile>&              prior_profiles       = ctx.prior_profiles;
    std::unordered_map<uint32_t, uint32_t>& seed_to_pattern      = ctx.seed_to_pattern;
    const std::string&                      doc_dir              = ctx.doc_dir;
    const std::string&                      path                 = ctx.path;
    (void)id; (void)name; (void)zt; (void)s; (void)out; (void)jf;
    (void)running_solid_id; (void)standalone_ids; (void)standalone_sheet_ids;
    (void)extrude_xy; (void)prior_profiles; (void)seed_to_pattern;
    (void)doc_dir; (void)path;

    // ZW3D cross trim (FtCrossTrim 修剪) -> FeatPayloadTrim (cross).
    // fld 5 "Surface 1" / fld 6 "Surface 2": each ent's "feat" names
    // a surface body and its "anchor" the point ON the kept region.
    // The two surfaces are mutually trimmed at their intersection
    // (OCCT computes it), each keeping the fragment under its anchor;
    // both survive in place, so both lineages redirect to the result.
    // D30_OUTDOOR 修剪1: Surface 3 x Surface 5 (the first divergence).
    if (zt == "FtCrossTrim")
    {
        auto is_built = [&](uint32_t fid) -> bool {
            for (const auto& f : out.features) {
                if (f.id == fid) {
                    return f.type != cadapp::FeatType::Unknown;
                }
            }
            return false;
        };
        auto read_surf = [&](int fld, uint32_t& fo,
                             double anc[3]) -> bool {
            auto fit = jf.find("fields");
            if (fit == jf.end() || !fit->is_array()) { return false; }
            for (const auto& fd : *fit) {
                if (JGet<int>(fd, "id", -1) != fld) { continue; }
                auto eit = fd.find("ents");
                if (eit == fd.end() || !eit->is_array() ||
                    eit->empty()) {
                    return false;
                }
                const auto& e  = eit->front();
                auto        ft = e.find("feat");
                if (ft == e.end() || !ft->is_number_integer()) {
                    return false;
                }
                fo = static_cast<uint32_t>(ft->get<int>());
                auto at = e.find("anchor");
                if (at != e.end() && at->is_array() &&
                    at->size() >= 3) {
                    anc[0] = at->at(0).get<double>();
                    anc[1] = at->at(1).get<double>();
                    anc[2] = at->at(2).get<double>();
                }
                return fo != 0;
            }
            return false;
        };
        uint32_t s1 = 0, s2 = 0;
        double   a1[3] = { 0, 0, 0 }, a2[3] = { 0, 0, 0 };
        if (read_surf(5, s1, a1) && read_surf(6, s2, a2) &&
            is_built(s1) && is_built(s2))
        {
            cadapp::FeatPayloadTrim pl;
            pl.cross  = true;
            pl.mutual = true;
            for (int k = 0; k < 3; ++k) {
                pl.keep_pt[k]  = a1[k] * s;
                pl.anchor2[k]  = a2[k] * s;
            }
            cadapp::FeatureIR tf;
            tf.id   = id;
            tf.name = name;
            tf.type = cadapp::FeatType::Trim;
            tf.data = std::move(pl);
            tf.ext_strings["zw_type"] = zt;
            const uint32_t s1_tip = ResolveTip(ctx.lineage_tip, s1);
            const uint32_t s2_tip = ResolveTip(ctx.lineage_tip, s2);
            PushInput(tf, s1_tip, cadapp::InputRole::Base);
            if (s2_tip != s1_tip) {
                PushInput(tf, s2_tip, cadapp::InputRole::Tool);
            }
            out.features.push_back(std::move(tf));
            ctx.lineage_tip[s1_tip] = id;
            ctx.lineage_tip[s2_tip] = id;
            if (running_solid_id == s1_tip ||
                running_solid_id == s2_tip ||
                running_solid_id == s1 || running_solid_id == s2) {
                running_solid_id = id;
            }
            return true;
        }
        // incomplete record -> fall through to opaque.
    }

    return false;
}

static bool TryBuildFtSolidSoloTrm(ZwBuildCtx& ctx)
{
    const json&                             jf                   = ctx.jf;
    const uint32_t                          id                   = ctx.id;
    const std::string&                      name                 = ctx.name;
    const std::string&                      zt                   = ctx.zt;
    const double                            s                    = ctx.s;
    cadapp::DocumentIR&                     out                  = ctx.out;
    uint32_t&                               running_solid_id     = ctx.running_solid_id;
    std::set<uint32_t>&                     standalone_ids       = ctx.standalone_ids;
    std::set<uint32_t>&                     standalone_sheet_ids = ctx.standalone_sheet_ids;
    std::vector<ExtrudeFootprint>&          extrude_xy           = ctx.extrude_xy;
    std::vector<PriorProfile>&              prior_profiles       = ctx.prior_profiles;
    std::unordered_map<uint32_t, uint32_t>& seed_to_pattern      = ctx.seed_to_pattern;
    const std::string&                      doc_dir              = ctx.doc_dir;
    const std::string&                      path                 = ctx.path;
    (void)id; (void)name; (void)zt; (void)s; (void)out; (void)jf;
    (void)running_solid_id; (void)standalone_ids; (void)standalone_sheet_ids;
    (void)extrude_xy; (void)prior_profiles; (void)seed_to_pattern;
    (void)doc_dir; (void)path;

    // ZW3D trim (FtSolidSoloTrm 修剪) -> FeatPayloadTrim. fld 1
    // "Base B" names the body being trimmed (its ent carries the
    // owning feature's JSON id in "feat"), fld 2 "Trimming T" the
    // trimming face(s) -- each ent's "feat" names the sheet body
    // owning them -- and fld 20 is the keep-side witness: "pt"
    // sits on the trimming surface and "dir" points INTO the kept
    // half. The Replayer splits base by tool and keeps the
    // witnessed side; both inputs are consumed (ZW3D removes the
    // trimming sheet from the part). 02-ear: 修剪1 cuts UV曲面1's
    // skin with 拉伸1's extruded band (truth flux -1716 -> -907).
    if (zt == "FtSolidSoloTrm")
    {
        auto fields_it = jf.find("fields");
        const bool has_fields =
            (fields_it != jf.end() && fields_it->is_array());

        auto is_built = [&](uint32_t fid) -> bool {
            for (const auto& f : out.features) {
                if (f.id == fid) {
                    return f.type != cadapp::FeatType::Unknown;
                }
            }
            return false;
        };

        uint32_t              base_fid = 0;
        std::vector<uint32_t> tool_fids;
        double kpt[3]  = { 0.0, 0.0, 0.0 };
        double kdir[3] = { 0.0, 0.0, 0.0 };
        bool   has_keep = false;
        if (has_fields) {
            for (const auto& fd : *fields_it) {
                const int fld = JGet<int>(fd, "id", -1);
                if (fld == 1 || fld == 2)
                {
                    auto eit = fd.find("ents");
                    if (eit == fd.end() || !eit->is_array()) {
                        continue;
                    }
                    for (const auto& e : *eit) {
                        auto ft = e.find("feat");
                        if (ft == e.end() ||
                            !ft->is_number_integer()) {
                            continue;
                        }
                        uint32_t t =
                            static_cast<uint32_t>(ft->get<int>());
                        if (t == 0 || !is_built(t)) { continue; }
                        if (fld == 1) {
                            if (base_fid == 0) { base_fid = t; }
                        } else {
                            if (t == base_fid) { continue; }
                            bool dup = false;
                            for (uint32_t x : tool_fids) {
                                if (x == t) { dup = true; break; }
                            }
                            if (!dup) { tool_fids.push_back(t); }
                        }
                    }
                }
                else if (fld == 20)
                {
                    auto pt = fd.find("pt");
                    auto dr = fd.find("dir");
                    if (pt != fd.end() && pt->is_array() &&
                        pt->size() >= 3 &&
                        dr != fd.end() && dr->is_array() &&
                        dr->size() >= 3)
                    {
                        for (int k = 0; k < 3; ++k) {
                            kpt[k]  = pt->at(k).get<double>();
                            kdir[k] = dr->at(k).get<double>();
                        }
                        has_keep = std::fabs(kdir[0]) +
                                   std::fabs(kdir[1]) +
                                   std::fabs(kdir[2]) > 1e-12;
                    }
                }
            }
        }

        if (base_fid != 0 && !tool_fids.empty() && has_keep)
        {
            cadapp::FeatPayloadTrim pl;
            for (int k = 0; k < 3; ++k) {
                pl.keep_pt[k]  = kpt[k] * s;
                pl.keep_dir[k] = kdir[k];
            }
            // fld8=1: mutual trim -- the tool is trimmed by
            // the base too and its remnant survives (修剪3/4
            // truth keeps a tool sliver as a visible body);
            // fld8=0: tool fully consumed (修剪1/2).
            pl.mutual = FieldValueById(jf, 8, 0.0) > 0.5;
            cadapp::FeatureIR tf;
            tf.id   = id;
            tf.name = name;
            tf.type = cadapp::FeatType::Trim;
            tf.data = std::move(pl);
            tf.ext_strings["zw_type"] = zt;
            // Redirect every ref to its current lineage tip:
            // ZW3D names the root body even after in-place ops,
            // so a raw ref reaches stale geometry and orphans
            // the intervening op as a phantom body.
            const uint32_t base_tip = ResolveTip(ctx.lineage_tip, base_fid);
            PushInput(tf, base_tip, cadapp::InputRole::Base);
            std::set<uint32_t> wired{ base_tip };
            for (uint32_t t : tool_fids) {
                const uint32_t tt = ResolveTip(ctx.lineage_tip, t);
                if (wired.insert(tt).second) {
                    PushInput(tf, tt, cadapp::InputRole::Tool);
                }
            }
            out.features.push_back(std::move(tf));
            // The trim supersedes its base lineage in place.
            ctx.lineage_tip[base_tip] = id;
            if (running_solid_id == base_tip ||
                running_solid_id == base_fid) {
                running_solid_id = id;
            }
            return true;
        }
        // incomplete record (no feat backrefs / no witness):
        // fall through to the opaque path below.
    }

    return false;
}

static bool TryBuildFtSew(ZwBuildCtx& ctx)
{
    const json&                             jf                   = ctx.jf;
    const uint32_t                          id                   = ctx.id;
    const std::string&                      name                 = ctx.name;
    const std::string&                      zt                   = ctx.zt;
    const double                            s                    = ctx.s;
    cadapp::DocumentIR&                     out                  = ctx.out;
    uint32_t&                               running_solid_id     = ctx.running_solid_id;
    std::set<uint32_t>&                     standalone_ids       = ctx.standalone_ids;
    std::set<uint32_t>&                     standalone_sheet_ids = ctx.standalone_sheet_ids;
    std::vector<ExtrudeFootprint>&          extrude_xy           = ctx.extrude_xy;
    std::vector<PriorProfile>&              prior_profiles       = ctx.prior_profiles;
    std::unordered_map<uint32_t, uint32_t>& seed_to_pattern      = ctx.seed_to_pattern;
    const std::string&                      doc_dir              = ctx.doc_dir;
    const std::string&                      path                 = ctx.path;
    (void)id; (void)name; (void)zt; (void)s; (void)out; (void)jf;
    (void)running_solid_id; (void)standalone_ids; (void)standalone_sheet_ids;
    (void)extrude_xy; (void)prior_profiles; (void)seed_to_pattern;
    (void)doc_dir; (void)path;

    // ZW3D sew (CdShapeSew 缝合) and combine-add (FtBoolSoloAdd
    // 组合-添加) -> FeatPayloadSew. fld 1 = Base body, fld 2 =
    // the bodies sewn/added into it (feat backrefs), fld 4 =
    // sew tolerance (CdShapeSew only; combine uses the same
    // 0.01 mm default). Sheet operands sew + solidify when the
    // shell closes; solid operands degrade to a plain fuse at
    // replay time. 02-ear: 缝合3 merges the dome skins into the
    // main skin, 组合1_添加 closes wall band + dome into the
    // final solid (truth flux -304 -> +4.5).
    if (zt == "CdShapeSew" || zt == "FtBoolSoloAdd")
    {
        auto fields_it = jf.find("fields");
        const bool has_fields =
            (fields_it != jf.end() && fields_it->is_array());

        auto is_built = [&](uint32_t fid) -> bool {
            for (const auto& f : out.features) {
                if (f.id == fid) {
                    return f.type != cadapp::FeatType::Unknown;
                }
            }
            return false;
        };

        uint32_t              base_fid = 0;
        std::vector<uint32_t> tool_fids;
        if (has_fields) {
            for (const auto& fd : *fields_it) {
                const int fld = JGet<int>(fd, "id", -1);
                if (fld != 1 && fld != 2) { continue; }
                auto eit = fd.find("ents");
                if (eit == fd.end() || !eit->is_array()) {
                    continue;
                }
                for (const auto& e : *eit) {
                    auto ft = e.find("feat");
                    if (ft == e.end() ||
                        !ft->is_number_integer()) {
                        continue;
                    }
                    uint32_t t =
                        static_cast<uint32_t>(ft->get<int>());
                    if (t == 0 || !is_built(t)) { continue; }
                    if (fld == 1) {
                        if (base_fid == 0) { base_fid = t; }
                    } else {
                        if (t == base_fid) { continue; }
                        bool dup = false;
                        for (uint32_t x : tool_fids) {
                            if (x == t) { dup = true; break; }
                        }
                        if (!dup) { tool_fids.push_back(t); }
                    }
                }
            }
        }

        if (base_fid != 0 && !tool_fids.empty())
        {
            cadapp::FeatPayloadSew pl;
            // fld4 is the dialog's sew tolerance in file units
            // (mm); both features default to ZW3D's 0.01 mm.
            const double tol_mm = FieldValueById(jf, 4, 0.01);
            pl.tolerance = (tol_mm > 0.0 ? tol_mm : 0.01) * s;
            cadapp::FeatureIR sf;
            sf.id   = id;
            sf.name = name;
            sf.type = cadapp::FeatType::Sew;
            sf.data = std::move(pl);
            sf.ext_strings["zw_type"] = zt;
            // Redirect every ref to its current lineage tip
            // (see the trim handler / lineage_tip note above):
            // ZW3D names the root body even after in-place ops.
            const uint32_t base_tip = ResolveTip(ctx.lineage_tip, base_fid);
            PushInput(sf, base_tip, cadapp::InputRole::Base);
            std::set<uint32_t> wired{ base_tip };
            for (uint32_t t : tool_fids) {
                const uint32_t tt = ResolveTip(ctx.lineage_tip, t);
                if (wired.insert(tt).second) {
                    PushInput(sf, tt, cadapp::InputRole::Tool);
                }
            }
            out.features.push_back(std::move(sf));
            // The sew supersedes its base lineage in place.
            ctx.lineage_tip[base_tip] = id;
            if (running_solid_id == base_tip ||
                running_solid_id == base_fid) {
                running_solid_id = id;
            }
            return true;
        }
        // incomplete record: fall through to the opaque path.
    }

    return false;
}

static bool TryBuildFtMirrorFtr(ZwBuildCtx& ctx)
{
    const json&                             jf                   = ctx.jf;
    const uint32_t                          id                   = ctx.id;
    const std::string&                      name                 = ctx.name;
    const std::string&                      zt                   = ctx.zt;
    const double                            s                    = ctx.s;
    cadapp::DocumentIR&                     out                  = ctx.out;
    uint32_t&                               running_solid_id     = ctx.running_solid_id;
    std::set<uint32_t>&                     standalone_ids       = ctx.standalone_ids;
    std::set<uint32_t>&                     standalone_sheet_ids = ctx.standalone_sheet_ids;
    std::vector<ExtrudeFootprint>&          extrude_xy           = ctx.extrude_xy;
    std::vector<PriorProfile>&              prior_profiles       = ctx.prior_profiles;
    std::unordered_map<uint32_t, uint32_t>& seed_to_pattern      = ctx.seed_to_pattern;
    const std::string&                      doc_dir              = ctx.doc_dir;
    const std::string&                      path                 = ctx.path;
    (void)id; (void)name; (void)zt; (void)s; (void)out; (void)jf;
    (void)running_solid_id; (void)standalone_ids; (void)standalone_sheet_ids;
    (void)extrude_xy; (void)prior_profiles; (void)seed_to_pattern;
    (void)doc_dir; (void)path;

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
            // A ZW3D feature-mirror reflects the CUMULATIVE geometry
            // the mirrored features added, INCLUDING their fillets /
            // chamfers / patterns. The per-original-tool path
            // (AddMirroredOriginals) mirrors each feature's raw tool
            // solid and silently drops dressups (a fillet has no tool
            // solid) -- so the mirror side comes out MISSING its
            // rounds. Prefer the Replayer's DELTA path: reflect
            // (running body) minus (body BEFORE the first mirrored
            // feature), which carries every detail. That pre-body is
            // the Base input of the lowest-id reconstructed mirrored
            // feature (an opaque feature adds nothing, so the next
            // built feature's base is the same body state). Wired as
            // Role::PatternTarget (-> Replayer's body_target); since
            // ResolvePatternInputs short-circuits on Tool originals,
            // the delta and per-tool wirings are mutually exclusive.
            // Assumes the mirrored set is the contiguous run of solid
            // features since that pre-body (true for R2900_100's
            // Mirror1/2); falls back to per-tool when no pre-body is
            // resolvable.
            uint32_t pre_body = 0;
            {
                uint32_t lowest = 0xFFFFFFFFu;
                for (uint32_t t : mir_tools) {
                    if (t < lowest) { lowest = t; }
                }
                for (const auto& f : out.features) {
                    if (f.id != lowest) { continue; }
                    for (size_t k = 0; k < f.input_feature_ids.size(); ++k) {
                        cadapp::InputRole r =
                            (k < f.input_roles.size())
                                ? f.input_roles[k]
                                : cadapp::InputRole::Base;
                        if (r == cadapp::InputRole::Base) {
                            pre_body = f.input_feature_ids[k];
                            break;
                        }
                    }
                    break;
                }
            }
            // Mirrors whose authored contribution is TINY
            // (result_ents: no new shapes, a handful of faces)
            // take the per-tool path. The delta path reflects
            // cut(body@post, body@pre), and when the running
            // body carries standalone pattern instances those
            // survive the cut untouched and get mirrored
            // wholesale -- R2900's Mirror8 is +6 faces in the
            // truth, but its delta tool came out 95 solids /
            // 2373 faces and the fuse ground >1000 s of CPU.
            // Mirroring the originals' raw tool solids loses
            // their dressups, which at this scale is a couple
            // of faces -- the honest trade. Big mirrors
            // (Mirror1: +298 faces) keep the delta path so
            // their fillets / chamfers come along.
            bool small_mirror = false;
            int  re_nshape    = 0;
            {
                auto re = jf.find("result_ents");
                if (re != jf.end()) {
                    small_mirror =
                        JGet<int>(*re, "n_face", 0) <= 50;
                    re_nshape = JGet<int>(*re, "n_shape", 0);
                }
            }
            // Boolean=none mirror: the copies came out as NEW
            // standalone bodies (n_shape > 0), the running body
            // is untouched. R2900's Mirror5/Mirror6 copy
            // Revolve4's three open SHEETS -- ZW3D keeps them
            // free forever. Wire Tools only: no Base, and the
            // chain tip does not advance, exactly like the
            // standalone pattern case. Gated to small mirrors:
            // a mixed mirror that both spawns a body AND grows
            // the running one (Mirror9: n_shape=1, n_face=212)
            // keeps the delta path -- losing its one free copy
            // is the lesser error than dropping 200 fused
            // faces.
            const bool standalone_mirror =
                re_nshape > 0 && small_mirror;
            if (standalone_mirror) {
                for (uint32_t t : mir_tools) {
                    PushInput(mf, t, cadapp::InputRole::Tool);
                }
                // fld 10 = COPY/MOVE method for FtMirrorFtr:
                // 0 = MOVE (the source is REFLECTED to the mirror
                // position and the original is consumed -- net
                // body count conserved), 1 = COPY (original kept +
                // reflected copy added). R2900 Mirror5 (fld10=0)
                // MOVES Revolve4's funnel sheets across y=50.887;
                // truth _state is unchanged (n_shape/n_face
                // conserved). Without this the source funnels stay
                // emitted alongside the copy -> phantom volume.
                // NB fld 10 is the linear/circular discriminator
                // for FtPtnFtr -- different meaning, so this read
                // is confined to the mirror block.
                if (FieldValueById(jf, 10, 1.0) < 0.5) {
                    mf.ext_params["zw_mirror_move"] = 1.0;
                }
                out.features.push_back(std::move(mf));
                standalone_ids.insert(id);
                // Small standalone mirrors copy open SHEET sets
                // (R2900 Mirror5/6 mirror Revolve4's funnels);
                // a later quilt that merges the solid chain with
                // one of these kills the chain's visibility.
                standalone_sheet_ids.insert(id);
                return true;
            }
            if (!small_mirror && pre_body != 0 && pre_body != id) {
                PushInput(mf, pre_body, cadapp::InputRole::PatternTarget);
                // Bound the delta: the SECOND PatternTarget is the
                // body tip at the HIGHEST mirrored feature, so the
                // Replayer reflects cut(post, pre) -- the mirrored
                // features' own contribution -- instead of
                // cut(running body, pre). Without it a mirror of
                // far-upstream features reflects every feature
                // built since. When the mirrored set IS the
                // contiguous tail before the mirror, post == the
                // mirror's own base and the lowering is identical
                // to the unbounded form.
                uint32_t highest = 0;
                for (uint32_t t : mir_tools) {
                    if (t > highest) { highest = t; }
                }
                if (highest != 0 && highest != pre_body) {
                    PushInput(mf, highest,
                              cadapp::InputRole::PatternTarget);
                }
            } else {
                // Fallback: per-original-tool mirror (drops dressups,
                // but better than the whole-body mirror that an empty
                // input set would trigger).
                for (uint32_t t : mir_tools) {
                    PushInput(mf, t, cadapp::InputRole::Tool);
                }
            }
            PushInput(mf, running_solid_id, cadapp::InputRole::Base);
            out.features.push_back(std::move(mf));
            running_solid_id = id;
            return true;
        }
    }

    return false;
}

static bool TryBuildGeomFallback(ZwBuildCtx& ctx)
{
    const json&                             jf                   = ctx.jf;
    const uint32_t                          id                   = ctx.id;
    const std::string&                      name                 = ctx.name;
    const std::string&                      zt                   = ctx.zt;
    const double                            s                    = ctx.s;
    cadapp::DocumentIR&                     out                  = ctx.out;
    uint32_t&                               running_solid_id     = ctx.running_solid_id;
    std::set<uint32_t>&                     standalone_ids       = ctx.standalone_ids;
    std::set<uint32_t>&                     standalone_sheet_ids = ctx.standalone_sheet_ids;
    std::vector<ExtrudeFootprint>&          extrude_xy           = ctx.extrude_xy;
    std::vector<PriorProfile>&              prior_profiles       = ctx.prior_profiles;
    std::unordered_map<uint32_t, uint32_t>& seed_to_pattern      = ctx.seed_to_pattern;
    const std::string&                      doc_dir              = ctx.doc_dir;
    const std::string&                      path                 = ctx.path;
    (void)id; (void)name; (void)zt; (void)s; (void)out; (void)jf;
    (void)running_solid_id; (void)standalone_ids; (void)standalone_sheet_ids;
    (void)extrude_xy; (void)prior_profiles; (void)seed_to_pattern;
    (void)doc_dir; (void)path;

    // Cumulative-body bake (plugin CAX_BAKE_CUMULATIVE): an
    // opaque IN-PLACE feature whose params the plugin couldn't
    // read or cax can't model -- sheet-metal flange/tab/punch
    // (CdSmd*/Smd*), API-opaque ___凸包 bosses -- but whose WHOLE
    // post-feature body the plugin baked to a STEP. Load it as the
    // new RUNNING body: consume the previous tip and advance
    // running_solid_id, so downstream reconstructable features
    // (cuts) replay on the exact body. Reconstructable arms above
    // already returned, so only un-reconstructable features
    // reach here (a pattern whose seed is one of these falls
    // through too -- its seed is neither extrude nor hole).
    {
        auto bc  = jf.find("baked_cumulative");
        auto geo = jf.find("geometry");
        const bool cumul = (bc != jf.end() && bc->is_boolean() &&
                            bc->get<bool>());
        if (cumul && geo != jf.end() && geo->is_string() &&
            !geo->get<std::string>().empty())
        {
            cadapp::FeatPayloadBakedShape pl;
            cadapp::FeatureIR f;
            f.id   = id;
            f.type = cadapp::FeatType::BakedShape;
            f.name = name;
            f.data = std::move(pl);
            f.ext_strings["zw_type"]     = zt;
            f.ext_strings["zw_geometry"] = ResolveFeatStepPath(
                path, doc_dir, geo->get<std::string>(), id);
            // The cumulative body subsumes EVERYTHING built so
            // far, so it must consume not just the running tip but
            // every standalone body too (e.g. the own-baked 平钣
            // flat-blank base) -- otherwise they double-count (2
            // solids, 2x volume) and the oversized flat blank
            // inflates the bbox. Redirect each lineage here.
            std::set<uint32_t> consumed_now;
            if (running_solid_id != 0) {
                const uint32_t prev = ResolveTip(ctx.lineage_tip, running_solid_id);
                if (consumed_now.insert(prev).second) {
                    PushInput(f, prev, cadapp::InputRole::Base);
                }
                ctx.lineage_tip[prev] = id;
            }
            for (uint32_t sid : standalone_ids) {
                const uint32_t t = ResolveTip(ctx.lineage_tip, sid);
                if (consumed_now.insert(t).second) {
                    PushInput(f, t, cadapp::InputRole::Operand);
                }
                ctx.lineage_tip[sid] = id;
            }
            standalone_ids.clear();
            out.features.push_back(std::move(f));
            running_solid_id = id;
            return true;
        }
    }

    // Authored-geometry fallback: the plugin dumps a per-feature
    // STEP for any feature whose result is a standalone SHAPE
    // entity (result_ents.n_shape >= 1 -- e.g. a base feature
    // opening a new body). When the parametric arms above could
    // not reconstruct the feature, that STEP is still the exact
    // authored geometry -- surface it as a BakedShape body root
    // instead of dropping the feature (R2900's Revolve6_Base
    // ring, whose reference-curve profile doesn't close).
    // running_solid_id is NOT advanced: the shape is an
    // independent body and the main chain must keep fusing onto
    // the body it was already building.
    {
        auto geo = jf.find("geometry");
        int  n_shape = 0;
        auto re = jf.find("result_ents");
        if (re != jf.end()) {
            n_shape = JGet<int>(*re, "n_shape", 0);
        }
        if (geo != jf.end() && geo->is_string() &&
            !geo->get<std::string>().empty() && n_shape >= 1)
        {
            cadapp::FeatPayloadBakedShape pl;
            cadapp::FeatureIR f;
            f.id   = id;
            f.type = cadapp::FeatType::BakedShape;
            f.name = name;
            f.data = std::move(pl);
            f.ext_strings["zw_type"]     = zt;
            f.ext_strings["zw_geometry"] = ResolveFeatStepPath(
                path, doc_dir, geo->get<std::string>(), id);
            out.features.push_back(std::move(f));
            standalone_ids.insert(id);
            return true;
        }
    }

    return false;
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

    // Bodies the part keeps BLANKED in its final state (plugin writes
    // them from cvxEntIsBlanked over the end-state shape list). The
    // history builds them, the visible part -- and ZW3D's own truth
    // STEP -- excludes them; the Replayer drops the matching solids at
    // emission. R2900: the Pattern17 plate+funnel composite (~46k mm^3)
    // is blanked, and replaying it visible was a +19% volume phantom.
    if (auto hb = doc.find("hidden_bodies");
        hb != doc.end() && hb->is_array())
    {
        for (const auto& b : *hb)
        {
            auto bx = b.find("bbox");
            if (bx == b.end() || !bx->is_array() || bx->size() < 6) {
                continue;
            }
            cadapp::HiddenBodyIR h;
            for (int k = 0; k < 3; ++k) {
                h.bbox_min[k] = bx->at(k).get<double>() * s;
                h.bbox_max[k] = bx->at(k + 3).get<double>() * s;
            }
            out.hidden_bodies.push_back(h);
        }
    }

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

    // Features emitted as STANDALONE bodies (Boolean=none patterns, sheet
    // mirrors, baked-shape roots): they own a live output candidate that
    // is not the running chain. A later feature whose fld 62 "Boolean
    // shapes" names one of these merged it away -- only those get the
    // Role::Operand consumption link (a chain feature in fld 62 is
    // already consumed by its successor's Base link; wiring it again
    // would only churn the IR).
    std::set<uint32_t> standalone_ids;

    // The subset of standalone_ids that are SHEET bodies (standalone
    // mirrors of open shells). A pattern whose fld 62 merges the running
    // SOLID chain with one of these performs ZW3D's quilt: the result is
    // a dead non-manifold composite that the visible part -- and the
    // truth STEP -- excludes forever (R2900 Pattern17 quilts the plate
    // with Mirror5's funnel sheets at feat 100; the 43.5k mm^3 plate
    // never reappears in any later state). The reader marks the pattern
    // with ext_param quilt_kill_running; the Replayer drops the solid
    // containing the quilted sheets at emission.
    std::set<uint32_t> standalone_sheet_ids;

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
    std::vector<ExtrudeFootprint> extrude_xy;

    // Profiles of already-processed extrudes, for the region-pick hole
    // import: a later extrude whose ref loop matches one of these outer
    // loops pulls the donor's inner loops in as holes. Pointers into the
    // parsed document json (stable for the lifetime of ReadFile).
    std::vector<PriorProfile> prior_profiles;

    // Records which prior pattern consumed a given seed feature, keyed by the
    // seed's feature id -> the pattern's feature id. Lets a LATER pattern that
    // copies the same seed nest the prior pattern's RESULT (e.g. Pattern4 over
    // Pattern3's ring) instead of just the bare seed: the seed id is mapped to
    // the pattern so the outer pattern's Tool becomes the inner pattern, which
    // the Replayer lowers as linear_pattern(circular_pattern(seed)). Overwritten
    // by the latest consumer, so a reference resolves to the most recent prior
    // pattern (processed in feature order).
    std::unordered_map<uint32_t, uint32_t> seed_to_pattern;

    // Reconstructed dress-ups (fillet/chamfer) keyed by feature id, so a
    // later FtPtnFtr that patterns a dress-up seed can re-apply it at the
    // pattern-imaged edges instead of dropping it (a dress-up has no
    // copyable tool solid).
    std::unordered_map<uint32_t, ZwDressupSeed> feat_dressup;

    // Body-lineage tip per root feature id, for in-place sew/trim/combine
    // ops that name the body's ROOT no matter how many ops ran since (see
    // the lineage_tip note in ZwBuildCtx / ResolveTip).
    std::unordered_map<uint32_t, uint32_t> lineage_tip;

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

                ZwBuildCtx ctx{ jf, id, name, zt, s, out,
                                running_solid_id, standalone_ids,
                                standalone_sheet_ids, extrude_xy,
                                prior_profiles, seed_to_pattern,
                                feat_dressup, lineage_tip,
                                doc_dir, path };

                // ZW-format feature builders, tried in order: each inspects
                // ctx.zt and either reconstructs the feature (-> true, on to
                // the next feature) or declines (-> false). Falling through
                // every arm leaves the feature for the opaque path below.
                if (TryBuildCdGeomCopy(ctx))      { continue; }
                if (TryBuildFtAllExt(ctx))        { continue; }
                if (TryBuildFtAllCyl(ctx))        { continue; }
                if (TryBuildFtAllSwp1(ctx))       { continue; }
                if (TryBuildFtAllRev(ctx))        { continue; }
                if (TryBuildFtPtnFtr(ctx))        { continue; }
                if (TryBuildFtDressup(ctx))       { continue; }
                if (TryBuildFtHoleMain(ctx))      { continue; }
                if (TryBuildFtCrossTrim(ctx))     { continue; }
                if (TryBuildFtSolidSoloTrm(ctx))  { continue; }
                if (TryBuildFtSew(ctx))           { continue; }
                if (TryBuildFtMirrorFtr(ctx))     { continue; }
                if (TryBuildGeomFallback(ctx))    { continue; }

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
        *err_msg = "ZwReader: not built (nlohmann/json was missing at CMake "
                   "configure time; run: git submodule update --init "
                   "thirdparty/nlohmann, then RE-RUN cmake configure and "
                   "rebuild -- pulling the submodule alone does not "
                   "re-enable the reader in an already-configured build)";
    }
    return false;
}

} // namespace cadcvt

#endif // CAX_ZW_OK
    