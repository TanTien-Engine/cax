// ============================================================
// tools/zw_verify/main.cpp
//
// Per-file "zero diff" gate + per-feature bisect for the ZW3D
// conversion pipeline.
//
// The zw_export plugin writes, per part:
//   <part>.cax.json -- the parametric history snapshot cadcvt replays
//   <part>.cax.step -- ZW3D's own STEP export of the finished body,
//                      i.e. the authoritative TRUTH geometry
//   per-feature "_state" blocks (newer plugin builds): cumulative
//   shape/face/edge counts + bbox (+ area/volume with CAX_FEAT_STATE=2)
//   of the WHOLE part right after each feature executed -- the per-step
//   truth this tool bisects against.
//
// Modes:
//   zw_verify <json>                      full verify vs .cax.step truth
//   zw_verify <json> --max-feat K         replay features 1..K only;
//                                         prints a STATE line, compares
//                                         vs --step <truth> when given
//   zw_verify <json> --states a,b,c|all   prefix-replay several K's;
//                                         PROBE-compares each against
//                                         the snapshot's _state truth
//   zw_verify <json> --bisect             binary-search the FIRST
//                                         feature whose replayed state
//                                         diverges from its _state truth
//   ... --dump out.step                   also write the replayed
//                                         (possibly truncated) body as
//                                         STEP, scaled back to mm
//
// Checks (full verify), in escalating cost:
//   - solid / shell / face / edge / vertex counts
//   - axis-aligned bounding box (rel. to truth bbox diagonal)
//   - volume + surface area (relative)
//   - per-face matching, both directions: nearest-by-centroid within
//     tol, normal alignment, area agreement. Unmatched faces are the
//     classic "feature replayed wrong / face fragmented" signal.
//
// Output is line-oriented and machine-parsable:
//   INFO  <k>=<v> ...
//   CHECK <name> <ok|bad> <details>
//   STATE feat=<K> <metrics>             replayed prefix state
//   TRUTHSTATE feat=<K> <metrics>        plugin-recorded state (m-scaled)
//   PROBE feat=<K> verdict=<good|bad> <details>
//   BISECT first_bad=<K> last_good=<J> name=<...> zw_type=<...>
//   VERDICT <PASS|FAIL> <json path>
// Exit: 0 PASS / bisect completed, 1 FAIL, 2 usage/io.
//
// Units: the replayed body is in metres when ZwReader auto-resolves the
// snapshot's "length_unit":"mm" (scale 0.001); the truth STEP is read
// by OCCT in mm, and _state metrics are recorded by the plugin in mm.
// Both are scaled by the reader's UnitScale() so every comparison runs
// in the same space. CAX_ZW_SCALE1 forces 1.0 (mm) on both, mirroring
// the editor's import.
//
// Bisect caveat: divergence is assumed monotonic (once a prefix is bad,
// longer prefixes stay bad). A later feature CAN mask an earlier bug
// (e.g. a cut removing the bad region); when in doubt run --states all
// for the exhaustive linear sweep.
// ============================================================

#include "cadcvt_c/reader/ZwReader.h"
#include "cadapp_c/emitter/Replayer.h"
#include "cadapp_c/resolve/TopoGeomUtils.h"
#include "brepkit_c/TopoAlgo.h"
#include "brepkit_c/TopoShape.h"
#include "brepgraph_c/computation/CalcGraph.h"

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepTools.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <TopAbs.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shape.hxx>
#include <BRep_Builder.hxx>
#include <gp_Trsf.hxx>
// route B (local-fuse) spike
#include <BOPAlgo_Builder.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_MapOfShape.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <BOPAlgo_BuilderSolid.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <list>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

namespace
{

// ---- shape metrics (same probes the golden harness pins) ----------

struct Counts { int solids = 0, shells = 0, faces = 0, edges = 0, verts = 0; };

int CountSub(const TopoDS_Shape& s, TopAbs_ShapeEnum kind)
{
    TopTools_IndexedMapOfShape m;
    TopExp::MapShapes(s, kind, m);
    return m.Extent();
}

Counts CountSubs(const TopoDS_Shape& s)
{
    Counts c;
    if (s.IsNull()) return c;
    c.solids = CountSub(s, TopAbs_SOLID);
    c.shells = CountSub(s, TopAbs_SHELL);
    c.faces  = CountSub(s, TopAbs_FACE);
    c.edges  = CountSub(s, TopAbs_EDGE);
    c.verts  = CountSub(s, TopAbs_VERTEX);
    return c;
}

struct Box { double mn[3] = {0,0,0}, mx[3] = {0,0,0}; bool valid = false; };

Box BBox(const TopoDS_Shape& s)
{
    Box b;
    if (s.IsNull()) return b;
    Bnd_Box bb;
    // AddOptimal, not Add: Add bounds a B-spline by its control polygon,
    // overshooting curved extremes by tens of um -- enough to fake a
    // divergence against ZW3D's exact kernel bbox (R2900_100 probe 48
    // showed 23.6um of pure method bias on a verified-identical pattern).
    // Exact geometry, no triangulation, no tolerance padding.
    BRepBndLib::AddOptimal(s, bb, /*useTriangulation=*/Standard_False,
                           /*useShapeTolerance=*/Standard_False);
    if (bb.IsVoid()) return b;
    bb.Get(b.mn[0], b.mn[1], b.mn[2], b.mx[0], b.mx[1], b.mx[2]);
    b.valid = true;
    return b;
}

double BoxDiag(const Box& b)
{
    if (!b.valid) return 0.0;
    const double dx = b.mx[0] - b.mn[0];
    const double dy = b.mx[1] - b.mn[1];
    const double dz = b.mx[2] - b.mn[2];
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

struct VolArea { double volume = 0.0, area = 0.0; };

VolArea VolumeAndArea(const TopoDS_Shape& s)
{
    VolArea r;
    if (s.IsNull()) return r;
    GProp_GProps gv, gs;
    BRepGProp::VolumeProperties(s, gv);
    BRepGProp::SurfaceProperties(s, gs);
    r.volume = gv.Mass();
    r.area   = gs.Mass();
    return r;
}

// Volume of the SOLID content only. Open sheet bodies have no defined
// volume; VolumeProperties on a shell returns the surface flux integral
// -- on R2900 the three Revolve4 funnel sheets book ~32k mm^3 of phantom
// "volume" and a MIRRORED copy books a different number again, so a
// whole-compound volume comparison against a truth STEP is dominated by
// sheet accounting noise. Used by the STEP comparisons (both sides);
// the _state probes keep the whole-compound number because ZW3D's own
// per-shape mass bookkeeping includes its sheets the same way.
// One "INFO solid_<side>" line per solid (vol + bbox + shell count).
// Shared by the full compare and the --max-feat --step prefix compare.
// A solid with >1 shell carries internal voids; each shell's signed
// flux volume is printed so a hollowed body (R2900: funnel cavities
// sewn into the main housing) is measurable directly.
void SolidRoster(const char* side, const TopoDS_Shape& s)
{
    int i = 0;
    for (TopExp_Explorer ex(s, TopAbs_SOLID); ex.More(); ex.Next(), ++i)
    {
        GProp_GProps g;
        BRepGProp::VolumeProperties(ex.Current(), g);
        Bnd_Box bb;
        BRepBndLib::Add(ex.Current(), bb);
        double x0=0, y0=0, z0=0, x1=0, y1=0, z1=0;
        if (!bb.IsVoid()) bb.Get(x0, y0, z0, x1, y1, z1);
        int nshell = 0;
        for (TopExp_Explorer sx(ex.Current(), TopAbs_SHELL); sx.More();
             sx.Next())
            ++nshell;
        std::printf("INFO solid_%s i=%d vol=%.6g shells=%d "
                    "bbox=(%.4g,%.4g,%.4g)(%.4g,%.4g,%.4g)\n",
                    side, i, g.Mass(), nshell, x0, y0, z0, x1, y1, z1);
        if (nshell > 1)
        {
            int j = 0;
            for (TopExp_Explorer sx(ex.Current(), TopAbs_SHELL); sx.More();
                 sx.Next(), ++j)
            {
                GProp_GProps sg;
                BRepGProp::VolumeProperties(sx.Current(), sg);
                Bnd_Box sb;
                BRepBndLib::Add(sx.Current(), sb);
                double a0=0, b0=0, c0=0, a1=0, b1=0, c1=0;
                if (!sb.IsVoid()) sb.Get(a0, b0, c0, a1, b1, c1);
                std::printf("INFO   shell_%s i=%d j=%d vol=%.6g "
                            "bbox=(%.4g,%.4g,%.4g)(%.4g,%.4g,%.4g)\n",
                            side, i, j, sg.Mass(), a0, b0, c0, a1, b1, c1);
            }
        }
    }
}

double SolidsVolume(const TopoDS_Shape& s)
{
    double v = 0.0;
    if (s.IsNull()) return v;
    for (TopExp_Explorer ex(s, TopAbs_SOLID); ex.More(); ex.Next())
    {
        GProp_GProps g;
        BRepGProp::VolumeProperties(ex.Current(), g);
        v += g.Mass();
    }
    return v;
}

// Count FREE sheet bodies: shells not owned by any solid. Reported in
// the STEP comparisons so the sheet population mismatch stays visible
// once the volume metric stops seeing it.
int FreeSheetCount(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0;
    int total = 0;
    for (TopExp_Explorer ex(s, TopAbs_SHELL); ex.More(); ex.Next()) ++total;
    for (TopExp_Explorer so(s, TopAbs_SOLID); so.More(); so.Next())
        for (TopExp_Explorer sh(so.Current(), TopAbs_SHELL); sh.More(); sh.Next())
            --total;
    return (total > 0) ? total : 0;
}

// ---- per-face matching (ported from tools/brp_diff) ----------------

struct FaceProbe
{
    gp_Pnt centre;
    gp_Dir normal;
    double area  = 0.0;
    bool   valid = false;
};

std::vector<FaceProbe> CollectFaceProbes(const TopoDS_Shape& s)
{
    std::vector<FaceProbe> out;
    if (s.IsNull()) return out;
    TopTools_IndexedMapOfShape m;
    TopExp::MapShapes(s, TopAbs_FACE, m);
    out.reserve(m.Extent());
    for (int i = 1; i <= m.Extent(); ++i)
    {
        const TopoDS_Face& f = TopoDS::Face(m.FindKey(i));
        FaceProbe p;
        p.area  = cadapp::FaceArea(f);
        p.valid = cadapp::FaceCenter(f, p.centre, p.normal);
        out.push_back(p);
    }
    return out;
}

struct FaceMatch
{
    int    total      = 0;   // valid probes on side A
    int    matched    = 0;   // centroid within tol of some B face
    int    normal_ok  = 0;   // matched and |dot| > 0.99
    int    area_ok    = 0;   // matched and area within 1% (or both tiny)
    double max_dist   = 0.0; // worst matched centroid distance
    std::vector<int> unmatched;  // indices into A
};

FaceMatch MatchFacesAB(const std::vector<FaceProbe>& A,
                       const std::vector<FaceProbe>& B,
                       double                        tol)
{
    FaceMatch s;
    for (size_t i = 0; i < A.size(); ++i)
    {
        const FaceProbe& fa = A[i];
        if (!fa.valid) continue;
        ++s.total;
        double best = std::numeric_limits<double>::max();
        int    bj   = -1;
        for (size_t j = 0; j < B.size(); ++j)
        {
            if (!B[j].valid) continue;
            const double d = fa.centre.Distance(B[j].centre);
            if (d < best) { best = d; bj = (int)j; }
        }
        if (bj >= 0 && best <= tol)
        {
            ++s.matched;
            s.max_dist = std::max(s.max_dist, best);
            const double dot = fa.normal.X()*B[bj].normal.X()
                             + fa.normal.Y()*B[bj].normal.Y()
                             + fa.normal.Z()*B[bj].normal.Z();
            if (std::fabs(dot) > 0.99) ++s.normal_ok;
            const double aa = fa.area, ab = B[bj].area;
            const double denom = std::max(std::fabs(aa), std::fabs(ab));
            if (denom <= 0.0 || std::fabs(aa - ab) / denom <= 0.01) ++s.area_ok;
        }
        else
        {
            s.unmatched.push_back((int)i);
        }
    }
    return s;
}

// ---- STEP io --------------------------------------------------------

// True when s is well-formed UTF-8 (ASCII counts). Distinguishes a path
// that is ALREADY UTF-8 from a system-ANSI (GBK) one, so the ACP
// round-trip below never mangles a UTF-8 input.
bool IsValidUtf8(const std::string& s)
{
    const auto* p   = reinterpret_cast<const unsigned char*>(s.data());
    const auto* end = p + s.size();
    while (p < end)
    {
        if (*p < 0x80) { ++p; continue; }
        int n = 0;
        if      ((*p & 0xE0) == 0xC0) { n = 1; }
        else if ((*p & 0xF0) == 0xE0) { n = 2; }
        else if ((*p & 0xF8) == 0xF0) { n = 3; }
        else { return false; }
        if (end - p <= n) { return false; }
        for (int k = 1; k <= n; ++k) {
            if ((p[k] & 0xC0) != 0x80) { return false; }
        }
        p += n + 1;
    }
    return true;
}

bool LoadStep(const std::string& path, TopoDS_Shape& out, std::string& err)
{
    // Two narrow-string worlds on Windows: argv and the CRT-based cax
    // readers speak the ANSI code page, but OCCT's OSD_OpenFile decodes
    // char* as UTF-8 -- a GBK Chinese path reaches it as invalid UTF-8
    // and the read fails. fs::path decodes narrow as ACP on MSVC, so a
    // round-trip through it re-encodes ACP -> UTF-8 for OCCT. A path
    // that is already valid UTF-8 must pass through untouched (the ACP
    // decode would mangle it).
    const std::string occt_path = IsValidUtf8(path)
        ? path
        : std::filesystem::path(path).u8string();
    STEPControl_Reader rd;
    const IFSelect_ReturnStatus st = rd.ReadFile(occt_path.c_str());
    if (st != IFSelect_RetDone)
    {
        err = "STEP read failed (status " + std::to_string((int)st) + "): " + path;
        return false;
    }
    rd.TransferRoots();
    TopoDS_Shape s = rd.OneShape();
    if (s.IsNull())
    {
        err = "STEP transfer produced a null shape: " + path;
        return false;
    }

    // ZW3D's whole-part export ("all objects") rides the part's visible
    // wireframe along: R2900_100's truth STEP carries one solid PLUS a
    // GEOMETRIC_CURVE_SET of 593 free curves -- exactly the surplus over
    // the kernel's own edge count, and the source of a phantom 0.253mm
    // bbox bulge (free curves add no faces, so volume / area / face
    // matching never noticed). Drop ONLY the faceless children. An
    // earlier version kept solids exclusively, which silently discarded
    // every SHEET body too -- on the full R2900 part ZW3D exports the
    // MAIN body itself as an open SHELL_BASED_SURFACE_MODEL (5 closed
    // solids = 1003 faces vs 6 open shells = 1852 faces incl. the main
    // body), so the "truth" the metrics ran against was missing most of
    // the part. Open sheet bodies are real geometry: keep them, and
    // report the sheet count so a solid/sheet classification gap
    // between the kernels stays visible.
    {
        // A top-level child is "free wireframe" when it contains no face
        // (the curve set arrives as a nested compound of edges, so the
        // shape TYPE alone cannot tell it from the body's compound).
        int n_free = 0, n_sheet = 0, kept = 0;
        TopoDS_Compound comp;
        BRep_Builder    bb;
        bb.MakeCompound(comp);
        for (TopoDS_Iterator it(s); it.More(); it.Next())
        {
            TopExp_Explorer f(it.Value(), TopAbs_FACE);
            if (!f.More()) { ++n_free; continue; }
            bb.Add(comp, it.Value());
            ++kept;
            TopExp_Explorer so(it.Value(), TopAbs_SOLID);
            if (!so.More()) ++n_sheet;
        }
        if (n_free > 0 && kept > 0)
        {
            std::printf("INFO truth_filtered kept=%d face-carrying bodies "
                        "(%d sheet), dropped %d faceless top-level entities "
                        "(free wireframe)\n",
                        kept, n_sheet, n_free);
            s = comp;
        }
        else if (n_sheet > 0)
        {
            std::printf("INFO truth_sheets %d of %d top-level bodies are "
                        "open sheets (kept)\n", n_sheet, kept);
        }
    }

    out = s;
    return true;
}

bool WriteStep(const std::string& path, const TopoDS_Shape& s, std::string& err)
{
    const std::string occt_path = std::filesystem::path(path).u8string();
    STEPControl_Writer wr;
    if (wr.Transfer(s, STEPControl_AsIs) != IFSelect_RetDone)
    {
        err = "STEP transfer failed for dump: " + path;
        return false;
    }
    if (wr.Write(occt_path.c_str()) != IFSelect_RetDone)
    {
        err = "STEP write failed: " + path;
        return false;
    }
    return true;
}

TopoDS_Shape ScaleShape(const TopoDS_Shape& s, double scale)
{
    if (s.IsNull() || scale == 1.0) return s;
    gp_Trsf t;
    t.SetScaleFactor(scale);
    BRepBuilderAPI_Transform xf(s, t, /*Copy=*/true);
    return xf.Shape();
}

// ---- snapshot scan: opaque stats + per-feature _state truth --------

// Cumulative whole-part state right after feature K executed, as the
// plugin recorded it (units: mm / mm^2 / mm^3 -- the snapshot's native
// length_unit; scaled by the caller before comparison).
struct FeatState
{
    bool   has      = false;
    int    n_shape  = -1, n_face = -1, n_edge = -1;
    int    n_blanked = 0;    // of n_shape, hidden construction bodies --
                             // counted by _state, absent from any STEP
    bool   has_box  = false;
    double bmin[3]  = {0,0,0}, bmax[3] = {0,0,0};
    bool   has_mass = false;
    double area     = -1.0, volume = -1.0;
};

struct FeatMeta
{
    uint32_t    id = 0;
    std::string name;
    std::string zw_type;
    std::string kind;
    FeatState   state;
};

struct SnapshotStats
{
    int total    = 0;
    int opaque   = 0;
    std::map<std::string, int> opaque_types;   // zw_type -> count
    std::vector<FeatMeta>      feats;          // in document order
};

SnapshotStats ScanSnapshot(const std::string& json_path)
{
    SnapshotStats st;
    std::ifstream in(json_path, std::ios::binary);
    if (!in) return st;
    nlohmann::json j;
    try { in >> j; } catch (...) { return st; }
    const auto& feats = j["document"]["features"];
    if (!feats.is_array()) return st;
    for (const auto& f : feats)
    {
        ++st.total;
        FeatMeta m;
        m.id      = f.value("id", 0u);
        m.name    = f.value("name", std::string());
        m.kind    = f.value("kind", std::string());
        m.zw_type = f.value("zw_type", std::string());
        if (m.kind == "opaque")
        {
            ++st.opaque;
            ++st.opaque_types[m.zw_type.empty() ? "?" : m.zw_type];
        }
        auto s = f.find("_state");
        if (s != f.end() && s->is_object())
        {
            m.state.has     = true;
            m.state.n_shape = s->value("n_shape", -1);
            m.state.n_face  = s->value("n_face", -1);
            m.state.n_edge  = s->value("n_edge", -1);
            m.state.n_blanked = s->value("n_blanked", 0);
            auto bb = s->find("bbox");
            if (bb != s->end() && bb->is_array() && bb->size() == 6)
            {
                for (int k = 0; k < 3; ++k)
                {
                    m.state.bmin[k] = bb->at(k).get<double>();
                    m.state.bmax[k] = bb->at(k + 3).get<double>();
                }
                m.state.has_box = true;
            }
            if (s->contains("volume"))
            {
                m.state.volume   = s->value("volume", -1.0);
                m.state.area     = s->value("area", -1.0);
                m.state.has_mass = true;
            }
        }
        st.feats.push_back(std::move(m));
    }
    return st;
}

std::string Lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return (char)std::tolower(c); });
    return s;
}

// ---- IR prefix truncation ------------------------------------------

// Synthetic features the reader fabricates (e.g. an FtAllExt's profile
// sketch) carry ids of 1000000 + <json id of the consuming feature>;
// real history features keep their 1-based JSON ordinal.
constexpr uint32_t kSyntheticIdBase = 1000000u;

// Keep only the IR features needed to replay history features 1..K.
// Features are stored in document order with each synthetic sketch
// pushed immediately before its consuming feature, so a positional cut
// at the first real feature with id > K, minus any synthetic sketches
// dangling right before it, is exactly the prefix. (A dangling sketch
// would otherwise become an unconsumed live output candidate and
// pollute the state compound with a stray face.)
void TruncateDoc(cadapp::DocumentIR& doc, uint32_t K)
{
    size_t cut = doc.features.size();
    for (size_t i = 0; i < doc.features.size(); ++i)
    {
        const cadapp::FeatureIR& f = doc.features[i];
        if (f.id < kSyntheticIdBase && f.id > K) { cut = i; break; }
    }
    while (cut > 0)
    {
        const cadapp::FeatureIR& f = doc.features[cut - 1];
        if (f.id >= kSyntheticIdBase && f.type == cadapp::FeatType::Sketch) --cut;
        else break;
    }
    doc.features.resize(cut);
}

// ---- authored geometry (ZwLoader parity) ---------------------------

// BakedShape features (e.g. a CdGeomCopy imported base) reference their
// geometry as a per-feature STEP path in ext_strings["zw_geometry"];
// the SDK-free reader can only record the path. Load them here so the
// Replayer's BakedShape arm has its body-root shape, mirroring
// wrap_CadCvt's ZwLoader (minus the editor-side VersionTree root
// registration -- this tool never serializes through BrepDB).
void LoadAuthoredShapes(cadapp::DocumentIR& doc, double unit_scale)
{
    for (auto& feat : doc.features)
    {
        auto it = feat.ext_strings.find("zw_geometry");
        if (it == feat.ext_strings.end() || it->second.empty()) continue;

        TopoDS_Shape shp;
        std::string  err;
        if (!LoadStep(it->second, shp, err))
        {
            std::printf("INFO authored_step_missing feat=%u err=%s\n",
                        feat.id, err.c_str());
            continue;   // Replayer reports the gap
        }
        shp = ScaleShape(shp, unit_scale);
        doc.authored_shapes[feat.id] = std::make_shared<brepkit::TopoShape>(shp);
    }
}

// ---- replay ---------------------------------------------------------

// Replays a COPY of the doc (Replay can mutate its input) with the same
// options the editor's ZwLoader uses. Empty feature list (a K=0 probe)
// yields a null shape and ok.
// drop_hidden=false keeps source-hidden bodies in the emitted shape:
// _state probes compare against per-feature truth that still contains
// construction sheets which only blank/delete at the end of history.
bool ReplayShape(const cadapp::DocumentIR& doc_in,
                 TopoDS_Shape&             out,
                 std::string&              err,
                 bool                      drop_hidden = true)
{
    out = TopoDS_Shape();
    if (doc_in.features.empty()) return true;

    cadapp::DocumentIR doc = doc_in;
    try
    {
        cadapp::Replayer      replayer;
        cadapp::ReplayOptions opt;
        opt.write_back_resolved = false;
        opt.commit_versions     = false;
        opt.drop_hidden         = drop_hidden;
        cadapp::ReplayResult res;
        replayer.Replay(doc, opt, res);
        if (!res.ok || !res.shape || res.shape->GetShape().IsNull())
        {
            err = res.err_msg.empty() ? "null shape" : res.err_msg;
            return false;
        }
        // The Replayer accumulates soft diagnostics in err_msg even on
        // success -- dropped dressups, dead-feature substitutions, rigid
        // pattern warnings. Those are exactly the "this feature silently
        // no-opped" trail (R2900_100's Fillet4), so surface them.
        if (!res.err_msg.empty())
        {
            std::printf("INFO replay_msg %s\n", res.err_msg.c_str());
        }
        out = res.shape->GetShape();
        return true;
    }
    catch (Standard_Failure& e)
    {
        const char* m = e.GetMessageString();
        err = std::string("threw=") + (m ? m : "Standard_Failure");
        return false;
    }
    catch (const std::exception& e)
    {
        err = std::string("threw=") + e.what();
        return false;
    }
}

void PrintState(uint32_t K, const TopoDS_Shape& s)
{
    const Counts  c  = CountSubs(s);
    const Box     b  = BBox(s);
    const VolArea va = VolumeAndArea(s);
    std::printf("STATE feat=%u solids=%d shells=%d faces=%d edges=%d "
                "vol=%.9g area=%.9g bbox=(%.6g,%.6g,%.6g)(%.6g,%.6g,%.6g)\n",
                K, c.solids, c.shells, c.faces, c.edges,
                va.volume, va.area,
                b.mn[0], b.mn[1], b.mn[2], b.mx[0], b.mx[1], b.mx[2]);
}

// ---- probe: replayed prefix vs plugin-recorded _state truth ---------

struct ProbeOutcome
{
    bool ran  = false;   // replay succeeded
    bool good = false;   // metrics agree within tolerance
};

// Compare the replayed prefix state against the plugin's _state truth
// for the same feature. Solids must match exactly; bbox (and volume,
// when the plugin recorded mass) must agree within rel_tol of the truth
// bbox diagonal / value. Face/edge counts are reported but never decide
// the verdict -- the two kernels legitimately split periodic faces
// differently.
ProbeOutcome ProbeAgainstState(const cadapp::DocumentIR& master,
                               const FeatMeta&           fm,
                               double                    unit_scale,
                               double                    rel_tol)
{
    ProbeOutcome out;

    cadapp::DocumentIR doc = master;
    TruncateDoc(doc, fm.id);

    TopoDS_Shape shape;
    std::string  err;
    if (!ReplayShape(doc, shape, err, /*drop_hidden=*/false))
    {
        // Empty-vs-empty is agreement, not failure: a prefix of pure
        // wireframe/datum features (02-ear opens with 16 of them) has no
        // body on EITHER side -- truth _state records 0 shapes. Only call
        // it a divergence when the truth says geometry should exist.
        if (fm.state.has && fm.state.n_shape == 0)
        {
            std::printf("PROBE feat=%u verdict=good reason=both_empty "
                        "name=%s\n", fm.id, fm.name.c_str());
            out.ran  = true;
            out.good = true;
            return out;
        }
        std::printf("PROBE feat=%u verdict=bad reason=replay_failed %s\n",
                    fm.id, err.c_str());
        out.ran  = true;   // a replay failure IS a divergence verdict
        out.good = false;
        return out;
    }
    out.ran = true;

    const Counts  c  = CountSubs(shape);
    const Box     b  = BBox(shape);
    const VolArea va = VolumeAndArea(shape);
    PrintState(fm.id, shape);

    const FeatState& st = fm.state;
    const double s1 = unit_scale, s2 = s1 * s1, s3 = s2 * s1;

    // Truth bbox in replay units.
    double tmin[3], tmax[3];
    for (int k = 0; k < 3; ++k)
    {
        tmin[k] = st.bmin[k] * s1;
        tmax[k] = st.bmax[k] * s1;
    }
    double tdiag = 0.0;
    if (st.has_box)
    {
        const double dx = tmax[0] - tmin[0];
        const double dy = tmax[1] - tmin[1];
        const double dz = tmax[2] - tmin[2];
        tdiag = std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    std::printf("TRUTHSTATE feat=%u n_shape=%d n_blanked=%d n_face=%d "
                "n_edge=%d vol=%.9g area=%.9g "
                "bbox=(%.6g,%.6g,%.6g)(%.6g,%.6g,%.6g)\n",
                fm.id, st.n_shape, st.n_blanked, st.n_face, st.n_edge,
                st.has_mass ? st.volume * s3 : -1.0,
                st.has_mass ? st.area * s2 : -1.0,
                tmin[0], tmin[1], tmin[2], tmax[0], tmax[1], tmax[2]);

    bool        good = true;
    std::string why;

    // Body count is reported as a TOPOLOGY signal but does not decide the
    // geometry verdict: ZW3D patterns legitimately leave instances as
    // standalone bodies until a later feature merges them, while the
    // replay's pattern op fuses immediately -- a representational gap, not
    // a placement error (R2900_100 Pattern9: 1 vs 4 bodies with instance
    // geometry verified identical). Bisect on geometry; read the topo
    // column to see where body-structure fidelity is lost.
    std::string topo;
    if (st.n_shape >= 0 && c.solids != st.n_shape)
    {
        topo = "topo=" + std::to_string(c.solids) + "/" +
               std::to_string(st.n_shape);
        if (st.n_blanked > 0) {
            topo += "(bl" + std::to_string(st.n_blanked) + ")";
        }
        topo += " ";
    }
    if (st.has_box && tdiag > 0.0)
    {
        double worst = 0.0;
        if (b.valid)
        {
            for (int k = 0; k < 3; ++k)
            {
                worst = std::max(worst, std::fabs(b.mn[k] - tmin[k]));
                worst = std::max(worst, std::fabs(b.mx[k] - tmax[k]));
            }
        }
        const double rel = b.valid ? worst / tdiag : 1.0;
        if (rel > rel_tol)
        {
            good = false;
            char buf[64];
            std::snprintf(buf, sizeof buf, "bbox_rel=%.3g ", rel);
            why += buf;
        }
    }
    // "Has volume" must mean MEANINGFULLY positive: the plugin's _state
    // volume is the SIGNED sum over all bodies, where open sheets
    // contribute orientation-dependent flux. Two failure shapes:
    //   - pure-sheet states land at ±1e-24 mm^3 (02-ear feat 12 recorded
    //     +4.8e-24; the old >0 gate exploded vol_rel to 1e16);
    //   - mixed states can NEAR-CANCEL into a small positive residue
    //     (02-ear feat 96: solids +3119 mm^3 + sheet flux -3114 = +4.5,
    //     verified against state96.step whose solids measure 3118.66 --
    //     the old 1e-9*d^3 gate took 4.5 as real and reported
    //     vol_rel=835 of pure noise).
    // 1e-4*d^3 keeps any plausible solid part (final visible volume here
    // is 8e-3*d^3) while rejecting cancellation residues; states below
    // it degrade to the area check, which is flux-immune.
    const double vol_eps = (tdiag > 0.0) ? 1e-4 * tdiag * tdiag * tdiag
                                         : 1e-15;
    if (st.has_mass && st.volume * s3 > vol_eps)
    {
        const double tv  = st.volume * s3;
        const double rel = std::fabs(va.volume - tv) / tv;
        if (rel > rel_tol)
        {
            good = false;
            char buf[64];
            std::snprintf(buf, sizeof buf, "vol_rel=%.3g ", rel);
            why += buf;
        }
    }
    else if (st.has_mass && st.volume * s3 < -vol_eps)
    {
        // Open-shell state: the recorded "volume" is signed flux, whose
        // sign convention is the kernel's own (face orientation). Report
        // the magnitude divergence but never let it decide -- area below
        // is the orientation-independent sheet metric.
        const double tv  = std::fabs(st.volume * s3);
        const double rv  = std::fabs(va.volume);
        const double rel = tv > 0.0 ? std::fabs(rv - tv) / tv : 0.0;
        if (rel > rel_tol)
        {
            char buf[64];
            std::snprintf(buf, sizeof buf, "flux_rel=%.3g(info) ", rel);
            why += buf;
        }
    }
    if (st.has_mass && st.area > 0.0)
    {
        // Surface area: orientation-independent, and on a sheet-only
        // state (truth volume <= 0) the only strong metric -- bbox can
        // stay put while a skin grows a whole new patch inside it. On
        // solid states it stays informational: cross-kernel face splits
        // carry ~0.3% area noise (R2900) that volume+bbox don't.
        const double ta  = st.area * s2;
        const double rel = std::fabs(va.area - ta) / ta;
        if (rel > rel_tol)
        {
            char buf[64];
            const bool decides = (st.volume * s3 <= vol_eps);
            std::snprintf(buf, sizeof buf, "area_rel=%.3g%s ", rel,
                          decides ? "" : "(info)");
            why += buf;
            if (decides) {
                good = false;
            }
        }
    }

    // WARN-grade signal only (kernel face-split differences are legal).
    const int dfaces = (st.n_face >= 0) ? c.faces - st.n_face : 0;

    std::printf("PROBE feat=%u verdict=%s %s%sdfaces=%d name=%s\n",
                fm.id, good ? "good" : "bad", why.c_str(), topo.c_str(),
                dfaces, fm.name.c_str());
    out.good = good;
    return out;
}

// Hard per-process commit cap via a Job Object. A pathological OCCT op
// (2026-06-12: BOPAlgo_MakerVolume on 02-ear 组合1, inside one face-face
// intersection that never polls UserBreak) otherwise allocates at
// ~110MB/s until the machine thrashes to death -- three freezes at
// 90+GB in one day. With the cap, the runaway allocation FAILS instead;
// OCCT turns that into Standard_Failure / bad_alloc, which the op-level
// catch (TopoAlgo) or ReplayShape's catch degrades gracefully.
// CAX_MEM_BUDGET_MB overrides the default 8192; 0 disables the cap.
void InstallMemoryBudget()
{
#ifdef _WIN32
    size_t mb = 8192;
    if (const char* e = std::getenv("CAX_MEM_BUDGET_MB"))
    {
        const long v = std::atol(e);
        if (v <= 0) return;
        mb = (size_t)v;
    }
    HANDLE job = CreateJobObjectW(nullptr, nullptr);
    if (!job) return;
    JOBOBJECT_EXTENDED_LIMIT_INFORMATION jeli{};
    jeli.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_PROCESS_MEMORY;
    jeli.ProcessMemoryLimit = mb * (size_t)1024 * (size_t)1024;
    if (SetInformationJobObject(job, JobObjectExtendedLimitInformation,
                                &jeli, sizeof jeli) &&
        AssignProcessToJobObject(job, GetCurrentProcess()))
    {
        std::fprintf(stderr,
            "[mem] process commit budget %zu MB (CAX_MEM_BUDGET_MB "
            "overrides, 0 disables)\n", mb);
    }
    // On failure (e.g. an enclosing job forbids nesting) just run
    // unbudgeted -- same behavior as before this guard.
#endif
}

} // namespace

int main(int argc, char** argv)
{
    InstallMemoryBudget();
    std::string json_path, step_path, dump_path, states_arg;
    double      rel_tol   = 1e-4;   // volume/area/bbox, relative
    double      face_tol  = 1e-4;   // face centroid match, fraction of bbox diag
    long        max_feat  = -1;     // --max-feat K: prefix replay
    long        detail_cap = 12;    // unmatched-face DETAIL lines per side
    bool        bisect    = false;

    // --fuse-probe a.brep b.brep: re-run ONE boolean pair in isolation.
    // CAX_BOP_DUMP=1 makes TopoAlgo::Fuse write every big-pair fuse's
    // operands to bop_<id>_{a,b}.brep; this mode loads such a pair and
    // calls the SAME TopoAlgo::Fuse, so a pathological boolean can be
    // iterated on in seconds instead of re-replaying an 8-minute
    // document prefix per experiment. Honors the same env knobs
    // (BREPKIT_BOP_PARALLEL, CAX_GEO_LOG, BREPKIT_BOP_PROF).
    if (argc == 4 && std::strcmp(argv[1], "--fuse-probe") == 0)
    {
        TopoDS_Shape a, b;
        BRep_Builder bb;
        if (!BRepTools::Read(a, argv[2], bb) || a.IsNull()) {
            std::fprintf(stderr, "cannot read %s\n", argv[2]);
            return 2;
        }
        if (!BRepTools::Read(b, argv[3], bb) || b.IsNull()) {
            std::fprintf(stderr, "cannot read %s\n", argv[3]);
            return 2;
        }
        auto ta = std::make_shared<brepkit::TopoShape>(a);
        auto tb = std::make_shared<brepkit::TopoShape>(b);
        const auto t0 = std::chrono::steady_clock::now();
        auto r = brepkit::TopoAlgo::Fuse(ta, tb, /*op_id=*/9999,
                                         nullptr, nullptr);
        const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        const Counts c = CountSubs(r ? r->GetShape() : TopoDS_Shape());
        const VolArea va = VolumeAndArea(r ? r->GetShape() : TopoDS_Shape());
        std::printf("FUSEPROBE ms=%.1f solids=%d faces=%d vol=%.9g\n",
                    ms, c.solids, c.faces, va.volume);
        return 0;
    }

    // --local-fuse-probe a.brep b.brep: Stage-0 spike for route B (local
    // boolean via boundary evaluation). Runs the general fuse (reference)
    // AND a local "B3" fuse, then compares volume/faces/validity/time. The
    // B3 pipeline: pick the body faces whose bbox meets the tool -> run
    // BOPAlgo_Builder on ONLY {touched faces, tool faces} (a LOCAL imprint +
    // split that OCCT gives consistent shared edges + history for) ->
    // classify each split piece in/out the other operand -> assemble the
    // union boundary = untouched body faces (TShape reused verbatim) + body
    // pieces outside the tool + tool pieces outside the body. Proves whether
    // a local fuse can be valid + faithful + faster on a big body. Results
    // discarded. Single-tool / small-footprint only; bails otherwise.
    if (argc == 4 && std::strcmp(argv[1], "--local-fuse-probe") == 0)
    {
        TopoDS_Shape body, tool;
        BRep_Builder rbb;
        if (!BRepTools::Read(body, argv[2], rbb) || body.IsNull() ||
            !BRepTools::Read(tool, argv[3], rbb) || tool.IsNull()) {
            std::fprintf(stderr, "cannot read operands\n");
            return 2;
        }

        // Reference: the general whole-body fuse.
        const auto refT0 = std::chrono::steady_clock::now();
        auto refr = brepkit::TopoAlgo::Fuse(
            std::make_shared<brepkit::TopoShape>(body),
            std::make_shared<brepkit::TopoShape>(tool), 9999, nullptr, nullptr);
        const double ref_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - refT0).count();
        const VolArea refva = VolumeAndArea(refr ? refr->GetShape() : TopoDS_Shape());
        const Counts  refc  = CountSubs(refr ? refr->GetShape() : TopoDS_Shape());
        const bool refValid = (refr && !refr->GetShape().IsNull())
            ? BRepCheck_Analyzer(refr->GetShape()).IsValid() : false;

        // Local B3 fuse.
        const auto locT0 = std::chrono::steady_clock::now();
        std::string note = "ok";
        TopoDS_Shape result;
        int nTouched = 0, nTotal = 0, nUntouched = 0, nKeptBody = 0, nKeptTool = 0;
        int nSolidsTotal = 0, nSolidsTouched = 0;
        bool rebuiltValid = false;
        do {
            Bnd_Box tbox; BRepBndLib::Add(tool, tbox);
            if (tbox.IsVoid()) { note = "tool bbox void"; break; }
            tbox.Enlarge(16e-6);

            // The body is (often) a multi-solid compound. Only the solid the
            // tool meets is rebuilt; the others pass through verbatim. For
            // this spike we require exactly one touched solid (the editor's
            // single-tool edit case); otherwise bail to the general fuse.
            // Touched solid = one with a FACE near the tool (face-level is
            // far tighter than solid-bbox: R2900's solids pack close, so a
            // solid bbox meets the tool while none of its faces do).
            TopTools_ListOfShape passThrough;
            TopoDS_Shape touchedSolid;
            for (TopExp_Explorer se(body, TopAbs_SOLID); se.More(); se.Next()) {
                ++nSolidsTotal;
                bool meets = false;
                for (TopExp_Explorer fe(se.Current(), TopAbs_FACE); fe.More() && !meets; fe.Next()) {
                    Bnd_Box fb; BRepBndLib::Add(fe.Current(), fb);
                    if (!fb.IsVoid() && !fb.IsOut(tbox)) meets = true;
                }
                if (meets) { touchedSolid = se.Current(); ++nSolidsTouched; }
                else passThrough.Append(se.Current());
            }
            if (nSolidsTouched != 1) {
                note = "tool meets " + std::to_string(nSolidsTouched) + " solids"; break;
            }

            // touched faces OF THE TOUCHED SOLID
            TopTools_IndexedMapOfShape solidFaces;
            TopExp::MapShapes(touchedSolid, TopAbs_FACE, solidFaces);
            nTotal = solidFaces.Extent();
            TopTools_ListOfShape args, touched;
            TopTools_MapOfShape  touchedSet;
            for (int i = 1; i <= solidFaces.Extent(); ++i) {
                Bnd_Box fb; BRepBndLib::Add(solidFaces(i), fb);
                if (fb.IsVoid() || fb.IsOut(tbox)) continue;
                touched.Append(solidFaces(i));
                touchedSet.Add(solidFaces(i));
                args.Append(solidFaces(i));
            }
            nTouched = touched.Extent();
            if (nTouched == 0) { note = "0 touched faces"; break; }
            if (nTouched > 200) { note = "footprint not local"; break; }

            TopTools_ListOfShape toolFaces;
            for (TopExp_Explorer te(tool, TopAbs_FACE); te.More(); te.Next()) {
                toolFaces.Append(te.Current());
                args.Append(te.Current());
            }

            // LOCAL imprint + split (OCCT builds consistent shared edges).
            BOPAlgo_Builder builder;
            builder.SetArguments(args);
            builder.SetRunParallel(Standard_False);
            builder.Perform();
            if (builder.HasErrors()) { note = "builder errors"; break; }

            // classify against the SINGLE touched body solid + the tool solid.
            BRepClass3d_SolidClassifier clsTool(tool);
            BRepClass3d_SolidClassifier clsBody(touchedSolid);
            auto inside = [](BRepClass3d_SolidClassifier& c, const gp_Pnt& p) -> bool {
                c.Perform(p, 1e-7); return c.State() == TopAbs_IN;
            };
            // A robust INTERIOR point on the face: param-centre can fall in a
            // hole / outside a trimmed split piece, so sample a UV grid and
            // take the first point the 2D classifier reports strictly IN.
            auto faceMid = [](const TopoDS_Shape& fs) -> gp_Pnt {
                const TopoDS_Face f = TopoDS::Face(fs);
                BRepAdaptor_Surface s(f);
                const double u0 = s.FirstUParameter(), u1 = s.LastUParameter();
                const double v0 = s.FirstVParameter(), v1 = s.LastVParameter();
                BRepTopAdaptor_FClass2d fc(f, 1e-7);
                for (int iu = 1; iu <= 5; ++iu)
                    for (int iv = 1; iv <= 5; ++iv) {
                        const double u = u0 + (u1 - u0) * iu / 6.0;
                        const double v = v0 + (v1 - v0) * iv / 6.0;
                        if (fc.Perform(gp_Pnt2d(u, v)) == TopAbs_IN)
                            return s.Value(u, v);
                    }
                return s.Value((u0 + u1) * 0.5, (v0 + v1) * 0.5);
            };

            // Collect the union-boundary faces: the touched solid's untouched
            // faces + body pieces outside the tool + tool pieces outside the
            // body. BOPAlgo_BuilderSolid then groups them into valid solid(s)
            // -- it handles connexity, orientation, and the multi-lump case
            // the hand-rolled single shell got wrong.
            BRep_Builder bb;
            TopTools_ListOfShape keptFaces;
            for (int i = 1; i <= solidFaces.Extent(); ++i)
                if (!touchedSet.Contains(solidFaces(i))) { keptFaces.Append(solidFaces(i)); ++nUntouched; }
            for (TopTools_ListIteratorOfListOfShape it(touched); it.More(); it.Next()) {
                const TopoDS_Shape& F = it.Value();
                if (builder.IsDeleted(F)) continue;
                const TopTools_ListOfShape& pcs = builder.Modified(F);
                if (pcs.IsEmpty()) {
                    if (!inside(clsTool, faceMid(F))) { keptFaces.Append(F); ++nKeptBody; }
                } else for (TopTools_ListIteratorOfListOfShape p(pcs); p.More(); p.Next())
                    if (!inside(clsTool, faceMid(p.Value()))) { keptFaces.Append(p.Value()); ++nKeptBody; }
            }
            for (TopTools_ListIteratorOfListOfShape it(toolFaces); it.More(); it.Next()) {
                const TopoDS_Shape& G = it.Value();
                if (builder.IsDeleted(G)) continue;
                const TopTools_ListOfShape& pcs = builder.Modified(G);
                if (pcs.IsEmpty()) {
                    if (!inside(clsBody, faceMid(G))) { keptFaces.Append(G); ++nKeptTool; }
                } else for (TopTools_ListIteratorOfListOfShape p(pcs); p.More(); p.Next())
                    if (!inside(clsBody, faceMid(p.Value()))) { keptFaces.Append(p.Value()); ++nKeptTool; }
            }

            BOPAlgo_BuilderSolid bs;
            bs.SetShapes(keptFaces);
            bs.Perform();
            if (bs.HasErrors()) { note = "buildersolid errors"; break; }

            // Local refine: merge the coplanar fragments the general fuse's
            // UnifySameDomain merges, so the face count matches -- on the
            // rebuilt solid(s) only, i.e. O(local). (For production this must
            // be made naming-aware so untouched faces keep their TShape; here
            // we only check geometric faithfulness.)
            TopTools_ListOfShape rebuilt;
            for (TopTools_ListIteratorOfListOfShape it(bs.Areas()); it.More(); it.Next()) {
                TopoDS_Shape s = it.Value();
                try {
                    ShapeUpgrade_UnifySameDomain u(s, Standard_True, Standard_True, Standard_False);
                    u.SetLinearTolerance(1e-6);
                    u.Build();
                    if (!u.Shape().IsNull()) s = u.Shape();
                } catch (...) {}
                rebuilt.Append(s);
            }

            // validity of MY rebuilt solid(s) only (the pass-through solids
            // carry the import's own BRepCheck dirtiness, not our concern).
            rebuiltValid = true;
            for (TopTools_ListIteratorOfListOfShape it(rebuilt); it.More(); it.Next())
                if (!BRepCheck_Analyzer(it.Value()).IsValid()) rebuiltValid = false;

            // result = pass-through solids (verbatim) + the rebuilt solid(s).
            TopoDS_Compound comp; bb.MakeCompound(comp);
            for (TopTools_ListIteratorOfListOfShape it(passThrough); it.More(); it.Next())
                bb.Add(comp, it.Value());
            for (TopTools_ListIteratorOfListOfShape it(rebuilt); it.More(); it.Next())
                bb.Add(comp, it.Value());
            result = comp;
        } while (false);
        const double loc_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - locT0).count();

        VolArea locva{}; Counts locc{};
        if (!result.IsNull()) {
            locva = VolumeAndArea(result);
            locc  = CountSubs(result);
        }
        const double volrel = (refva.volume > 0)
            ? std::fabs(locva.volume - refva.volume) / refva.volume : -1.0;
        std::printf("LOCALFUSE solids=%d/%d touched=%d/%d untouched=%d keptBody=%d keptTool=%d note=%s\n",
                    nSolidsTouched, nSolidsTotal, nTouched, nTotal,
                    nUntouched, nKeptBody, nKeptTool, note.c_str());
        std::printf("REF      ms=%.1f solids=%d faces=%d vol=%.9g valid=%d\n",
                    ref_ms, refc.solids, refc.faces, refva.volume, refValid ? 1 : 0);
        std::printf("LOCAL    ms=%.1f solids=%d faces=%d vol=%.9g valid=%d(rebuilt)\n",
                    loc_ms, locc.solids, locc.faces, locva.volume, rebuiltValid ? 1 : 0);
        std::printf("COMPARE  vol_rel=%.3e dfaces=%d speedup=%.1fx\n",
                    volrel, locc.faces - refc.faces, loc_ms > 0 ? ref_ms / loc_ms : 0.0);
        return 0;
    }

    // --dump-steps: build the calc graph (analyze_only -- no geometry
    // eval) and list every step's op + design-intent desc. Correlates
    // a grinding [eval-begin] op with the FEATURE whose lowering
    // emitted it ("Mirror6:delta", "Pattern16:fuse", ...) without
    // paying for a single boolean.
    if (argc >= 3 && std::strcmp(argv[1], "--dump-steps") == 0)
    {
        cadcvt::ZwReader rd;
        if (std::getenv("CAX_ZW_SCALE1")) rd.SetUnitScale(1.0);
        cadapp::DocumentIR d;
        std::string e;
        if (!rd.ReadFile(argv[2], d, &e)) {
            std::fprintf(stderr, "reader failed: %s\n", e.c_str());
            return 2;
        }
        cadapp::Replayer      rp;
        cadapp::ReplayOptions o;
        o.analyze_only = true;
        cadapp::ReplayResult  rr;
        if (!rp.Replay(d, o, rr) || !rr.calc_graph) {
            std::fprintf(stderr, "analyze replay failed: %s\n",
                         rr.err_msg.c_str());
            return 2;
        }
        const auto& cg = *rr.calc_graph;
        for (size_t sid = 0; sid < cg.GetHistorySize(); ++sid) {
            std::printf("STEP %zu op=%s desc=%s\n", sid,
                        cg.GetStepOpName((int)sid).c_str(),
                        cg.GetStepDesc((int)sid).c_str());
        }
        return 0;
    }

    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--step" && i + 1 < argc)           step_path  = argv[++i];
        else if (a == "--rel-tol" && i + 1 < argc)   rel_tol    = std::atof(argv[++i]);
        else if (a == "--face-tol" && i + 1 < argc)  face_tol   = std::atof(argv[++i]);
        else if (a == "--detail-cap" && i + 1 < argc) detail_cap = std::atol(argv[++i]);
        else if (a == "--max-feat" && i + 1 < argc)  max_feat   = std::atol(argv[++i]);
        else if (a == "--states" && i + 1 < argc)    states_arg = argv[++i];
        else if (a == "--dump" && i + 1 < argc)      dump_path  = argv[++i];
        else if (a == "--bisect")                    bisect     = true;
        else if (json_path.empty())                  json_path  = a;
        else { std::fprintf(stderr, "unexpected arg: %s\n", a.c_str()); return 2; }
    }
    if (json_path.empty())
    {
        std::fprintf(stderr,
            "usage: zw_verify <part.cax.json> [--step <truth.step>]\n"
            "         [--max-feat K] [--states a,b,c|all] [--bisect]\n"
            "         [--dump <out.step>] [--rel-tol r] [--face-tol r]\n");
        return 2;
    }

    bool pass = true;
    auto check = [&pass](bool ok, const char* name, const std::string& detail)
    {
        std::printf("CHECK %s %s %s\n", name, ok ? "ok" : "bad", detail.c_str());
        if (!ok) pass = false;
    };

    // 1. snapshot scan: opaque features are *known* coverage gaps; a
    //    snapshot full of opaques cannot replay to the truth body, and
    //    the report wants them named either way. Also pulls each
    //    feature's _state truth for the probe/bisect modes.
    const SnapshotStats st = ScanSnapshot(json_path);
    {
        std::string types;
        for (const auto& [k, v] : st.opaque_types)
            types += k + "x" + std::to_string(v) + " ";
        int n_state = 0;
        for (const auto& f : st.feats) if (f.state.has) ++n_state;
        std::printf("INFO features=%d opaque=%d states=%d %s\n",
                    st.total, st.opaque, n_state,
                    types.empty() ? "" : ("opaque_types: " + types).c_str());
    }

    // 2. read the snapshot into IR
    cadcvt::ZwReader   reader;
    if (std::getenv("CAX_ZW_SCALE1")) reader.SetUnitScale(1.0);
    cadapp::DocumentIR master;
    std::string        err;
    if (!reader.ReadFile(json_path, master, &err))
    {
        std::printf("CHECK reader bad %s\n", err.c_str());
        std::printf("VERDICT FAIL %s\n", json_path.c_str());
        return 1;
    }
    std::printf("CHECK reader ok features=%d\n", st.total);
    LoadAuthoredShapes(master, reader.UnitScale());

    // ---- mode: bisect against per-feature _state truth -------------
    if (bisect)
    {
        std::vector<const FeatMeta*> probes;
        for (const auto& f : st.feats)
            if (f.state.has && f.id > 0) probes.push_back(&f);
        if (probes.empty())
        {
            std::printf("CHECK bisect bad no _state blocks in snapshot -- "
                        "re-export with a CAX_FEAT_STATE-capable CaxExport.dll\n");
            std::printf("VERDICT FAIL %s\n", json_path.c_str());
            return 1;
        }

        // Invariant: lo == -1 (empty part) is good; probe the last
        // state first so a metrics-clean part is reported instead of
        // bisecting noise.
        int lo = -1, hi = (int)probes.size() - 1;
        ProbeOutcome last = ProbeAgainstState(master, *probes[hi],
                                              reader.UnitScale(), rel_tol);
        if (last.good)
        {
            std::printf("BISECT clean -- final state matches truth within "
                        "tolerance; divergence (if any) is below rel_tol or "
                        "face-decomposition only. Run the full verify.\n");
            std::printf("VERDICT PASS %s\n", json_path.c_str());
            return 0;
        }
        while (lo + 1 < hi)
        {
            const int mid = lo + (hi - lo) / 2;
            const ProbeOutcome r = ProbeAgainstState(master, *probes[mid],
                                                     reader.UnitScale(), rel_tol);
            if (r.good) lo = mid; else hi = mid;
        }
        const FeatMeta& bad = *probes[hi];
        std::printf("BISECT first_bad=%u last_good=%s name=%s zw_type=%s\n",
                    bad.id,
                    lo >= 0 ? std::to_string(probes[lo]->id).c_str() : "none",
                    bad.name.c_str(), bad.zw_type.c_str());
        return 0;
    }

    // ---- mode: probe an explicit list of prefix states -------------
    if (!states_arg.empty())
    {
        std::vector<const FeatMeta*> probes;
        if (Lower(states_arg) == "all")
        {
            for (const auto& f : st.feats)
                if (f.state.has && f.id > 0) probes.push_back(&f);
        }
        else
        {
            std::string tok;
            std::vector<uint32_t> ids;
            for (char c : states_arg + ",")
            {
                if (c == ',')
                {
                    if (!tok.empty()) ids.push_back((uint32_t)std::atol(tok.c_str()));
                    tok.clear();
                }
                else tok += c;
            }
            for (uint32_t id : ids)
                for (const auto& f : st.feats)
                    if (f.id == id) { probes.push_back(&f); break; }
        }
        if (probes.empty())
        {
            std::printf("CHECK states bad nothing to probe (no _state blocks "
                        "or unknown feature ids: %s)\n", states_arg.c_str());
            std::printf("VERDICT FAIL %s\n", json_path.c_str());
            return 1;
        }
        bool all_good = true;
        for (const FeatMeta* f : probes)
        {
            const ProbeOutcome r = ProbeAgainstState(master, *f,
                                                     reader.UnitScale(), rel_tol);
            if (!r.good) all_good = false;
        }
        std::printf("VERDICT %s %s\n", all_good ? "PASS" : "FAIL",
                    json_path.c_str());
        return all_good ? 0 : 1;
    }

    // ---- mode: single prefix replay ---------------------------------
    if (max_feat >= 0)
    {
        cadapp::DocumentIR doc = master;
        TruncateDoc(doc, (uint32_t)max_feat);
        std::printf("INFO prefix max_feat=%ld ir_features=%zu/%zu\n",
                    max_feat, doc.features.size(), master.features.size());

        // Truth-side convention split: state<K>.step files contain the
        // VISIBLE bodies only (blanked bodies excluded, same as the final
        // .cax.step), while the _state JSON metrics include them. So a
        // prefix compared against a --step truth must drop hidden bodies;
        // a bare prefix (eyeballed against _state metrics) must not.
        TopoDS_Shape shape;
        if (!ReplayShape(doc, shape, err,
                         /*drop_hidden=*/!step_path.empty()))
        {
            std::printf("CHECK replay bad %s\n", err.c_str());
            std::printf("VERDICT FAIL %s\n", json_path.c_str());
            return 1;
        }
        std::printf("CHECK replay ok -\n");
        PrintState((uint32_t)max_feat, shape);

        if (!dump_path.empty())
        {
            // Back to the snapshot's native unit (mm) so the dump drops
            // into ZW3D / CAD viewers next to the plugin's own STEPs.
            const double inv = reader.UnitScale() != 0.0
                             ? 1.0 / reader.UnitScale() : 1.0;
            std::string werr;
            if (WriteStep(dump_path, ScaleShape(shape, inv), werr))
                std::printf("INFO dump %s\n", dump_path.c_str());
            else
                std::printf("INFO dump_failed %s\n", werr.c_str());
        }

        if (step_path.empty())
        {
            std::printf("VERDICT PASS %s\n", json_path.c_str());
            return 0;
        }

        // Fall through to the truth comparison below with the prefix
        // shape standing in for the full replay.
        TopoDS_Shape truth;
        if (!LoadStep(step_path, truth, err))
        {
            std::printf("CHECK truth bad %s\n", err.c_str());
            std::printf("VERDICT FAIL %s\n", json_path.c_str());
            return 1;
        }
        truth = ScaleShape(truth, reader.UnitScale());
        std::printf("CHECK truth ok scale=%g\n", reader.UnitScale());

        const Counts  cr = CountSubs(shape), ct = CountSubs(truth);
        const Box     br = BBox(shape),      bt = BBox(truth);
        {
            char buf[256];
            std::snprintf(buf, sizeof buf,
                "replay solids=%d faces=%d edges=%d | truth solids=%d faces=%d edges=%d",
                cr.solids, cr.faces, cr.edges, ct.solids, ct.faces, ct.edges);
            check(cr.solids == ct.solids, "count_solids", buf);
            std::printf("INFO count_sheets replay=%d truth=%d\n",
                        FreeSheetCount(shape), FreeSheetCount(truth));
        }
        const double diag = std::max(BoxDiag(bt), 1e-12);
        {
            double worst = 0.0;
            if (br.valid && bt.valid)
                for (int k = 0; k < 3; ++k)
                {
                    worst = std::max(worst, std::fabs(br.mn[k] - bt.mn[k]));
                    worst = std::max(worst, std::fabs(br.mx[k] - bt.mx[k]));
                }
            char buf[128];
            std::snprintf(buf, sizeof buf, "worst=%.3g diag=%.3g rel=%.3g",
                          worst, diag, worst / diag);
            check(br.valid && bt.valid && worst / diag <= rel_tol, "bbox", buf);
        }
        {
            // SOLID volume only: sheet flux is accounting noise (see
            // SolidsVolume).
            const double rv = SolidsVolume(shape);
            const double tv = SolidsVolume(truth);
            const double dv = std::fabs(rv - tv) /
                              std::max(std::fabs(tv), 1e-12);
            char buf[160];
            std::snprintf(buf, sizeof buf, "replay=%.9g truth=%.9g rel=%.3g",
                          rv, tv, dv);
            check(dv <= rel_tol, "volume", buf);
            // Per-solid roster, both sides -- same as the full compare;
            // a prefix vs state<K>.step is exactly where a duplicated /
            // missing body needs to be visible per solid.
            SolidRoster("replay", shape);
            SolidRoster("truth",  truth);
        }
        {
            const auto pr = CollectFaceProbes(shape);
            const auto pt = CollectFaceProbes(truth);
            const double tol = face_tol * diag;
            const FaceMatch r2t = MatchFacesAB(pr, pt, tol);
            const FaceMatch t2r = MatchFacesAB(pt, pr, tol);
            char buf[256];
            std::snprintf(buf, sizeof buf,
                "replay->truth %d/%d truth->replay %d/%d",
                r2t.matched, r2t.total, t2r.matched, t2r.total);
            check(r2t.matched == r2t.total && t2r.matched == t2r.total,
                  "face_match", buf);
            // Same unmatched-face DETAIL dump as the full mode -- a
            // prefix compare against a state<K>.step is exactly where
            // the displacement forensics happen, and it had no output.
            const auto dump = [detail_cap](const char* tag,
                                           const std::vector<int>& idxs,
                                           const std::vector<FaceProbe>& probes)
            {
                const size_t cap = (detail_cap > 0) ? (size_t)detail_cap : 12;
                for (size_t k = 0; k < idxs.size() && k < cap; ++k)
                {
                    const int i = idxs[k];
                    std::printf("DETAIL %s idx=%d centre=(%.6g,%.6g,%.6g) "
                                "area=%.6g n=(%.3f,%.3f,%.3f)\n",
                                tag, i, probes[i].centre.X(),
                                probes[i].centre.Y(), probes[i].centre.Z(),
                                probes[i].area,
                                probes[i].normal.X(), probes[i].normal.Y(),
                                probes[i].normal.Z());
                }
                if (idxs.size() > cap)
                    std::printf("DETAIL %s ... and %zu more\n",
                                tag, idxs.size() - cap);
            };
            dump("unmatched_replay_face", r2t.unmatched, pr);
            dump("unmatched_truth_face",  t2r.unmatched, pt);
        }
        std::printf("VERDICT %s %s\n", pass ? "PASS" : "FAIL", json_path.c_str());
        return pass ? 0 : 1;
    }

    // ---- mode: full verify vs .cax.step truth (default) -------------

    // Truth defaults to the sibling the plugin wrote: X.cax.json -> X.cax.step
    if (step_path.empty())
    {
        step_path = json_path;
        const std::string suffix = ".json";
        if (step_path.size() > suffix.size() &&
            Lower(step_path.substr(step_path.size() - suffix.size())) == suffix)
        {
            step_path.erase(step_path.size() - suffix.size());
        }
        step_path += ".step";
    }

    TopoDS_Shape replayed;
    if (!ReplayShape(master, replayed, err))
    {
        std::printf("CHECK replay bad %s\n", err.c_str());
        std::printf("VERDICT FAIL %s\n", json_path.c_str());
        return 1;
    }
    std::printf("CHECK replay ok -\n");

    if (!dump_path.empty())
    {
        const double inv = reader.UnitScale() != 0.0
                         ? 1.0 / reader.UnitScale() : 1.0;
        std::string werr;
        if (WriteStep(dump_path, ScaleShape(replayed, inv), werr))
            std::printf("INFO dump %s\n", dump_path.c_str());
        else
            std::printf("INFO dump_failed %s\n", werr.c_str());
    }

    // 3. truth
    TopoDS_Shape truth;
    if (!LoadStep(step_path, truth, err))
    {
        std::printf("CHECK truth bad %s\n", err.c_str());
        std::printf("VERDICT FAIL %s\n", json_path.c_str());
        return 1;
    }
    truth = ScaleShape(truth, reader.UnitScale());
    std::printf("CHECK truth ok scale=%g\n", reader.UnitScale());

    // 4. compare
    const Counts  cr = CountSubs(replayed), ct = CountSubs(truth);
    const Box     br = BBox(replayed),      bt = BBox(truth);
    const VolArea vr = VolumeAndArea(replayed), vt = VolumeAndArea(truth);

    {
        char buf[256];
        std::snprintf(buf, sizeof buf,
            "replay solids=%d faces=%d edges=%d | truth solids=%d faces=%d edges=%d",
            cr.solids, cr.faces, cr.edges, ct.solids, ct.faces, ct.edges);
        check(cr.solids == ct.solids, "count_solids", buf);
        // Face/edge counts are reported but judged via face matching below:
        // ZW3D's kernel and OCCT legitimately split periodic faces
        // differently, so a raw count inequality is a WARN-grade signal.
        std::printf("INFO count_faces replay=%d truth=%d delta=%d\n",
                    cr.faces, ct.faces, cr.faces - ct.faces);
        std::printf("INFO count_edges replay=%d truth=%d delta=%d\n",
                    cr.edges, ct.edges, cr.edges - ct.edges);
    }

    const double diag = std::max(BoxDiag(bt), 1e-12);
    {
        double worst = 0.0;
        if (br.valid && bt.valid)
        {
            for (int k = 0; k < 3; ++k)
            {
                worst = std::max(worst, std::fabs(br.mn[k] - bt.mn[k]));
                worst = std::max(worst, std::fabs(br.mx[k] - bt.mx[k]));
            }
        }
        char buf[128];
        std::snprintf(buf, sizeof buf, "worst=%.3g diag=%.3g rel=%.3g",
                      worst, diag, worst / diag);
        check(br.valid && bt.valid && worst / diag <= rel_tol, "bbox", buf);
    }
    {
        // SOLID volume only on both sides: sheet flux is accounting
        // noise (see SolidsVolume). The sheet population itself is
        // reported via INFO count_sheets.
        const double rv = SolidsVolume(replayed);
        const double tv = SolidsVolume(truth);
        const double dv = std::fabs(rv - tv) /
                          std::max(std::fabs(tv), 1e-12);
        char buf[160];
        std::snprintf(buf, sizeof buf, "replay=%.9g truth=%.9g rel=%.3g",
                      rv, tv, dv);
        check(dv <= rel_tol, "volume", buf);
        std::printf("INFO count_sheets replay=%d truth=%d\n",
                    FreeSheetCount(replayed), FreeSheetCount(truth));
        // Per-solid roster, both sides: an extra / misplaced body shows
        // up here by volume + bbox instead of needing face clustering.
        SolidRoster("replay", replayed);
        SolidRoster("truth",  truth);
    }
    {
        const double da = std::fabs(vr.area - vt.area) /
                          std::max(std::fabs(vt.area), 1e-12);
        char buf[160];
        std::snprintf(buf, sizeof buf, "replay=%.9g truth=%.9g rel=%.3g",
                      vr.area, vt.area, da);
        check(da <= rel_tol, "area", buf);
    }
    {
        const auto pr = CollectFaceProbes(replayed);
        const auto pt = CollectFaceProbes(truth);
        const double tol = face_tol * diag;

        const FaceMatch r2t = MatchFacesAB(pr, pt, tol);
        const FaceMatch t2r = MatchFacesAB(pt, pr, tol);

        char buf[256];
        std::snprintf(buf, sizeof buf,
            "replay->truth %d/%d (normal_ok=%d area_ok=%d maxd=%.3g) "
            "truth->replay %d/%d",
            r2t.matched, r2t.total, r2t.normal_ok, r2t.area_ok, r2t.max_dist,
            t2r.matched, t2r.total);
        // Zero-diff gate: every face on BOTH sides finds a counterpart.
        // (Fragmentation shows as matched-but-area_bad or unmatched.)
        check(r2t.matched == r2t.total && t2r.matched == t2r.total,
              "face_match", buf);

        const auto dump = [detail_cap](const char* tag, const std::vector<int>& idxs,
                                       const std::vector<FaceProbe>& probes)
        {
            // 12 is enough to localize; --detail-cap N raises it for full
            // displacement clustering (pairing every unmatched truth face
            // with its nearest unmatched replay face offline).
            const size_t cap = (detail_cap > 0) ? (size_t)detail_cap : 12;
            for (size_t k = 0; k < idxs.size() && k < cap; ++k)
            {
                const int i = idxs[k];
                std::printf("DETAIL %s idx=%d centre=(%.6g,%.6g,%.6g) area=%.6g "
                            "n=(%.3f,%.3f,%.3f)\n",
                            tag, i, probes[i].centre.X(), probes[i].centre.Y(),
                            probes[i].centre.Z(), probes[i].area,
                            probes[i].normal.X(), probes[i].normal.Y(),
                            probes[i].normal.Z());
            }
            if (idxs.size() > cap)
                std::printf("DETAIL %s ... and %zu more\n", tag, idxs.size() - cap);
        };
        dump("unmatched_replay_face", r2t.unmatched, pr);
        dump("unmatched_truth_face",  t2r.unmatched, pt);

        // Classify the mismatch. When the unmatched areas on both sides
        // agree in total and every coarse metric passed, the geometry is
        // the same and only the face DECOMPOSITION differs (typically
        // coplanar BOP fragments left unmerged) -- a different bug class
        // than "the replayed body is actually wrong".
        if (r2t.matched != r2t.total || t2r.matched != t2r.total)
        {
            double ar = 0.0, at = 0.0;
            for (int i : r2t.unmatched) ar += pr[i].area;
            for (int i : t2r.unmatched) at += pt[i].area;
            const double denom = std::max({std::fabs(ar), std::fabs(at), 1e-12});
            const bool frag = std::fabs(ar - at) / denom <= 0.01;
            std::printf("INFO diff_class %s unmatched_area replay=%.6g truth=%.6g\n",
                        frag ? "fragmentation" : "geometry", ar, at);
        }
    }

    if (!pass)
    {
        // Point the operator at the bisect flow when the snapshot has
        // per-feature truth to bisect against.
        bool any_state = false;
        for (const auto& f : st.feats) if (f.state.has) { any_state = true; break; }
        std::printf("INFO next %s\n", any_state
            ? "run with --bisect to localize the first diverging feature"
            : "snapshot has no _state blocks; re-export with the "
              "CAX_FEAT_STATE-capable CaxExport.dll, then run --bisect");
    }

    std::printf("VERDICT %s %s\n", pass ? "PASS" : "FAIL", json_path.c_str());
    return pass ? 0 : 1;
}
