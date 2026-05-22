// ============================================================
// tools/brp_diff/main.cpp
//
// Auto-diff cax's per-feature .brp dumps against the matching
// PartShapeN.brp entries inside a FreeCAD .FCStd zip.
//
// Pairs by feature name:
//   cax_<id>_<name>.brp        (written by Replayer when dump hook
//                               is on; see Replayer.cpp)
//   <name> -> PartShapeN.brp   (read from Document.xml's
//                               Part::PropertyPartShape properties)
//
// For each pair, compares:
//   - shape type (COMPOUND / SOLID / ...)
//   - sub-shape counts (solids / shells / faces / edges / verts)
//   - axis-aligned bounding box
//   - volume + surface area (BRepGProp)
//   - per-face match: nearest-neighbour centroid distance with
//     normal alignment check; flags unmatched faces on either
//     side as candidate fragmentation
//   - median face area ratio (sniff test for "cax fragmented one
//     FreeCAD face into N smaller ones")
//
// Usage:
//   brp_diff <fcstd_path> [<dump_dir>]
//
//   <dump_dir> defaults to the current working directory.
//
// Exit code: 0 if every paired feature passes all checks, 1 if any
// pair fails, 2 on usage / IO errors.
// ============================================================

#include "miniz.h"
#include "pugixml.hpp"

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <TopAbs.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include "cadapp_c/resolve/TopoGeomUtils.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{

// ---- ANSI status tags (degraded to plain text on Windows shells
// that don't render escape sequences; the tags are still readable).
const char* tagOK()   { return "[OK]"; }
const char* tagWARN() { return "[WARN]"; }
const char* tagFAIL() { return "[FAIL]"; }

const char* ShapeTypeName(TopAbs_ShapeEnum t)
{
    switch (t)
    {
    case TopAbs_COMPOUND:  return "COMPOUND";
    case TopAbs_COMPSOLID: return "COMPSOLID";
    case TopAbs_SOLID:     return "SOLID";
    case TopAbs_SHELL:     return "SHELL";
    case TopAbs_FACE:      return "FACE";
    case TopAbs_WIRE:      return "WIRE";
    case TopAbs_EDGE:      return "EDGE";
    case TopAbs_VERTEX:    return "VERTEX";
    case TopAbs_SHAPE:     return "SHAPE";
    }
    return "?";
}

struct Counts { int solids, shells, faces, edges, verts; };

Counts CountSubs(const TopoDS_Shape& s)
{
    Counts c{};
    if (s.IsNull()) return c;

    TopTools_IndexedMapOfShape m;
    TopExp::MapShapes(s, TopAbs_SOLID,  m); c.solids = m.Extent(); m.Clear();
    TopExp::MapShapes(s, TopAbs_SHELL,  m); c.shells = m.Extent(); m.Clear();
    TopExp::MapShapes(s, TopAbs_FACE,   m); c.faces  = m.Extent(); m.Clear();
    TopExp::MapShapes(s, TopAbs_EDGE,   m); c.edges  = m.Extent(); m.Clear();
    TopExp::MapShapes(s, TopAbs_VERTEX, m); c.verts  = m.Extent();
    return c;
}

struct VolArea { double volume, area; };

VolArea VolumeAndArea(const TopoDS_Shape& s)
{
    VolArea r{0.0, 0.0};
    if (s.IsNull()) return r;
    GProp_GProps gv, gs;
    BRepGProp::VolumeProperties(s, gv);
    BRepGProp::SurfaceProperties(s, gs);
    r.volume = gv.Mass();
    r.area   = gs.Mass();
    return r;
}

struct Box { double mn[3], mx[3]; bool valid = false; };

Box BBox(const TopoDS_Shape& s)
{
    Box b{};
    if (s.IsNull()) return b;
    Bnd_Box bb;
    BRepBndLib::Add(s, bb);
    if (bb.IsVoid()) return b;
    bb.Get(b.mn[0], b.mn[1], b.mn[2], b.mx[0], b.mx[1], b.mx[2]);
    b.valid = true;
    return b;
}

struct FaceProbe
{
    gp_Pnt centre;
    gp_Dir normal;
    double area;
    bool   valid;
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
        FaceProbe p{};
        p.area  = cadapp::FaceArea(f);
        p.valid = cadapp::FaceCenter(f, p.centre, p.normal);
        out.push_back(p);
    }
    return out;
}

// For each face on side A, find its nearest-by-centroid match on
// side B. Returns (matched_count_within_tol, mean_dist_among_matches,
// unmatched_indices_on_A).
struct MatchStats
{
    int    matched;
    double mean_dist;
    double max_dist;
    int    normal_ok;     // matches where |dot(nA, nB)| > 0.95
};

MatchStats MatchFacesAB(const std::vector<FaceProbe>& A,
                        const std::vector<FaceProbe>& B,
                        double                        tol_m)
{
    MatchStats s{0, 0.0, 0.0, 0};
    double sum = 0.0;
    for (const auto& fa : A)
    {
        if (!fa.valid) continue;
        double best = std::numeric_limits<double>::max();
        int    best_j = -1;
        for (size_t j = 0; j < B.size(); ++j)
        {
            if (!B[j].valid) continue;
            double dx = fa.centre.X() - B[j].centre.X();
            double dy = fa.centre.Y() - B[j].centre.Y();
            double dz = fa.centre.Z() - B[j].centre.Z();
            double d  = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (d < best) { best = d; best_j = (int)j; }
        }
        if (best_j >= 0 && best <= tol_m)
        {
            ++s.matched;
            sum += best;
            if (best > s.max_dist) s.max_dist = best;

            double dot = fa.normal.X() * B[best_j].normal.X()
                       + fa.normal.Y() * B[best_j].normal.Y()
                       + fa.normal.Z() * B[best_j].normal.Z();
            if (std::fabs(dot) > 0.95) ++s.normal_ok;
        }
    }
    s.mean_dist = (s.matched > 0) ? (sum / s.matched) : 0.0;
    return s;
}

double Median(std::vector<double> xs)
{
    if (xs.empty()) return 0.0;
    std::sort(xs.begin(), xs.end());
    return xs[xs.size() / 2];
}


// ---- .FCStd extraction --------------------------------------

struct FcstdArchive
{
    mz_zip_archive zip{};
    bool           opened = false;

    bool Open(const std::string& path, std::string* err)
    {
        std::memset(&zip, 0, sizeof(zip));
        if (!mz_zip_reader_init_file(&zip, path.c_str(), 0))
        {
            if (err) *err = "miniz: cannot open: " + path;
            return false;
        }
        opened = true;
        return true;
    }

    ~FcstdArchive() { if (opened) mz_zip_reader_end(&zip); }

    bool ReadEntry(const std::string& name, std::string& out) const
    {
        int idx = mz_zip_reader_locate_file(
            const_cast<mz_zip_archive*>(&zip), name.c_str(), nullptr, 0);
        if (idx < 0) return false;
        size_t sz = 0;
        void*  buf = mz_zip_reader_extract_to_heap(
            const_cast<mz_zip_archive*>(&zip), idx, &sz, 0);
        if (!buf) return false;
        out.assign(static_cast<const char*>(buf), sz);
        mz_free(buf);
        return true;
    }

    bool LoadBRep(const std::string& name, TopoDS_Shape& out) const
    {
        std::string body;
        if (!ReadEntry(name, body)) return false;
        std::istringstream iss(body);
        BRep_Builder        builder;
        TopoDS_Shape        s;
        try { BRepTools::Read(s, iss, builder); }
        catch (...) { return false; }
        if (s.IsNull()) return false;
        out = s;
        return true;
    }
};

// Parse Document.xml -> map<object_name, PartShapeN.brp>.
std::map<std::string, std::string>
ParseFeatToBrep(const std::string& doc_xml, std::string* err)
{
    std::map<std::string, std::string> result;
    pugi::xml_document doc;
    auto res = doc.load_buffer(doc_xml.data(), doc_xml.size());
    if (!res)
    {
        if (err) *err = std::string("pugixml: ") + res.description();
        return result;
    }
    auto root = doc.child("Document");
    if (!root)
    {
        if (err) *err = "missing <Document>";
        return result;
    }
    auto od = root.child("ObjectData");
    for (auto obj = od.child("Object"); obj; obj = obj.next_sibling("Object"))
    {
        std::string name = obj.attribute("name").value();
        auto props = obj.child("Properties");
        for (auto prop = props.child("Property"); prop; prop = prop.next_sibling("Property"))
        {
            if (std::string(prop.attribute("type").value()) != "Part::PropertyPartShape")
                continue;
            auto part = prop.child("Part");
            if (!part) continue;
            std::string file = part.attribute("file").value();
            if (!file.empty()) result[name] = file;
            break;
        }
    }
    return result;
}

// ---- cax dump file discovery --------------------------------

// Parse "cax_<id>_<name>.brp" -> { id, name }. Returns nullopt on
// malformed filenames.
struct DumpKey { uint32_t id; std::string name; };

std::optional<DumpKey> ParseDumpName(const std::string& stem)
{
    // stem == "cax_12_Pad"
    if (stem.rfind("cax_", 0) != 0) return std::nullopt;
    size_t p1 = 4;
    size_t p2 = stem.find('_', p1);
    if (p2 == std::string::npos) return std::nullopt;
    DumpKey k{};
    try { k.id = (uint32_t)std::stoul(stem.substr(p1, p2 - p1)); }
    catch (...) { return std::nullopt; }
    k.name = stem.substr(p2 + 1);
    if (k.name.empty()) return std::nullopt;
    return k;
}


// ---- per-feature comparison --------------------------------

struct PairResult
{
    std::string name;
    uint32_t    id;
    bool        passed = true;
    bool        skipped = false;
    std::string note;
};

void PrintCounts(const Counts& a, const Counts& b)
{
    bool ok = (a.solids==b.solids && a.shells==b.shells &&
               a.faces==b.faces && a.edges==b.edges && a.verts==b.verts);
    std::printf("  counts        FreeCAD: solids=%d shells=%d faces=%d edges=%d verts=%d\n",
                a.solids, a.shells, a.faces, a.edges, a.verts);
    std::printf("                cax:     solids=%d shells=%d faces=%d edges=%d verts=%d  %s",
                b.solids, b.shells, b.faces, b.edges, b.verts,
                ok ? tagOK() : tagFAIL());
    if (!ok && b.faces > a.faces) {
        std::printf("  (+%d faces -- fragmented?)", b.faces - a.faces);
    } else if (!ok && b.faces < a.faces) {
        std::printf("  (-%d faces -- merged or missing?)", a.faces - b.faces);
    }
    std::putchar('\n');
}

bool PrintVolumeArea(const VolArea& a, const VolArea& b)
{
    bool v_ok = std::fabs(a.volume - b.volume) <= 1e-6 * std::max(std::fabs(a.volume), 1.0);
    bool s_ok = std::fabs(a.area   - b.area)   <= 1e-3 * std::max(std::fabs(a.area),   1.0);
    std::printf("  volume        FreeCAD: %.6e   cax: %.6e   %s\n",
                a.volume, b.volume, v_ok ? tagOK() : tagFAIL());
    std::printf("  surface area  FreeCAD: %.6e   cax: %.6e   %s\n",
                a.area,   b.area,   s_ok ? tagOK() : tagWARN());
    return v_ok && s_ok;
}

bool PrintBBox(const Box& a, const Box& b)
{
    if (!a.valid || !b.valid) {
        std::printf("  bbox          (one side is void)                      %s\n", tagWARN());
        return false;
    }
    auto close = [](const double x[3], const double y[3]) {
        return std::fabs(x[0]-y[0]) < 1e-4
            && std::fabs(x[1]-y[1]) < 1e-4
            && std::fabs(x[2]-y[2]) < 1e-4;
    };
    bool ok = close(a.mn, b.mn) && close(a.mx, b.mx);
    std::printf("  bbox          FreeCAD: (%7.4f,%7.4f,%7.4f)..(%7.4f,%7.4f,%7.4f)\n",
                a.mn[0], a.mn[1], a.mn[2], a.mx[0], a.mx[1], a.mx[2]);
    std::printf("                cax:     (%7.4f,%7.4f,%7.4f)..(%7.4f,%7.4f,%7.4f)  %s\n",
                b.mn[0], b.mn[1], b.mn[2], b.mx[0], b.mx[1], b.mx[2],
                ok ? tagOK() : tagFAIL());
    return ok;
}

bool PrintFaceMatch(const std::vector<FaceProbe>& a,
                    const std::vector<FaceProbe>& b)
{
    const double tol = 1e-3;   // 1mm centroid-match tolerance
    MatchStats fc_to_cax = MatchFacesAB(a, b, tol);
    MatchStats cax_to_fc = MatchFacesAB(b, a, tol);

    int total_a = 0, total_b = 0;
    std::vector<double> area_a, area_b;
    for (const auto& f : a) if (f.valid) { ++total_a; area_a.push_back(f.area); }
    for (const auto& f : b) if (f.valid) { ++total_b; area_b.push_back(f.area); }

    bool ok = (fc_to_cax.matched == total_a)
           && (cax_to_fc.matched == total_b)
           && (fc_to_cax.normal_ok == fc_to_cax.matched);

    std::printf("  face match    FreeCAD->cax  %d/%d within %.0fmm "
                "(normals_ok=%d, mean=%.4fm, max=%.4fm)\n",
                fc_to_cax.matched, total_a, tol*1000.0,
                fc_to_cax.normal_ok, fc_to_cax.mean_dist, fc_to_cax.max_dist);
    std::printf("                cax->FreeCAD  %d/%d within %.0fmm "
                "(mean=%.4fm, max=%.4fm)            %s\n",
                cax_to_fc.matched, total_b, tol*1000.0,
                cax_to_fc.mean_dist, cax_to_fc.max_dist,
                ok ? tagOK() : tagFAIL());

    double med_a = Median(area_a);
    double med_b = Median(area_b);
    double ratio = (med_a > 0.0) ? (med_b / med_a) : 0.0;
    std::printf("                median face area  FreeCAD: %.4e  cax: %.4e  ratio cax/fc=%.3f",
                med_a, med_b, ratio);
    if (med_a > 0 && (ratio < 0.7 || ratio > 1.4)) {
        std::printf("  %s (likely fragmentation/merge)", tagWARN());
    }
    std::putchar('\n');
    return ok;
}

PairResult DiffPair(const std::string&  feat_name,
                    uint32_t            feat_id,
                    const std::string&  freecad_brep_name,
                    const fs::path&     cax_dump_path,
                    const FcstdArchive& archive,
                    double              unit_scale)
{
    PairResult r;
    r.name = feat_name;
    r.id   = feat_id;

    std::printf("\n== feat=%s id=%u  freecad=%s  cax=%s\n",
                feat_name.c_str(), feat_id,
                freecad_brep_name.c_str(),
                cax_dump_path.filename().string().c_str());

    TopoDS_Shape sa_raw;
    if (!archive.LoadBRep(freecad_brep_name, sa_raw)) {
        std::printf("  FreeCAD .brp missing or unreadable: %s   %s\n",
                    freecad_brep_name.c_str(), tagFAIL());
        r.passed = false;
        return r;
    }

    // Scale FreeCAD's shape into cax's units. Skip if unit_scale ==
    // 1 (identity) so we don't pay for a meaningless BOP rebuild.
    TopoDS_Shape sa = sa_raw;
    if (std::fabs(unit_scale - 1.0) > 1e-12)
    {
        gp_Trsf t;
        t.SetScale(gp_Pnt(0, 0, 0), unit_scale);
        BRepBuilderAPI_Transform xf(sa_raw, t, /*copy=*/Standard_True);
        if (xf.IsDone()) {
            sa = xf.Shape();
        } else {
            std::printf("  warning: scaling FreeCAD shape by %g failed   %s\n",
                        unit_scale, tagWARN());
        }
    }

    TopoDS_Shape sb;
    {
        std::ifstream f(cax_dump_path, std::ios::binary);
        if (!f) {
            std::printf("  cax dump missing: %s   %s\n",
                        cax_dump_path.string().c_str(), tagFAIL());
            r.passed = false;
            return r;
        }
        BRep_Builder builder;
        try { BRepTools::Read(sb, f, builder); }
        catch (...) {
            std::printf("  cax dump unreadable                 %s\n", tagFAIL());
            r.passed = false;
            return r;
        }
        if (sb.IsNull()) {
            std::printf("  cax dump empty                      %s\n", tagFAIL());
            r.passed = false;
            return r;
        }
    }

    bool type_ok = (sa.ShapeType() == sb.ShapeType());
    std::printf("  type          FreeCAD: %s   cax: %s   %s\n",
                ShapeTypeName(sa.ShapeType()),
                ShapeTypeName(sb.ShapeType()),
                type_ok ? tagOK() : tagWARN());

    Counts ca = CountSubs(sa);
    Counts cb = CountSubs(sb);
    PrintCounts(ca, cb);

    Box ba = BBox(sa);
    Box bb = BBox(sb);
    bool bbox_ok = PrintBBox(ba, bb);

    VolArea va = VolumeAndArea(sa);
    VolArea vb = VolumeAndArea(sb);
    bool va_ok = PrintVolumeArea(va, vb);

    bool face_ok = PrintFaceMatch(CollectFaceProbes(sa), CollectFaceProbes(sb));

    r.passed = type_ok && bbox_ok && va_ok && face_ok
            && (ca.faces == cb.faces) && (ca.edges == cb.edges);
    std::printf("  >>> %s\n", r.passed ? tagOK() : tagFAIL());
    return r;
}

} // anonymous namespace

int main(int argc, char** argv)
{
    // ---- Parse args ----
    //
    // brp_diff <fcstd_path> [<dump_dir>] [--unit-scale=K]
    //
    // FreeCAD writes its native units (mm) to .brp; cax stores
    // metres (it multiplies by m_unit_scale=0.001 on read). The
    // comparison applies a 1/K scaling to FreeCAD's shape so both
    // sides land in cax's metres frame. Override with --unit-scale
    // for non-default reader configurations.
    if (argc < 2) {
        std::fprintf(stderr,
            "usage: brp_diff <fcstd_path> [<dump_dir>] [--unit-scale=K]\n"
            "  Compares cax_<id>_<name>.brp dumps against the\n"
            "  matching PartShapeN.brp entries inside <fcstd_path>.\n"
            "  <dump_dir> defaults to current working directory.\n"
            "  --unit-scale=K (default 0.001): scaling factor applied\n"
            "    to FreeCAD's shape before diffing. 0.001 matches the\n"
            "    reader's mm->m default.\n");
        return 2;
    }

    std::string fcstd_path = argv[1];
    fs::path    dump_dir   = fs::current_path();
    double      unit_scale = 0.001;

    for (int i = 2; i < argc; ++i)
    {
        std::string a = argv[i];
        if (a.rfind("--unit-scale=", 0) == 0) {
            unit_scale = std::stod(a.substr(13));
        } else {
            dump_dir = fs::path(a);
        }
    }

    // ---- 1. Open .FCStd, parse Document.xml ----
    FcstdArchive archive;
    {
        std::string err;
        if (!archive.Open(fcstd_path, &err)) {
            std::fprintf(stderr, "%s\n", err.c_str());
            return 2;
        }
    }
    std::string doc_xml;
    if (!archive.ReadEntry("Document.xml", doc_xml)) {
        std::fprintf(stderr, "no Document.xml inside %s\n", fcstd_path.c_str());
        return 2;
    }
    std::string parse_err;
    auto feat_to_brep = ParseFeatToBrep(doc_xml, &parse_err);
    if (feat_to_brep.empty()) {
        std::fprintf(stderr, "no Part::PropertyPartShape entries; %s\n",
                     parse_err.c_str());
        return 2;
    }

    std::printf("brp_diff:\n  fcstd:      %s\n  dump_dir:   %s\n"
                "  unit_scale: %g (applied to FreeCAD side)\n"
                "  FreeCAD has %zu shape entries\n",
                fcstd_path.c_str(),
                dump_dir.string().c_str(),
                unit_scale,
                feat_to_brep.size());

    // ---- 2. Collect cax dumps ----
    std::vector<fs::path> dumps;
    if (!fs::is_directory(dump_dir)) {
        std::fprintf(stderr, "not a directory: %s\n", dump_dir.string().c_str());
        return 2;
    }
    for (const auto& ent : fs::directory_iterator(dump_dir))
    {
        if (!ent.is_regular_file()) continue;
        std::string stem = ent.path().stem().string();
        std::string ext  = ent.path().extension().string();
        if (ext != ".brp" && ext != ".BRP") continue;
        if (stem.rfind("cax_", 0) != 0) continue;
        dumps.push_back(ent.path());
    }
    if (dumps.empty()) {
        std::fprintf(stderr,
            "no cax_*.brp files in %s\n"
            "  (run editor.exe with the .FCStd first; "
            "the dump hook in Replayer.cpp writes them to CWD)\n",
            dump_dir.string().c_str());
        return 2;
    }
    std::sort(dumps.begin(), dumps.end());
    std::printf("  cax dumps found: %zu\n", dumps.size());

    // ---- 3. Diff each pair ----
    int failed = 0;
    int total  = 0;
    int skipped = 0;
    for (const auto& path : dumps)
    {
        auto key = ParseDumpName(path.stem().string());
        if (!key) {
            std::printf("\n== skip (cannot parse name): %s\n",
                        path.filename().string().c_str());
            ++skipped;
            continue;
        }
        auto it = feat_to_brep.find(key->name);
        if (it == feat_to_brep.end()) {
            std::printf("\n== feat=%s id=%u  (no FreeCAD shape entry; skip)\n",
                        key->name.c_str(), key->id);
            ++skipped;
            continue;
        }
        ++total;
        PairResult r = DiffPair(key->name, key->id, it->second, path,
                                archive, unit_scale);
        if (!r.passed) ++failed;
    }

    std::printf("\n----\n%d pairs compared, %d failed, %d skipped\n",
                total, failed, skipped);
    return failed == 0 ? 0 : 1;
}
