#include "cadcvt_c/reader/FreeCadWriter.h"

#include "miniz.h"
#include "pugixml.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace cadcvt {

namespace {

bool CopyFile(const std::string& from, const std::string& to)
{
    std::ifstream in(from, std::ios::binary);
    if (!in) return false;
    std::ofstream out(to, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out << in.rdbuf();
    return in.good() || in.eof();
}

// Format like FreeCAD's PropertyPlacement (16 digits after the decimal),
// so the rewritten attributes match the surrounding file's style.
std::string Fmt(double v)
{
    char b[64];
    std::snprintf(b, sizeof(b), "%.16f", v);
    return b;
}

// Quaternion (x,y,z,w) -> FreeCAD axis-angle (A radians + unit axis).
void QuatToAxisAngle(double qx, double qy, double qz, double qw,
                     double& A, double& ox, double& oy, double& oz)
{
    double n = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (n > 0) { qx/=n; qy/=n; qz/=n; qw/=n; }
    if (qw < 0) { qx=-qx; qy=-qy; qz=-qz; qw=-qw; }
    if (qw > 1.0) qw = 1.0;
    double s = std::sqrt(std::max(0.0, 1.0 - qw*qw));
    if (s < 1e-9) { A = 0.0; ox = 1.0; oy = 0.0; oz = 0.0; }
    else          { A = 2.0 * std::acos(qw); ox = qx/s; oy = qy/s; oz = qz/s; }
}

std::string MakePlacementElement(const BodyPlacement& bp, double unit_scale)
{
    double A, ox, oy, oz;
    QuatToAxisAngle(bp.qx, bp.qy, bp.qz, bp.qw, A, ox, oy, oz);
    std::string s = "<PropertyPlacement";
    s += " Px=\"" + Fmt(bp.px / unit_scale) + "\"";   // metres -> FreeCAD mm
    s += " Py=\"" + Fmt(bp.py / unit_scale) + "\"";
    s += " Pz=\"" + Fmt(bp.pz / unit_scale) + "\"";
    s += " Q0=\"" + Fmt(bp.qx) + "\"";
    s += " Q1=\"" + Fmt(bp.qy) + "\"";
    s += " Q2=\"" + Fmt(bp.qz) + "\"";
    s += " Q3=\"" + Fmt(bp.qw) + "\"";
    s += " A=\""  + Fmt(A)  + "\"";
    s += " Ox=\"" + Fmt(ox) + "\"";
    s += " Oy=\"" + Fmt(oy) + "\"";
    s += " Oz=\"" + Fmt(oz) + "\"";
    s += "/>";
    return s;
}

// Byte span [lt, end) of the element whose tag-name pugixml reported at
// `name_off` in `xml` (an offset_debug() value). Scans back to '<' and forward
// to the first unquoted '>', so it captures the whole self-closing tag.
bool ElementSpan(const std::string& xml, ptrdiff_t name_off,
                 size_t& lt, size_t& end)
{
    if (name_off <= 0 || (size_t)name_off >= xml.size()) return false;
    size_t p = (size_t)name_off;
    while (p > 0 && xml[p] != '<') --p;
    if (xml[p] != '<') return false;
    lt = p;
    char q = 0;
    size_t i = lt + 1;
    for (; i < xml.size(); ++i) {
        char c = xml[i];
        if (q)              { if (c == q) q = 0; }
        else if (c == '"' || c == '\'') q = c;
        else if (c == '>')  break;
    }
    if (i >= xml.size()) return false;
    end = i + 1;
    return true;
}

} // namespace

bool WriteFreeCadPlacements(const std::string& src_path,
                            const std::string& out_path,
                            const std::map<std::string, BodyPlacement>& bodies,
                            double unit_scale,
                            const std::string& backup_path,
                            int* written,
                            std::string* err)
{
    auto fail = [&](const std::string& m) { if (err) *err = m; return false; };
    if (written) *written = 0;
    if (unit_scale == 0.0) return fail("unit_scale is zero");

    // ---- open source zip + read Document.xml as raw bytes ----
    mz_zip_archive zr;
    std::memset(&zr, 0, sizeof(zr));
    if (!mz_zip_reader_init_file(&zr, src_path.c_str(), 0))
        return fail("cannot open source zip: " + src_path);

    int didx = mz_zip_reader_locate_file(&zr, "Document.xml", nullptr, 0);
    if (didx < 0) { mz_zip_reader_end(&zr); return fail("no Document.xml in " + src_path); }
    size_t xml_sz = 0;
    void*  xml_raw = mz_zip_reader_extract_to_heap(&zr, didx, &xml_sz, 0);
    if (!xml_raw) { mz_zip_reader_end(&zr); return fail("extract Document.xml failed"); }
    std::string xml(static_cast<const char*>(xml_raw), xml_sz);
    mz_free(xml_raw);

    // ---- LOCATE the target placements (pugixml, parse_minimal so byte offsets
    //      match the raw bytes -- no EOL/escape rewriting). We never serialise
    //      the tree; we only read offsets and byte-patch the raw XML, so every
    //      other byte of Document.xml (topological naming, hashers, ...) is
    //      preserved exactly as FreeCAD wrote it. ----
    pugi::xml_document doc;
    pugi::xml_parse_result pr = doc.load_buffer(xml.data(), xml.size(),
                                                pugi::parse_minimal);
    if (!pr) { mz_zip_reader_end(&zr); return fail(std::string("xml parse: ") + pr.description()); }

    struct Edit { size_t lt, end; std::string repl; };
    std::vector<Edit> edits;
    pugi::xml_node objdata = doc.child("Document").child("ObjectData");
    for (pugi::xml_node obj = objdata.child("Object"); obj;
         obj = obj.next_sibling("Object"))
    {
        auto it = bodies.find(obj.attribute("name").as_string());
        if (it == bodies.end()) continue;
        pugi::xml_node place;
        for (pugi::xml_node p = obj.child("Properties").child("Property"); p;
             p = p.next_sibling("Property"))
        {
            if (std::strcmp(p.attribute("name").as_string(), "Placement") == 0) {
                place = p.child("PropertyPlacement");
                break;
            }
        }
        if (!place) continue;
        size_t lt = 0, end = 0;
        if (!ElementSpan(xml, place.offset_debug(), lt, end)) continue;
        edits.push_back({lt, end, MakePlacementElement(it->second, unit_scale)});
    }

    // apply edits last-to-first so earlier offsets stay valid
    std::sort(edits.begin(), edits.end(),
              [](const Edit& a, const Edit& b) { return a.lt > b.lt; });
    for (const Edit& e : edits) xml.replace(e.lt, e.end - e.lt, e.repl);

    // ---- backup before any destructive write. NEVER overwrite an existing
    //      backup: the first one is the pristine original, and later saves
    //      must not clobber it with an already-edited file. ----
    if (!backup_path.empty()) {
        std::ifstream have(backup_path, std::ios::binary);
        bool exists = have.good();
        have.close();
        if (!exists && !CopyFile(src_path, backup_path)) {
            mz_zip_reader_end(&zr);
            return fail("backup failed: " + backup_path);
        }
    }

    // ---- re-zip: substitute Document.xml, copy every other entry verbatim ----
    std::string tmp = out_path + ".tmp";
    mz_zip_archive zw;
    std::memset(&zw, 0, sizeof(zw));
    if (!mz_zip_writer_init_file(&zw, tmp.c_str(), 0)) {
        mz_zip_reader_end(&zr);
        return fail("cannot create temp zip: " + tmp);
    }
    mz_uint nfiles = mz_zip_reader_get_num_files(&zr);
    bool ok = true;
    for (mz_uint i = 0; i < nfiles && ok; ++i) {
        mz_zip_archive_file_stat st;
        if (!mz_zip_reader_file_stat(&zr, i, &st)) { ok = false; break; }
        if (std::strcmp(st.m_filename, "Document.xml") == 0)
            ok = mz_zip_writer_add_mem(&zw, "Document.xml",
                                       xml.data(), xml.size(), MZ_DEFAULT_LEVEL);
        else
            ok = mz_zip_writer_add_from_zip_reader(&zw, &zr, i);
    }
    if (ok) ok = mz_zip_writer_finalize_archive(&zw);
    mz_zip_writer_end(&zw);
    mz_zip_reader_end(&zr);
    if (!ok) { std::remove(tmp.c_str()); return fail("zip write failed"); }

    std::remove(out_path.c_str());
    if (std::rename(tmp.c_str(), out_path.c_str()) != 0) {
        std::remove(tmp.c_str());
        return fail("rename into place failed: " + out_path);
    }
    if (written) *written = static_cast<int>(edits.size());
    return true;
}

} // namespace cadcvt
