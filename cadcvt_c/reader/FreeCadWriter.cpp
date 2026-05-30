#include "cadcvt_c/reader/FreeCadWriter.h"

#include "miniz.h"
#include "pugixml.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

namespace cadcvt {

namespace {

// Copy a whole file (for the safety backup). Returns false on any I/O error.
bool CopyFile(const std::string& from, const std::string& to)
{
    std::ifstream in(from, std::ios::binary);
    if (!in) return false;
    std::ofstream out(to, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out << in.rdbuf();
    return in.good() || in.eof();
}

// Set an attribute to a double with full round-trip precision (pugixml's
// built-in double formatting can truncate the 16-digit poses FreeCAD writes).
void SetAttrD(pugi::xml_node n, const char* name, double v)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.17g", v);
    pugi::xml_attribute a = n.attribute(name);
    if (a) a.set_value(buf);
    else   n.append_attribute(name).set_value(buf);
}

// Quaternion (x,y,z,w) -> FreeCAD axis-angle (A radians + unit axis), matching
// the redundant A/Ox/Oy/Oz that FreeCAD stores alongside Q0..Q3.
void QuatToAxisAngle(double qx, double qy, double qz, double qw,
                     double& A, double& ox, double& oy, double& oz)
{
    double n = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (n > 0) { qx/=n; qy/=n; qz/=n; qw/=n; }
    if (qw < 0) { qx=-qx; qy=-qy; qz=-qz; qw=-qw; }   // canonical: A in [0, pi]
    if (qw > 1.0) qw = 1.0;
    double s = std::sqrt(std::max(0.0, 1.0 - qw*qw));
    if (s < 1e-9) { A = 0.0; ox = 1.0; oy = 0.0; oz = 0.0; }   // ~no rotation
    else          { A = 2.0 * std::acos(qw); ox = qx/s; oy = qy/s; oz = qz/s; }
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

    // ---- open the source zip ----
    mz_zip_archive zr;
    std::memset(&zr, 0, sizeof(zr));
    if (!mz_zip_reader_init_file(&zr, src_path.c_str(), 0))
        return fail("cannot open source zip: " + src_path);

    // ---- extract + edit Document.xml ----
    int didx = mz_zip_reader_locate_file(&zr, "Document.xml", nullptr, 0);
    if (didx < 0) { mz_zip_reader_end(&zr); return fail("no Document.xml in " + src_path); }
    size_t xml_sz = 0;
    void*  xml_raw = mz_zip_reader_extract_to_heap(&zr, didx, &xml_sz, 0);
    if (!xml_raw) { mz_zip_reader_end(&zr); return fail("extract Document.xml failed"); }

    pugi::xml_document doc;
    pugi::xml_parse_result pr =
        doc.load_buffer(xml_raw, xml_sz, pugi::parse_full);
    mz_free(xml_raw);
    if (!pr) { mz_zip_reader_end(&zr); return fail(std::string("xml parse: ") + pr.description()); }

    int n_written = 0;
    pugi::xml_node objdata = doc.child("Document").child("ObjectData");
    for (pugi::xml_node obj = objdata.child("Object"); obj;
         obj = obj.next_sibling("Object"))
    {
        auto it = bodies.find(obj.attribute("name").as_string());
        if (it == bodies.end()) continue;

        // find <Property name="Placement"> ... <PropertyPlacement .../>
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

        const BodyPlacement& bp = it->second;
        SetAttrD(place, "Px", bp.px / unit_scale);   // metres -> FreeCAD mm
        SetAttrD(place, "Py", bp.py / unit_scale);
        SetAttrD(place, "Pz", bp.pz / unit_scale);
        SetAttrD(place, "Q0", bp.qx);
        SetAttrD(place, "Q1", bp.qy);
        SetAttrD(place, "Q2", bp.qz);
        SetAttrD(place, "Q3", bp.qw);
        double A, ox, oy, oz;
        QuatToAxisAngle(bp.qx, bp.qy, bp.qz, bp.qw, A, ox, oy, oz);
        SetAttrD(place, "A",  A);
        SetAttrD(place, "Ox", ox);
        SetAttrD(place, "Oy", oy);
        SetAttrD(place, "Oz", oz);
        ++n_written;
    }

    std::ostringstream oss;
    doc.save(oss, "  ");
    std::string new_xml = oss.str();

    // ---- backup the source before writing anything destructive ----
    if (!backup_path.empty() && !CopyFile(src_path, backup_path)) {
        mz_zip_reader_end(&zr);
        return fail("backup failed: " + backup_path);
    }

    // ---- write the new zip to a temp file: substitute Document.xml, copy
    //      every other entry verbatim ----
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
        if (std::strcmp(st.m_filename, "Document.xml") == 0) {
            ok = mz_zip_writer_add_mem(&zw, "Document.xml",
                                       new_xml.data(), new_xml.size(),
                                       MZ_DEFAULT_LEVEL);
        } else {
            ok = mz_zip_writer_add_from_zip_reader(&zw, &zr, i);
        }
    }
    if (ok) ok = mz_zip_writer_finalize_archive(&zw);
    mz_zip_writer_end(&zw);
    mz_zip_reader_end(&zr);

    if (!ok) { std::remove(tmp.c_str()); return fail("zip write failed"); }

    // ---- atomically replace the target with the temp file ----
    std::remove(out_path.c_str());            // no-op if absent
    if (std::rename(tmp.c_str(), out_path.c_str()) != 0) {
        std::remove(tmp.c_str());
        return fail("rename into place failed: " + out_path);
    }

    if (written) *written = n_written;
    return true;
}

} // namespace cadcvt
