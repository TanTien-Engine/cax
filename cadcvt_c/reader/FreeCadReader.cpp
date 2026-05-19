#include "cadcvt_c/reader/FreeCadReader.h"

// Single-file deps in thirdparty/. See PLACE_*_HERE.txt in each dir
// for how to drop them in.
#include "miniz.h"
#include "pugixml.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

// ============================================================
// FreeCadReader.cpp
//
// Section A : tiny utilities (string suffix, attribute helpers)
// Section B : .FCStd zip extractor (miniz wrapper)
// Section C : sketch geometry / constraint parsers
// Section D : feature parsers (Pad / Pocket / Fillet / ...)
// Section E : top-level walk + ReadFile / ParseDocumentXml glue
// ============================================================

namespace cadcvt
{

// ============================================================
// Section A: utilities
// ============================================================

namespace
{

bool EndsWithICase(const std::string& s, const char* suffix)
{
    size_t sl = s.size();
    size_t xl = std::strlen(suffix);
    if (xl > sl) {
        return false;
    }
    for (size_t i = 0; i < xl; ++i)
    {
        char a = (char)std::tolower((unsigned char)s[sl - xl + i]);
        char b = (char)std::tolower((unsigned char)suffix[i]);
        if (a != b) {
            return false;
        }
    }
    return true;
}

double AttrDouble(const pugi::xml_node& n, const char* name, double def = 0.0)
{
    auto a = n.attribute(name);
    if (!a) {
        return def;
    }
    return a.as_double(def);
}

int AttrInt(const pugi::xml_node& n, const char* name, int def = 0)
{
    auto a = n.attribute(name);
    if (!a) {
        return def;
    }
    return a.as_int(def);
}

bool AttrBool(const pugi::xml_node& n, const char* name, bool def = false)
{
    auto a = n.attribute(name);
    if (!a) {
        return def;
    }
    // FreeCAD writes booleans as "true"/"false" or "0"/"1"; pugixml
    // handles both via as_bool.
    return a.as_bool(def);
}

const char* AttrStr(const pugi::xml_node& n, const char* name, const char* def = "")
{
    auto a = n.attribute(name);
    if (!a) {
        return def;
    }
    return a.value();
}

// Find a <Property name="X"> child node. FreeCAD names every
// property; the actual value is a child element (e.g.
// PropertyPlacement, Float, Bool, ...).
pugi::xml_node FindProperty(const pugi::xml_node& props_node, const char* prop_name)
{
    for (auto p = props_node.child("Property"); p; p = p.next_sibling("Property"))
    {
        if (std::strcmp(p.attribute("name").value(), prop_name) == 0) {
            return p;
        }
    }
    return pugi::xml_node{};
}

// Read a scalar property value. FreeCAD wraps it in a child element
// whose tag name varies (e.g. <Float value="3.0" />,
// <Bool value="true" />, <Integer value="1" />, <String value="..."/>).
double PropDouble(const pugi::xml_node& props_node, const char* prop_name, double def = 0.0)
{
    auto p = FindProperty(props_node, prop_name);
    if (!p) {
        return def;
    }
    auto v = p.first_child();
    if (!v) {
        return def;
    }
    return v.attribute("value").as_double(def);
}

int PropInt(const pugi::xml_node& props_node, const char* prop_name, int def = 0)
{
    auto p = FindProperty(props_node, prop_name);
    if (!p) {
        return def;
    }
    auto v = p.first_child();
    if (!v) {
        return def;
    }
    return v.attribute("value").as_int(def);
}

bool PropBool(const pugi::xml_node& props_node, const char* prop_name, bool def = false)
{
    auto p = FindProperty(props_node, prop_name);
    if (!p) {
        return def;
    }
    auto v = p.first_child();
    if (!v) {
        return def;
    }
    return v.attribute("value").as_bool(def);
}

// PropertyLink / PropertyLinkSub - returns the referenced object
// name, plus an optional list of sub element names ("Edge1", "Face3").
struct LinkRef
{
    std::string              object_name;
    std::vector<std::string> sub_names;
};

LinkRef PropLink(const pugi::xml_node& props_node, const char* prop_name)
{
    LinkRef out;
    auto p = FindProperty(props_node, prop_name);
    if (!p) {
        return out;
    }
    // <Property name="Profile" type="App::PropertyLinkSub">
    //   <LinkSub value="Sketch001" count="1">
    //     <Sub value="Face1" />
    //   </LinkSub>
    // </Property>
    auto v = p.first_child();
    if (!v) {
        return out;
    }
    out.object_name = v.attribute("value").value();
    for (auto sub = v.child("Sub"); sub; sub = sub.next_sibling("Sub"))
    {
        out.sub_names.push_back(sub.attribute("value").value());
    }
    return out;
}

// App::PropertyLinkList: ordered list of object refs.
//   <Property name="Group" type="App::PropertyLinkList">
//     <LinkList count="3">
//       <Link value="Origin"/>
//       <Link value="Sketch"/>
//       <Link value="Pad"/>
//     </LinkList>
//   </Property>
//
// Returns an empty vector when the property is absent (which is
// fine for "no body" parts).
std::vector<std::string> PropLinkList(const pugi::xml_node& props_node,
                                      const char*           prop_name)
{
    std::vector<std::string> out;
    auto p = FindProperty(props_node, prop_name);
    if (!p) {
        return out;
    }
    auto list = p.first_child();
    if (!list) {
        return out;
    }
    for (auto ln = list.child("Link"); ln; ln = ln.next_sibling("Link"))
    {
        out.emplace_back(ln.attribute("value").value());
    }
    return out;
}

// FreeCAD object types we ignore entirely. These either describe
// the document tree (App::Part), a coordinate frame (App::Origin
// and its anchor planes / axes / point), or auxiliary datum
// features that don't produce a solid.
bool IsSkipType(const std::string& t)
{
    if (t == "App::Part")                  { return true; }
    if (t == "App::Origin")                { return true; }
    if (t == "App::Plane")                 { return true; }
    if (t == "App::Line")                  { return true; }
    if (t == "App::Point")                 { return true; }
    if (t == "PartDesign::CoordinateSystem"){ return true; }
    if (t == "PartDesign::Plane")          { return true; }
    if (t == "PartDesign::Line")           { return true; }
    if (t == "PartDesign::Point")          { return true; }
    if (t == "PartDesign::ShapeBinder")    { return true; }
    if (t == "PartDesign::SubShapeBinder") { return true; }
    return false;
}

// Container types that have ordered children (Group) but do not
// produce a feature of their own. PartDesign::Body is the main
// example: its Group lists every Sketch / Pad / Pocket / ... in
// modeling order (and Tip points at the active leaf).
bool IsContainerType(const std::string& t)
{
    if (t == "PartDesign::Body") { return true; }
    return false;
}

// Quaternion (qx, qy, qz, qw) -> world X axis and Z axis (sketch
// plane x_dir and normal).
void QuatToAxes(double qx, double qy, double qz, double qw,
                double x_axis[3],
                double z_axis[3])
{
    x_axis[0] = 1.0 - 2.0 * (qy * qy + qz * qz);
    x_axis[1] =       2.0 * (qx * qy + qz * qw);
    x_axis[2] =       2.0 * (qx * qz - qy * qw);

    z_axis[0] =       2.0 * (qx * qz + qy * qw);
    z_axis[1] =       2.0 * (qy * qz - qx * qw);
    z_axis[2] = 1.0 - 2.0 * (qx * qx + qy * qy);
}


// ============================================================
// Section B: .FCStd zip extractor
// ============================================================
//
// .FCStd is a deflate zip with at least one entry "Document.xml".
// We pull that single entry into a heap buffer via miniz.

bool LoadFileBytes(const std::string& path, std::vector<char>& out)
{
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) {
        return false;
    }
    std::fseek(f, 0, SEEK_END);
    long n = std::ftell(f);
    if (n < 0)
    {
        std::fclose(f);
        return false;
    }
    out.resize((size_t)n);
    std::fseek(f, 0, SEEK_SET);
    size_t got = std::fread(out.data(), 1, (size_t)n, f);
    std::fclose(f);
    return got == (size_t)n;
}

} // anonymous namespace


bool FreeCadReader::ExtractDocumentXml(const std::string& path,
                                       char**             out_text,
                                       size_t*            out_size,
                                       std::string*       err_msg)
{
    *out_text = nullptr;
    *out_size = 0;

    mz_zip_archive zip;
    std::memset(&zip, 0, sizeof(zip));

    if (!mz_zip_reader_init_file(&zip, path.c_str(), 0))
    {
        if (err_msg) {
            *err_msg = "miniz: cannot open zip: " + path;
        }
        return false;
    }

    int idx = mz_zip_reader_locate_file(&zip, "Document.xml", nullptr, 0);
    if (idx < 0)
    {
        if (err_msg) {
            *err_msg = "FCStd has no Document.xml inside: " + path;
        }
        mz_zip_reader_end(&zip);
        return false;
    }

    size_t out_sz = 0;
    void*  buf    = mz_zip_reader_extract_to_heap(&zip, idx, &out_sz, 0);
    mz_zip_reader_end(&zip);

    if (!buf)
    {
        if (err_msg) {
            *err_msg = "miniz: extract failed for Document.xml in " + path;
        }
        return false;
    }

    *out_text = (char*)buf;
    *out_size = out_sz;
    return true;
}

void FreeCadReader::FreeXmlBuffer(char* buf)
{
    if (buf) {
        mz_free(buf);
    }
}


// ============================================================
// Section C: sketch geometry / constraint parsers
// ============================================================

namespace
{

// FreeCAD ConstraintType enum to cadcvt::SkConsType. Returned None
// values are dropped at the caller (not all FreeCAD constraints
// have a sketchlib counterpart yet).
SkConsType MapFreeCadConsType(int t)
{
    switch (t)
    {
    case 1:  return SkConsType::Coincident;
    case 2:  return SkConsType::Horizontal;
    case 3:  return SkConsType::Vertical;
    case 4:  return SkConsType::Parallel;
    case 5:  return SkConsType::Tangent;
    case 6:  return SkConsType::Distance;
    case 7:  return SkConsType::DistanceX;
    case 8:  return SkConsType::DistanceY;
    case 9:  return SkConsType::Angle;
    case 10: return SkConsType::Perpendicular;
    case 11: return SkConsType::CircleRadius;     // FreeCAD lumps Arc + Circle radius into Radius
    case 12: return SkConsType::Equal;
    case 13: return SkConsType::PointOnLine;      // best approximation; FreeCAD's PointOnObject is generic
    case 14: return SkConsType::Symmetric;
    case 18: return SkConsType::CircleDiameter;
    // 15 InternalAlignment, 16 SnellsLaw, 17 Block, 19 Weight -> drop
    default: return SkConsType::None;
    }
}

// FreeCAD PointPos enum to cadcvt::SkPointPos.
SkPointPos MapFreeCadPointPos(int p)
{
    switch (p)
    {
    case 0:  return SkPointPos::None;
    case 1:  return SkPointPos::Start;
    case 2:  return SkPointPos::End;
    case 3:  return SkPointPos::Mid;
    default: return SkPointPos::None;
    }
}

// One sketch geometry node -> SkGeoIR. unit_scale is applied to
// every coordinate / radius read from the file.
//
// Returns SkGeoType::None if the geometry type is unsupported. In
// that case the caller still bumps the geometry index counter
// (FreeCAD references geos by index, so we must not skip indices
// silently).
SkGeoIR ParseSketchGeometry(const pugi::xml_node& geo_node,
                            uint32_t              assign_id,
                            double                unit_scale)
{
    const char*    geo_type = AttrStr(geo_node, "type");
    bool           is_constr = false;
    pugi::xml_node constr_node = geo_node.child("Construction");
    if (constr_node) {
        is_constr = AttrBool(constr_node, "value", false);
    }

    SkGeoIR result;
    result.id           = assign_id;
    result.construction = is_constr;

    if (std::strcmp(geo_type, "Part::GeomLineSegment") == 0)
    {
        auto seg = geo_node.child("LineSegment");
        result = SkGeoIR::Line(
            assign_id,
            AttrDouble(seg, "StartX") * unit_scale,
            AttrDouble(seg, "StartY") * unit_scale,
            AttrDouble(seg, "EndX")   * unit_scale,
            AttrDouble(seg, "EndY")   * unit_scale,
            is_constr);
    }
    else if (std::strcmp(geo_type, "Part::GeomCircle") == 0)
    {
        auto c = geo_node.child("Circle");
        result = SkGeoIR::Circle(
            assign_id,
            AttrDouble(c, "CenterX") * unit_scale,
            AttrDouble(c, "CenterY") * unit_scale,
            AttrDouble(c, "Radius")  * unit_scale,
            is_constr);
    }
    else if (std::strcmp(geo_type, "Part::GeomArcOfCircle") == 0)
    {
        auto a = geo_node.child("ArcOfCircle");
        result = SkGeoIR::Arc(
            assign_id,
            AttrDouble(a, "CenterX")    * unit_scale,
            AttrDouble(a, "CenterY")    * unit_scale,
            AttrDouble(a, "Radius")     * unit_scale,
            AttrDouble(a, "StartAngle"),    // radians; not scaled
            AttrDouble(a, "EndAngle"),
            is_constr);
    }
    else if (std::strcmp(geo_type, "Part::GeomEllipse") == 0)
    {
        auto e = geo_node.child("Ellipse");
        result = SkGeoIR::Ellipse(
            assign_id,
            AttrDouble(e, "CenterX")     * unit_scale,
            AttrDouble(e, "CenterY")     * unit_scale,
            AttrDouble(e, "MajorRadius") * unit_scale,
            AttrDouble(e, "MinorRadius") * unit_scale,
            is_constr);
    }
    else if (std::strcmp(geo_type, "Part::GeomPoint") == 0)
    {
        auto p = geo_node.child("Point");
        result = SkGeoIR::Point(
            assign_id,
            AttrDouble(p, "X") * unit_scale,
            AttrDouble(p, "Y") * unit_scale,
            is_constr);
    }
    else if (std::strcmp(geo_type, "Part::GeomBSplineCurve") == 0)
    {
        // BSpline: pole list is the closest we can pass to the
        // store (the solver ignores splines anyway).
        auto bs = geo_node.child("BSplineCurve");
        std::vector<double> xs;
        std::vector<double> ys;
        auto poles = bs.child("Poles");
        for (auto pn = poles.child("Pole"); pn; pn = pn.next_sibling("Pole"))
        {
            xs.push_back(AttrDouble(pn, "X") * unit_scale);
            ys.push_back(AttrDouble(pn, "Y") * unit_scale);
        }
        result = SkGeoIR::Spline(assign_id, xs, ys, is_constr);
    }
    else
    {
        // Unsupported geometry type; leave the SkGeoType::None
        // entry so indices stay aligned.
        result.type = SkGeoType::None;
    }

    return result;
}

// ConstraintList walker. FreeCAD wrote "Constrait" (typo) in older
// versions and "Constraint" in newer; accept both.
void ParseSketchConstraints(const pugi::xml_node&                cons_list_node,
                            const std::unordered_map<int, uint32_t>& geo_idx_to_id,
                            uint32_t&                            next_cons_id,
                            std::vector<SkConsIR>&               out_cons)
{
    auto walk = [&](const char* tag)
    {
        for (auto c = cons_list_node.child(tag); c; c = c.next_sibling(tag))
        {
            int fc_type = AttrInt(c, "Type");
            SkConsType type = MapFreeCadConsType(fc_type);
            if (type == SkConsType::None) {
                continue;
            }

            SkConsIR cons;
            cons.id      = next_cons_id++;
            cons.type    = type;
            cons.value   = AttrDouble(c, "Value");
            cons.driving = AttrBool(c, "IsDriving", true);

            int first       = AttrInt(c, "First", -1);
            int first_pos   = AttrInt(c, "FirstPos", 0);
            int second      = AttrInt(c, "Second", -2000);
            int second_pos  = AttrInt(c, "SecondPos", 0);

            auto resolve = [&](int idx, SkPointPos pos, SkGeoRef& out)
            {
                if (idx >= 0)
                {
                    auto it = geo_idx_to_id.find(idx);
                    if (it != geo_idx_to_id.end()) {
                        out.geo_id = it->second;
                    }
                }
                out.point_pos = pos;
            };

            resolve(first,  MapFreeCadPointPos(first_pos),  cons.a);
            resolve(second, MapFreeCadPointPos(second_pos), cons.b);

            out_cons.push_back(cons);
        }
    };

    walk("Constraint");
    walk("Constrait");   // legacy typo
}

} // anonymous namespace


// ============================================================
// Section D: feature parsers
// ============================================================

namespace
{

// FreeCAD Pad TypeEnums: {Length, UpToLast, UpToFirst, UpToFace, TwoLengths, UpToShape}.
// UpToLast for an additive feature is the additive analog of ThroughAll.
// UpToFirst / UpToShape need a target face; we fall back to ThroughAll
// until we can resolve those targets.
//   0 = Length      (Blind)
//   1 = UpToLast    (ThroughAll)
//   2 = UpToFirst   (no target -> ThroughAll fallback)
//   3 = UpToFace    (UpToSurface)
//   4 = TwoLengths  (Blind on both sides; deprecated upstream)
//   5 = UpToShape   (no target -> ThroughAll fallback)
ExtrudeEndType MapPadEndType(int t, bool midplane)
{
    if (midplane) {
        return ExtrudeEndType::MidPlane;
    }
    switch (t)
    {
    case 0:  return ExtrudeEndType::Blind;
    case 1:  return ExtrudeEndType::ThroughAll;
    case 2:  return ExtrudeEndType::ThroughAll;
    case 3:  return ExtrudeEndType::UpToSurface;
    case 4:  return ExtrudeEndType::Blind;  // TwoLengths: handled via distance2
    case 5:  return ExtrudeEndType::ThroughAll;
    default: return ExtrudeEndType::Blind;
    }
}

// Pocket TypeEnums: {Length, ThroughAll, UpToFirst, UpToFace, TwoLengths, UpToShape}.
// Index 1 differs from Pad (ThroughAll vs UpToLast) but both collapse to our
// ThroughAll, so the table can be shared.
ExtrudeEndType MapPocketEndType(int t, bool midplane)
{
    return MapPadEndType(t, midplane);
}

// Build a stub TopoRefIR for a FreeCAD edge / face name. The
// geo-match attributes are 0 (Replayer will not match via geo);
// the original FreeCAD reference is stored in the parent
// FeatureIR::ext_strings so a FreeCAD-aware resolver can pick
// them up later.
TopoRefIR MakeStubTopoRef(TopoRefIR::Kind kind)
{
    TopoRefIR r;
    r.kind = kind;
    return r;
}

// Stash a list of FreeCAD sub-names (e.g. "Edge1", "Edge2") into
// ext_strings keyed "<prefix>_<i>_name", and emit matching stub
// TopoRefIRs into out_refs.
void StashRefNames(FeatureIR&                       feat,
                   const std::vector<std::string>&  sub_names,
                   const std::string&               object_name,
                   const std::string&               prefix,
                   TopoRefIR::Kind                  kind,
                   std::vector<TopoRefIR>&          out_refs)
{
    for (size_t i = 0; i < sub_names.size(); ++i)
    {
        TopoRefIR r = MakeStubTopoRef(kind);
        out_refs.push_back(r);

        std::string key = prefix + "_" + std::to_string(i) + "_name";
        feat.ext_strings[key] = object_name + "." + sub_names[i];
    }
}

} // anonymous namespace


// ============================================================
// Section E: top-level walk + ReadFile glue
// ============================================================

FreeCadReader::FreeCadReader()  = default;
FreeCadReader::~FreeCadReader() = default;

bool FreeCadReader::ReadFile(const std::string& path,
                             DocumentIR&        out,
                             std::string*       err_msg)
{
    out          = DocumentIR{};
    out.source   = Name();
    out.doc_path = path;

    m_name_to_id.clear();
    m_sk_geo_idx_to_id.clear();
    m_next_feature_id     = 1;
    m_next_sketch_geo_id  = 1;
    m_next_sketch_cons_id = 1;

    // Branch by suffix; .FCStd needs unzip, plain .xml is parsed
    // directly.
    if (EndsWithICase(path, ".fcstd"))
    {
        char*  buf  = nullptr;
        size_t size = 0;
        if (!ExtractDocumentXml(path, &buf, &size, err_msg)) {
            return false;
        }
        bool ok = ParseDocumentXml(buf, size, out, err_msg);
        FreeXmlBuffer(buf);
        return ok;
    }

    if (EndsWithICase(path, ".xml"))
    {
        std::vector<char> bytes;
        if (!LoadFileBytes(path, bytes))
        {
            if (err_msg) {
                *err_msg = "cannot read xml file: " + path;
            }
            return false;
        }
        return ParseDocumentXml(bytes.data(), bytes.size(), out, err_msg);
    }

    if (err_msg) {
        *err_msg = "unrecognized FreeCAD path (.FCStd or .xml expected): " + path;
    }
    return false;
}

bool FreeCadReader::ParseDocumentXml(const char*  xml_data,
                                     size_t       xml_size,
                                     DocumentIR&  out,
                                     std::string* err_msg)
{
    pugi::xml_document doc;
    auto res = doc.load_buffer(xml_data, xml_size);
    if (!res)
    {
        if (err_msg) {
            *err_msg = std::string("pugixml parse error: ") + res.description();
        }
        return false;
    }

    auto root = doc.child("Document");
    if (!root)
    {
        if (err_msg) {
            *err_msg = "missing <Document> root";
        }
        return false;
    }

    // ---- First pass: walk <Objects>, allocate feature ids ----
    //
    // Allocation is up-front (before we know if the object will be
    // emitted) so that BaseFeature / Profile cross-refs always
    // find a valid id later, even for skipped objects (the id
    // simply never appears in out.features).
    struct Pending
    {
        std::string name;
        std::string type;
        uint32_t    feature_id;
    };
    std::vector<Pending>                            queue;
    std::unordered_map<std::string, const Pending*> by_name;

    auto objects_node = root.child("Objects");
    for (auto obj = objects_node.child("Object"); obj; obj = obj.next_sibling("Object"))
    {
        Pending p;
        p.name       = AttrStr(obj, "name");
        p.type       = AttrStr(obj, "type");
        p.feature_id = m_next_feature_id++;
        m_name_to_id[p.name] = p.feature_id;
        queue.push_back(std::move(p));
    }
    for (const auto& p : queue) {
        by_name[p.name] = &p;
    }

    // Index ObjectData by name for cheap lookup.
    auto object_data_node = root.child("ObjectData");
    std::unordered_map<std::string, pugi::xml_node> data_by_name;
    for (auto obj = object_data_node.child("Object"); obj; obj = obj.next_sibling("Object"))
    {
        data_by_name[AttrStr(obj, "name")] = obj;
    }

    // ---- Compute emission order ----
    //
    // PartDesign::Body containers carry a Group property listing
    // their children (sketches + features) in modeling order. When
    // present we honor it; when absent we fall back to declaration
    // order. Multi-body docs concatenate body groups in Body
    // declaration order; standalone objects outside every body
    // follow at the tail (also in declaration order).
    std::vector<const Pending*> emission;
    emission.reserve(queue.size());

    std::unordered_set<std::string> seen;

    auto push_if_unseen = [&](const Pending* p)
    {
        if (!p) {
            return;
        }
        if (seen.count(p->name)) {
            return;
        }
        seen.insert(p->name);
        emission.push_back(p);
    };

    for (const auto& pending : queue)
    {
        if (!IsContainerType(pending.type)) {
            continue;
        }
        auto it = data_by_name.find(pending.name);
        if (it == data_by_name.end()) {
            continue;
        }
        auto props = it->second.child("Properties");
        for (const auto& child_name : PropLinkList(props, "Group"))
        {
            auto cit = by_name.find(child_name);
            if (cit != by_name.end()) {
                push_if_unseen(cit->second);
            }
        }
        // Mark the Body itself as seen so it doesn't get re-queued
        // at the tail; we never emit a feature for the container.
        seen.insert(pending.name);
    }

    for (const auto& pending : queue)
    {
        push_if_unseen(&pending);
    }

    // ---- Second pass: build IR per object in emission order ----
    for (const Pending* pending_ptr : emission)
    {
        const auto& pending = *pending_ptr;

        if (IsSkipType(pending.type) || IsContainerType(pending.type)) {
            continue;
        }

        auto it = data_by_name.find(pending.name);
        if (it == data_by_name.end()) {
            continue;
        }
        auto props = it->second.child("Properties");

        FeatureIR feat;
        feat.id   = pending.feature_id;
        feat.name = pending.name;

        if (pending.type == "Sketcher::SketchObject")
        {
            // ---- Sketch ----
            SketchIR sk;
            sk.feature_id = pending.feature_id;
            sk.name       = pending.name;

            // Plane from Placement: (Px,Py,Pz) is origin, Q* is
            // the rotation quaternion. Default identity gives the
            // XY plane.
            auto place_prop = FindProperty(props, "Placement");
            if (place_prop)
            {
                auto place = place_prop.child("PropertyPlacement");
                sk.plane_origin[0] = AttrDouble(place, "Px") * m_unit_scale;
                sk.plane_origin[1] = AttrDouble(place, "Py") * m_unit_scale;
                sk.plane_origin[2] = AttrDouble(place, "Pz") * m_unit_scale;

                double qx = AttrDouble(place, "Q0", 0.0);
                double qy = AttrDouble(place, "Q1", 0.0);
                double qz = AttrDouble(place, "Q2", 0.0);
                double qw = AttrDouble(place, "Q3", 1.0);
                QuatToAxes(qx, qy, qz, qw, sk.plane_x_dir, sk.plane_normal);
            }

            // Geometry list.
            m_sk_geo_idx_to_id.clear();
            auto geom_prop = FindProperty(props, "Geometry");
            if (geom_prop)
            {
                auto geom_list = geom_prop.child("GeometryList");
                int  fc_idx    = 0;
                for (auto g = geom_list.child("Geometry"); g; g = g.next_sibling("Geometry"))
                {
                    uint32_t gid = m_next_sketch_geo_id++;
                    SkGeoIR  gi  = ParseSketchGeometry(g, gid, m_unit_scale);
                    if (gi.type != SkGeoType::None)
                    {
                        m_sk_geo_idx_to_id[fc_idx] = gid;
                        sk.geos.push_back(std::move(gi));
                    }
                    ++fc_idx;
                }
            }

            // Constraint list.
            auto cons_prop = FindProperty(props, "Constraints");
            if (cons_prop)
            {
                auto cons_list = cons_prop.child("ConstraintList");
                ParseSketchConstraints(
                    cons_list,
                    m_sk_geo_idx_to_id,
                    m_next_sketch_cons_id,
                    sk.cons);
            }

            out.sketches.push_back(std::move(sk));

            // FeatureIR wrapper carrying the sketch id.
            FeatPayloadSketch pl;
            pl.sketch_id = pending.feature_id;
            feat.type    = FeatType::Sketch;
            feat.data    = std::move(pl);
        }
        else if (pending.type == "PartDesign::Pad" ||
                 pending.type == "PartDesign::Pocket")
        {
            FeatPayloadExtrude pl;

            // Profile -> sketch object name -> our feature id.
            LinkRef profile = PropLink(props, "Profile");
            if (!profile.object_name.empty())
            {
                auto sid = m_name_to_id.find(profile.object_name);
                if (sid != m_name_to_id.end()) {
                    pl.sketch_id = sid->second;
                }
            }

            // Direction: Pad/Pocket take the sketch's normal by
            // default; FreeCAD stores a UseCustomVector flag and a
            // Direction property. Default to (0,0,1) and let the
            // Replayer compute the world-space direction from the
            // sketch plane.
            pl.direction[0] = 0.0;
            pl.direction[1] = 0.0;
            pl.direction[2] = 1.0;
            if (PropBool(props, "UseCustomVector", false))
            {
                auto dir_prop = FindProperty(props, "Direction");
                if (dir_prop)
                {
                    auto v = dir_prop.first_child();
                    pl.direction[0] = AttrDouble(v, "x", 0.0);
                    pl.direction[1] = AttrDouble(v, "y", 0.0);
                    pl.direction[2] = AttrDouble(v, "z", 1.0);
                }
            }

            pl.flip_direction = PropBool  (props, "Reversed", false);
            // FreeCAD Pocket defaults to cutting INTO the material
            // (opposite of sketch normal), while Pad extrudes along
            // the normal.  Invert flip so the tool body lands on the
            // correct side of the sketch plane.
            if (pending.type == "PartDesign::Pocket")
                pl.flip_direction = !pl.flip_direction;

            int  type_enum = PropInt (props, "Type", 0);
            bool midplane  = PropBool(props, "Midplane", false);
            pl.end_type    = (pending.type == "PartDesign::Pad")
                               ? MapPadEndType(type_enum, midplane)
                               : MapPocketEndType(type_enum, midplane);

            // Length / Length2 are stored in mm. FreeCAD always
            // writes Length2 (default 100), but it is only used for
            // the legacy TwoLengths mode (Type == 4); pulling it in
            // for ThroughAll / UpToFace / Length-with-Midplane would
            // cause the downstream extruder to spuriously build a
            // second prism in the reverse direction.
            pl.distance = PropDouble(props, "Length", 0.0) * m_unit_scale;
            if (type_enum == 4)
            {
                pl.distance2 = PropDouble(props, "Length2", 0.0) * m_unit_scale;
                pl.end_type2 = ExtrudeEndType::Blind;
            }

            // UpToFace target (only meaningful for Type == 3).
            if (pl.end_type == ExtrudeEndType::UpToSurface)
            {
                LinkRef up = PropLink(props, "UpToFace");
                if (!up.object_name.empty())
                {
                    pl.has_end1_target = true;
                    pl.end1_target     = MakeStubTopoRef(TopoRefIR::Kind::Face);
                    if (!up.sub_names.empty())
                    {
                        feat.ext_strings["end1_target_name"] =
                            up.object_name + "." + up.sub_names[0];
                    }
                }
            }

            feat.type = (pending.type == "PartDesign::Pad")
                          ? FeatType::BossExtrude
                          : FeatType::CutExtrude;
            feat.data = std::move(pl);
        }
        else if (pending.type == "PartDesign::Fillet")
        {
            FeatPayloadFillet pl;
            pl.radius = PropDouble(props, "Radius", 0.0) * m_unit_scale;

            LinkRef base = PropLink(props, "Base");
            StashRefNames(feat, base.sub_names, base.object_name,
                          "edge_ref", TopoRefIR::Kind::Edge, pl.edges);

            feat.type = FeatType::Fillet;
            feat.data = std::move(pl);
        }
        else if (pending.type == "PartDesign::Chamfer")
        {
            FeatPayloadChamfer pl;
            pl.distance1 = PropDouble(props, "Size",  0.0) * m_unit_scale;
            pl.distance2 = PropDouble(props, "Size2", 0.0) * m_unit_scale;
            // ChamferType: 0 = equal, 1 = two distances,
            //              2 = distance + angle. Distance + angle
            // is not yet representable in FeatPayloadChamfer;
            // record it in ext_params so a future field can pick
            // it up.
            int ctype = PropInt(props, "ChamferType", 0);
            if (ctype == 2) {
                feat.ext_params["chamfer_angle"] = PropDouble(props, "Angle", 0.0);
            }

            LinkRef base = PropLink(props, "Base");
            StashRefNames(feat, base.sub_names, base.object_name,
                          "edge_ref", TopoRefIR::Kind::Edge, pl.edges);

            feat.type = FeatType::Chamfer;
            feat.data = std::move(pl);
        }
        else if (pending.type == "PartDesign::Thickness")
        {
            FeatPayloadShell pl;
            pl.thickness     = PropDouble(props, "Value", 0.0) * m_unit_scale;
            pl.shell_outward = PropBool  (props, "Reversed", false);

            LinkRef base = PropLink(props, "Base");
            StashRefNames(feat, base.sub_names, base.object_name,
                          "face_ref", TopoRefIR::Kind::Face, pl.faces_to_open);

            feat.type = FeatType::Shell;
            feat.data = std::move(pl);
        }
        else if (pending.type == "Part::Box")
        {
            FeatPayloadPrimBox pl;
            pl.length = PropDouble(props, "Length", 1.0) * m_unit_scale;
            pl.width  = PropDouble(props, "Width",  1.0) * m_unit_scale;
            pl.height = PropDouble(props, "Height", 1.0) * m_unit_scale;
            feat.type = FeatType::PrimBox;
            feat.data = std::move(pl);
        }
        else if (pending.type == "Part::Cylinder")
        {
            FeatPayloadPrimCylinder pl;
            pl.radius = PropDouble(props, "Radius", 0.5) * m_unit_scale;
            pl.height = PropDouble(props, "Height", 1.0) * m_unit_scale;
            feat.type = FeatType::PrimCylinder;
            feat.data = std::move(pl);
        }
        else if (pending.type == "Part::Sphere")
        {
            FeatPayloadPrimSphere pl;
            pl.radius = PropDouble(props, "Radius", 0.5) * m_unit_scale;
            feat.type = FeatType::PrimSphere;
            feat.data = std::move(pl);
        }
        else if (pending.type == "Part::Cone")
        {
            FeatPayloadPrimCone pl;
            pl.radius1 = PropDouble(props, "Radius1", 0.5) * m_unit_scale;
            pl.radius2 = PropDouble(props, "Radius2", 0.0) * m_unit_scale;
            pl.height  = PropDouble(props, "Height",  1.0) * m_unit_scale;
            feat.type  = FeatType::PrimCone;
            feat.data  = std::move(pl);
        }
        else if (pending.type == "Part::Torus")
        {
            FeatPayloadPrimTorus pl;
            pl.major_radius = PropDouble(props, "Radius1", 1.0)  * m_unit_scale;
            pl.minor_radius = PropDouble(props, "Radius2", 0.25) * m_unit_scale;
            feat.type       = FeatType::PrimTorus;
            feat.data       = std::move(pl);
        }
        else
        {
            // Real unknown feature: container / origin / datum
            // types were already filtered out by IsSkipType /
            // IsContainerType at the top of this loop, so anything
            // reaching here is a feature kind we just don't model
            // yet (e.g. Loft, Revolution, Mirrored, ...). In strict
            // mode this is an error; otherwise we preserve it as
            // Opaque so a later pass can recognise it.
            if (m_strict)
            {
                if (err_msg)
                {
                    std::ostringstream oss;
                    oss << "unknown FreeCAD feature type: "
                        << pending.type
                        << " (object: " << pending.name << ")";
                    *err_msg = oss.str();
                }
                return false;
            }

            FeatPayloadOpaque pl;
            // Preserve the original FreeCAD type tag for a future
            // pass to recognise.
            pl.strings["freecad_type"] = pending.type;
            feat.type = FeatType::Unknown;
            feat.data = std::move(pl);
        }

        out.features.push_back(std::move(feat));
    }

    return true;
}

} // namespace cadcvt
