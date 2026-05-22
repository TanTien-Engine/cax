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

// IR types (DocumentIR / FeatureIR / SkGeoIR / ...) all live in
// cadapp now. Pull them into the cadcvt namespace via a directive so
// the existing unqualified usages keep compiling.
using namespace cadapp;

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

// Map a body-origin sub-name (e.g. "X_Axis", "Y_Axis", "Z_Axis")
// to a unit direction. Returns false when the name is not a
// recognised origin axis -- callers can then fall back to a
// default and stash the original ref in ext_strings.
bool LookupOriginAxisDir(const std::string& sub_name, double out[3])
{
    if (sub_name == "X_Axis") { out[0] = 1.0; out[1] = 0.0; out[2] = 0.0; return true; }
    if (sub_name == "Y_Axis") { out[0] = 0.0; out[1] = 1.0; out[2] = 0.0; return true; }
    if (sub_name == "Z_Axis") { out[0] = 0.0; out[1] = 0.0; out[2] = 1.0; return true; }
    return false;
}

// Map a body-origin plane sub-name (e.g. "XY_Plane") to its
// unit normal vector. Returns false on unknown names.
bool LookupOriginPlaneNormal(const std::string& sub_name, double out[3])
{
    if (sub_name == "XY_Plane") { out[0] = 0.0; out[1] = 0.0; out[2] = 1.0; return true; }
    if (sub_name == "XZ_Plane") { out[0] = 0.0; out[1] = 1.0; out[2] = 0.0; return true; }
    if (sub_name == "YZ_Plane") { out[0] = 1.0; out[1] = 0.0; out[2] = 0.0; return true; }
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


// Resolve a (object_name, sub_name) ref to a world-frame plane
// normal. Recognised forms:
//   - "" / Origin + ("XY_Plane" | "XZ_Plane" | "YZ_Plane")
//     -> body origin plane
//   - <Sketch> + ("V_Axis" | "H_Axis" | "Normal")
//     -> derived from the sketch's Placement quaternion. V_Axis as
//     a mirror "plane" is FreeCAD shorthand for "plane perpendicular
//     to the sketch's H direction, containing V_Axis and the sketch
//     normal"; its world normal is the sketch's H direction. H_Axis
//     is the symmetric case.
bool LookupRefPlaneNormal(
    const std::string& object_name,
    const std::string& sub_name,
    const std::unordered_map<std::string, pugi::xml_node>& data_by_name,
    double out_normal[3])
{
    // Body origin plane (object is "Origin" or omitted).
    if (LookupOriginPlaneNormal(sub_name, out_normal)) {
        return true;
    }

    // Sketch-local axis (V_Axis / H_Axis / Normal). Resolve through
    // the referenced sketch's Placement.
    if (sub_name == "V_Axis" || sub_name == "H_Axis" || sub_name == "Normal")
    {
        auto it = data_by_name.find(object_name);
        if (it == data_by_name.end()) {
            return false;
        }
        auto props = it->second.child("Properties");
        auto place_prop = FindProperty(props, "Placement");
        if (!place_prop) return false;
        auto place = place_prop.child("PropertyPlacement");
        if (!place) return false;

        double qx = AttrDouble(place, "Q0", 0.0);
        double qy = AttrDouble(place, "Q1", 0.0);
        double qz = AttrDouble(place, "Q2", 0.0);
        double qw = AttrDouble(place, "Q3", 1.0);

        double x_axis[3];
        double z_axis[3];
        QuatToAxes(qx, qy, qz, qw, x_axis, z_axis);

        // V_Axis direction in world = z_axis x x_axis (right-handed).
        double y_axis[3] = {
            z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
            z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
            z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0]
        };

        if (sub_name == "V_Axis") {
            // Plane through V_Axis + sketch normal; world normal = H.
            out_normal[0] = x_axis[0];
            out_normal[1] = x_axis[1];
            out_normal[2] = x_axis[2];
        } else if (sub_name == "H_Axis") {
            // Plane through H_Axis + sketch normal; world normal = V.
            out_normal[0] = y_axis[0];
            out_normal[1] = y_axis[1];
            out_normal[2] = y_axis[2];
        } else {
            // "Normal" -> mirror across the sketch plane itself.
            out_normal[0] = z_axis[0];
            out_normal[1] = z_axis[1];
            out_normal[2] = z_axis[2];
        }
        return true;
    }

    return false;
}

// Resolve a PartDesign::Line (datum line) reference to its world-
// frame origin and direction. The line's "down the line" direction
// is the local +Z, rotated by the Placement quaternion; the line's
// origin is Placement.Px/Py/Pz (in FreeCAD's mm units, so the
// caller scales). Used by Polar/Linear pattern readers when the
// Axis link points at a datum line instead of an Origin axis --
// previously only the direction defaulted (correct by accident for
// identity-quat datums) but axis_origin stayed at (0,0,0) which
// shoved every patterned cylinder onto the wrong circle.
bool LookupDatumLineAxis(
    const std::string& object_name,
    const std::unordered_map<std::string, pugi::xml_node>& data_by_name,
    double             unit_scale,
    double             out_origin[3],
    double             out_dir[3])
{
    auto it = data_by_name.find(object_name);
    if (it == data_by_name.end()) return false;
    auto props = it->second.child("Properties");
    auto place_prop = FindProperty(props, "Placement");
    if (!place_prop) return false;
    auto place = place_prop.child("PropertyPlacement");
    if (!place) return false;

    out_origin[0] = AttrDouble(place, "Px", 0.0) * unit_scale;
    out_origin[1] = AttrDouble(place, "Py", 0.0) * unit_scale;
    out_origin[2] = AttrDouble(place, "Pz", 0.0) * unit_scale;

    double qx = AttrDouble(place, "Q0", 0.0);
    double qy = AttrDouble(place, "Q1", 0.0);
    double qz = AttrDouble(place, "Q2", 0.0);
    double qw = AttrDouble(place, "Q3", 1.0);
    double x_axis[3], z_axis[3];
    QuatToAxes(qx, qy, qz, qw, x_axis, z_axis);
    out_dir[0] = z_axis[0];
    out_dir[1] = z_axis[1];
    out_dir[2] = z_axis[2];
    return true;
}

// Resolve a (object_name, sub_name) ref to BOTH a world-frame axis
// origin and direction. Recognises:
//   - "Origin" / "" + X_Axis|Y_Axis|Z_Axis -> body origin (0,0,0) + axis
//   - <Sketch> + V_Axis|H_Axis|Normal      -> sketch placement origin + axis
// Returns false on unknown refs (caller falls back / leaves defaults).
//
// Used by Groove/Revolution which take a ReferenceAxis link that
// must produce a full gp_Ax1 (both location and direction). The
// pre-existing LookupRefAxisDir only returns direction, which is fine
// for patterns that rotate about an origin set elsewhere but loses
// the sketch placement origin needed by revolve.
bool LookupRefAxisOriginDir(
    const std::string& object_name,
    const std::string& sub_name,
    const std::unordered_map<std::string, pugi::xml_node>& data_by_name,
    double             unit_scale,
    double             out_origin[3],
    double             out_dir[3])
{
    out_origin[0] = 0.0;
    out_origin[1] = 0.0;
    out_origin[2] = 0.0;

    if (LookupOriginAxisDir(sub_name, out_dir)) {
        return true;
    }
    if (sub_name == "V_Axis" || sub_name == "H_Axis" || sub_name == "Normal")
    {
        auto it = data_by_name.find(object_name);
        if (it == data_by_name.end()) return false;
        auto props = it->second.child("Properties");
        auto place_prop = FindProperty(props, "Placement");
        if (!place_prop) return false;
        auto place = place_prop.child("PropertyPlacement");
        if (!place) return false;

        out_origin[0] = AttrDouble(place, "Px", 0.0) * unit_scale;
        out_origin[1] = AttrDouble(place, "Py", 0.0) * unit_scale;
        out_origin[2] = AttrDouble(place, "Pz", 0.0) * unit_scale;

        double qx = AttrDouble(place, "Q0", 0.0);
        double qy = AttrDouble(place, "Q1", 0.0);
        double qz = AttrDouble(place, "Q2", 0.0);
        double qw = AttrDouble(place, "Q3", 1.0);

        double x_axis[3], z_axis[3];
        QuatToAxes(qx, qy, qz, qw, x_axis, z_axis);
        double y_axis[3] = {
            z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
            z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
            z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0]
        };

        if (sub_name == "H_Axis") {
            out_dir[0] = x_axis[0]; out_dir[1] = x_axis[1]; out_dir[2] = x_axis[2];
        } else if (sub_name == "V_Axis") {
            out_dir[0] = y_axis[0]; out_dir[1] = y_axis[1]; out_dir[2] = y_axis[2];
        } else {
            out_dir[0] = z_axis[0]; out_dir[1] = z_axis[1]; out_dir[2] = z_axis[2];
        }
        return true;
    }
    return false;
}

// Resolve a (object_name, sub_name) ref to a world-frame axis
// direction. Recognises body-origin axes and sketch axes.
bool LookupRefAxisDir(
    const std::string& object_name,
    const std::string& sub_name,
    const std::unordered_map<std::string, pugi::xml_node>& data_by_name,
    double out_dir[3])
{
    if (LookupOriginAxisDir(sub_name, out_dir)) {
        return true;
    }
    if (sub_name == "V_Axis" || sub_name == "H_Axis" || sub_name == "Normal")
    {
        auto it = data_by_name.find(object_name);
        if (it == data_by_name.end()) return false;
        auto props = it->second.child("Properties");
        auto place_prop = FindProperty(props, "Placement");
        if (!place_prop) return false;
        auto place = place_prop.child("PropertyPlacement");
        if (!place) return false;

        double qx = AttrDouble(place, "Q0", 0.0);
        double qy = AttrDouble(place, "Q1", 0.0);
        double qz = AttrDouble(place, "Q2", 0.0);
        double qw = AttrDouble(place, "Q3", 1.0);

        double x_axis[3];
        double z_axis[3];
        QuatToAxes(qx, qy, qz, qw, x_axis, z_axis);
        double y_axis[3] = {
            z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
            z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
            z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0]
        };

        if (sub_name == "H_Axis") {
            out_dir[0] = x_axis[0]; out_dir[1] = x_axis[1]; out_dir[2] = x_axis[2];
        } else if (sub_name == "V_Axis") {
            out_dir[0] = y_axis[0]; out_dir[1] = y_axis[1]; out_dir[2] = y_axis[2];
        } else {
            out_dir[0] = z_axis[0]; out_dir[1] = z_axis[1]; out_dir[2] = z_axis[2];
        }
        return true;
    }
    return false;
}

// Translate a FreeCAD Transformed child (Mirrored / LinearPattern /
// PolarPattern) into one MultiTransformStep. Returns false when the
// type is not a recognised transformation kind. Used by the
// PartDesign::MultiTransform path which carries an ordered list of
// such child features in its Transformations property.
bool ReadTransformedStep(
    const std::string&    child_type,
    const pugi::xml_node& props,
    double                unit_scale,
    const std::unordered_map<std::string, pugi::xml_node>& data_by_name,
    MultiTransformStep&   out)
{
    if (child_type == "PartDesign::Mirrored")
    {
        out.kind = MultiTransformStep::Kind::Mirror;
        out.plane_origin[0] = 0.0;
        out.plane_origin[1] = 0.0;
        out.plane_origin[2] = 0.0;
        out.plane_normal[0] = 1.0;
        out.plane_normal[1] = 0.0;
        out.plane_normal[2] = 0.0;

        LinkRef mp = PropLink(props, "MirrorPlane");
        if (!mp.sub_names.empty())
        {
            double n[3];
            if (LookupRefPlaneNormal(mp.object_name, mp.sub_names[0],
                                     data_by_name, n))
            {
                out.plane_normal[0] = n[0];
                out.plane_normal[1] = n[1];
                out.plane_normal[2] = n[2];
            }
        }
        return true;
    }

    if (child_type == "PartDesign::LinearPattern")
    {
        out.kind    = MultiTransformStep::Kind::LinearPattern;
        out.dir1[0] = 1.0;
        out.dir1[1] = 0.0;
        out.dir1[2] = 0.0;
        out.dir2[0] = 0.0;
        out.dir2[1] = 1.0;
        out.dir2[2] = 0.0;
        out.count2  = 1;
        out.spacing2 = 0.0;

        LinkRef dir = PropLink(props, "Direction");
        if (!dir.sub_names.empty())
        {
            double d[3];
            if (LookupRefAxisDir(dir.object_name, dir.sub_names[0],
                                 data_by_name, d))
            {
                out.dir1[0] = d[0];
                out.dir1[1] = d[1];
                out.dir1[2] = d[2];
            }
        }

        int    count  = PropInt   (props, "Occurrences", 2);
        double length = PropDouble(props, "Length",      0.0) * unit_scale;
        bool   rev    = PropBool  (props, "Reversed",    false);

        out.count1   = (count >= 1) ? count : 2;
        out.spacing1 = (out.count1 > 1)
                        ? (length / (double)(out.count1 - 1))
                        : 0.0;
        if (rev)
        {
            out.dir1[0] = -out.dir1[0];
            out.dir1[1] = -out.dir1[1];
            out.dir1[2] = -out.dir1[2];
        }
        return true;
    }

    if (child_type == "PartDesign::PolarPattern")
    {
        out.kind = MultiTransformStep::Kind::CircularPattern;
        out.axis_origin[0] = 0.0;
        out.axis_origin[1] = 0.0;
        out.axis_origin[2] = 0.0;
        out.axis_dir[0]    = 0.0;
        out.axis_dir[1]    = 0.0;
        out.axis_dir[2]    = 1.0;

        LinkRef axis = PropLink(props, "Axis");
        if (!axis.sub_names.empty())
        {
            double d[3];
            if (LookupRefAxisDir(axis.object_name, axis.sub_names[0],
                                 data_by_name, d))
            {
                out.axis_dir[0] = d[0];
                out.axis_dir[1] = d[1];
                out.axis_dir[2] = d[2];
            }
        }
        else if (!axis.object_name.empty())
        {
            // Axis link with empty sub: object_name is the axis itself,
            // e.g. <LinkSub value="Z_Axis"><Sub value=""/></LinkSub>.
            double d[3];
            if (LookupOriginAxisDir(axis.object_name, d))
            {
                out.axis_dir[0] = d[0];
                out.axis_dir[1] = d[1];
                out.axis_dir[2] = d[2];
            }
        }

        int    count   = PropInt   (props, "Occurrences", 2);
        double angDeg  = PropDouble(props, "Angle",     360.0);
        bool   rev     = PropBool  (props, "Reversed",  false);

        out.count        = (count >= 1) ? count : 2;
        out.total_angle  = angDeg * 3.14159265358979323846 / 180.0;
        if (rev) {
            out.total_angle = -out.total_angle;
        }
        return true;
    }

    return false;
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

// Read FreeCAD's PropertyPlacement (translation + axis-angle rotation)
// and stash it on the FeatureIR via ext_params. The Replayer applies
// it as rotate-then-translate when emitting a primitive. Keys:
//   placement_px / py / pz  : translation
//   placement_ox / oy / oz  : rotation axis (unit vector)
//   placement_angle         : rotation angle in radians
// All keys are absent when the placement is identity.
void StashPlacement(FeatureIR&            feat,
                    const pugi::xml_node& props_node,
                    double                unit_scale)
{
    auto p = FindProperty(props_node, "Placement");
    if (!p) {
        return;
    }
    auto v = p.child("PropertyPlacement");
    if (!v) {
        return;
    }
    double px = AttrDouble(v, "Px", 0.0) * unit_scale;
    double py = AttrDouble(v, "Py", 0.0) * unit_scale;
    double pz = AttrDouble(v, "Pz", 0.0) * unit_scale;
    double a  = AttrDouble(v, "A",  0.0);
    double ox = AttrDouble(v, "Ox", 0.0);
    double oy = AttrDouble(v, "Oy", 0.0);
    double oz = AttrDouble(v, "Oz", 1.0);

    bool has_t = (px != 0.0) || (py != 0.0) || (pz != 0.0);
    bool has_r = (a  != 0.0);
    if (!has_t && !has_r) {
        return;
    }
    if (has_t)
    {
        feat.ext_params["placement_px"] = px;
        feat.ext_params["placement_py"] = py;
        feat.ext_params["placement_pz"] = pz;
    }
    if (has_r)
    {
        feat.ext_params["placement_angle"] = a;
        feat.ext_params["placement_ox"]    = ox;
        feat.ext_params["placement_oy"]    = oy;
        feat.ext_params["placement_oz"]    = oz;
    }
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

// FreeCAD ConstraintType enum to cadapp::SkConsType. Returned None
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

// Whether a constraint's `Value` field carries a length (and must
// therefore be multiplied by m_unit_scale on the way out). Angles
// stay in radians; positional / topological constraints have no
// numeric value (FreeCAD writes 0 / ignored).
bool IsLengthConstraint(SkConsType t)
{
    switch (t)
    {
    case SkConsType::Distance:
    case SkConsType::DistanceX:
    case SkConsType::DistanceY:
    case SkConsType::CircleRadius:
    case SkConsType::CircleDiameter:
    case SkConsType::ArcRadius:
    case SkConsType::ArcDiameter:
        return true;
    default:
        return false;
    }
}

// FreeCAD PointPos enum to cadapp::SkPointPos.
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
//
// unit_scale converts FreeCAD's internal millimetres to project
// metres for length-bearing constraints (Distance / Radius / ...);
// angles and topological constraints pass through unchanged. The
// sketch geometry was already scaled in ParseSketchGeometry, so
// without this the geometry and its constraints would disagree by
// 1000x and the solver would warp the sketch to satisfy it.
void ParseSketchConstraints(const pugi::xml_node&                cons_list_node,
                            const std::unordered_map<int, uint32_t>& geo_idx_to_id,
                            uint32_t&                            next_cons_id,
                            std::vector<SkConsIR>&               out_cons,
                            double                               unit_scale)
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
            if (IsLengthConstraint(type)) {
                cons.value *= unit_scale;
            }
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
//
// Midplane is NOT a Type value -- it's a separate boolean that
// shifts the prism start by -L/2 so the extrusion straddles the
// sketch plane. When Type == ThroughAll/UpToLast/UpToFirst/UpToShape
// FreeCAD still computes a "through all" L and applies the midplane
// shift, giving a symmetric cut/pad that goes through material on
// both sides. We previously short-circuited to MidPlane on midplane
// regardless of Type, so a Type=ThroughAll + Midplane=true pocket
// (Page_015_Exercise2D-07: 5mm midplane "through all" in a 10mm
// midplane pad) became a 5mm symmetric slot stuck inside the pad
// with 2.5mm caps on each side, and the holes never punched through.
// Now ThroughAll wins; the caller mirrors the second side via
// end_type2 so the engine builds both halves.
ExtrudeEndType MapPadEndType(int t, bool midplane)
{
    switch (t)
    {
    case 0:  return midplane ? ExtrudeEndType::MidPlane : ExtrudeEndType::Blind;
    case 1:  return ExtrudeEndType::ThroughAll;
    case 2:  return ExtrudeEndType::ThroughAll;
    case 3:  return ExtrudeEndType::UpToSurface;
    case 4:  return midplane ? ExtrudeEndType::MidPlane : ExtrudeEndType::Blind;  // TwoLengths: handled via distance2
    case 5:  return ExtrudeEndType::ThroughAll;
    default: return midplane ? ExtrudeEndType::MidPlane : ExtrudeEndType::Blind;
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

// Map a FreeCAD sub-name like "Edge1" / "Face3" / "Vertex2" to the
// matching TopoRefIR::Kind. Fillet/Chamfer accept either edges (the
// edge gets rounded directly) or faces (every edge of the face gets
// rounded). Treating a Face<N> ref as an edge made the resolver miss
// it entirely and the engine fall through to "fillet all edges",
// which crashes on non-trivial bodies.
TopoRefIR::Kind ClassifySubName(const std::string& sub, TopoRefIR::Kind fallback)
{
    if (sub.rfind("Edge",   0) == 0) return TopoRefIR::Kind::Edge;
    if (sub.rfind("Face",   0) == 0) return TopoRefIR::Kind::Face;
    if (sub.rfind("Vertex", 0) == 0) return TopoRefIR::Kind::Vertex;
    return fallback;
}

// Stash a list of FreeCAD sub-names (e.g. "Edge1", "Edge2") into
// ext_strings keyed "<prefix>_<i>_name", and emit matching stub
// TopoRefIRs into out_refs. The kind of each ref is inferred from
// the sub-name prefix; `fallback_kind` only applies to unknown
// shapes (e.g. "Anything1").
void StashRefNames(FeatureIR&                       feat,
                   const std::vector<std::string>&  sub_names,
                   const std::string&               object_name,
                   const std::string&               prefix,
                   TopoRefIR::Kind                  fallback_kind,
                   std::vector<TopoRefIR>&          out_refs)
{
    for (size_t i = 0; i < sub_names.size(); ++i)
    {
        TopoRefIR r = MakeStubTopoRef(
            ClassifySubName(sub_names[i], fallback_kind));
        out_refs.push_back(r);

        std::string key = prefix + "_" + std::to_string(i) + "_name";
        feat.ext_strings[key] = object_name + "." + sub_names[i];
    }
}

// Record an "Originals" link list (the features a Transformed
// feature operates on) into FeatureIR ext_params:
//   originals_count            -> N
//   originals_id_<i>           -> feature_id of i-th original
//   originals_name_<i>         -> name (ext_strings, for diagnostics)
// The Replayer reads these to apply the pattern to each Original's
// tool shape rather than to the whole running body.
void StashOriginals(FeatureIR&                                          feat,
                    const std::vector<std::string>&                     originals,
                    const std::unordered_map<std::string, uint32_t>&    name_to_id)
{
    if (originals.empty()) return;
    feat.ext_params["originals_count"] = (double)originals.size();
    for (size_t i = 0; i < originals.size(); ++i)
    {
        auto it = name_to_id.find(originals[i]);
        if (it != name_to_id.end())
        {
            feat.ext_params["originals_id_" + std::to_string(i)] =
                (double)it->second;
        }
        feat.ext_strings["originals_name_" + std::to_string(i)] = originals[i];
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

    // ---- Collect MultiTransform children ----
    //
    // PartDesign::MultiTransform owns an ordered list of Mirrored /
    // LinearPattern / PolarPattern child objects via its
    // Transformations property. Those children are NOT standalone
    // body features -- the MultiTransform reads their params and
    // chains them itself. Track their names so the emission walk
    // can skip them and the MultiTransform handler can find their
    // properties.
    std::unordered_set<std::string> mt_children;
    for (const auto& pending : queue)
    {
        if (pending.type != "PartDesign::MultiTransform") {
            continue;
        }
        auto it = data_by_name.find(pending.name);
        if (it == data_by_name.end()) {
            continue;
        }
        auto props = it->second.child("Properties");
        for (const auto& child_name : PropLinkList(props, "Transformations"))
        {
            mt_children.insert(child_name);
        }
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

    // Children owned by a MultiTransform must never be emitted
    // standalone; the MultiTransform itself encodes their effect.
    for (const auto& n : mt_children) {
        seen.insert(n);
    }

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
                    sk.cons,
                    m_unit_scale);
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
            // ThroughAll + Midplane: FreeCAD cuts through material on
            // BOTH sides of the sketch plane (a "symmetric through
            // all"). Our engine's ThroughAll only sweeps one side,
            // so mirror the second direction explicitly via end_type2.
            // Without this, midplane through-all pockets only punch
            // half-way, and patterned versions of them (see
            // Page_015_Exercise2D-07's PolarPattern of the central
            // hole) leave the part visually un-drilled.
            else if (midplane && pl.end_type == ExtrudeEndType::ThroughAll)
            {
                pl.end_type2 = ExtrudeEndType::ThroughAll;
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
        else if (pending.type == "PartDesign::Revolution" ||
                 pending.type == "PartDesign::Groove")
        {
            // Boss-Revolve (Revolution, additive) and Cut-Revolve
            // (Groove, subtractive). FreeCAD properties:
            //   Profile        -> LinkSub(sketch object)
            //   ReferenceAxis  -> LinkSub(axis object + sub-name)
            //   Angle          -> degrees
            //   Reversed       -> flip sweep direction (negate angle)
            //   Midplane       -> symmetric sweep (TODO: not yet honored
            //                     by the revolve op; one-sided for now,
            //                     stashed in ext_params for a future pass)
            //   Base           -> additional Vector offset added to the
            //                     resolved axis origin (rarely non-zero
            //                     in PartDesign; FreeCAD almost always
            //                     leaves it (0,0,0) and lets the
            //                     ReferenceAxis carry the location)
            FeatPayloadRevolve pl;

            LinkRef profile = PropLink(props, "Profile");
            if (!profile.object_name.empty())
            {
                auto sid = m_name_to_id.find(profile.object_name);
                if (sid != m_name_to_id.end()) {
                    pl.sketch_id = sid->second;
                }
            }

            // Defaults: rotate about world Z through the body origin.
            pl.axis_origin[0] = 0.0;
            pl.axis_origin[1] = 0.0;
            pl.axis_origin[2] = 0.0;
            pl.axis_dir[0]    = 0.0;
            pl.axis_dir[1]    = 0.0;
            pl.axis_dir[2]    = 1.0;

            LinkRef axis = PropLink(props, "ReferenceAxis");
            std::string sub0 = axis.sub_names.empty() ? std::string()
                                                       : axis.sub_names[0];
            bool axis_resolved = false;
            double ao[3], ad[3];
            if (!sub0.empty()
                && LookupRefAxisOriginDir(axis.object_name, sub0,
                                          data_by_name, m_unit_scale,
                                          ao, ad))
            {
                pl.axis_origin[0] = ao[0];
                pl.axis_origin[1] = ao[1];
                pl.axis_origin[2] = ao[2];
                pl.axis_dir[0]    = ad[0];
                pl.axis_dir[1]    = ad[1];
                pl.axis_dir[2]    = ad[2];
                axis_resolved = true;
            }
            else if (!axis.object_name.empty()
                     && LookupOriginAxisDir(axis.object_name, ad))
            {
                pl.axis_dir[0] = ad[0];
                pl.axis_dir[1] = ad[1];
                pl.axis_dir[2] = ad[2];
                axis_resolved = true;
            }
            if (!axis_resolved && !axis.object_name.empty())
            {
                // Fallback: ReferenceAxis pointing at a PartDesign::Line
                // datum -- its Placement carries both origin and dir.
                if (LookupDatumLineAxis(axis.object_name, data_by_name,
                                        m_unit_scale, ao, ad))
                {
                    pl.axis_origin[0] = ao[0];
                    pl.axis_origin[1] = ao[1];
                    pl.axis_origin[2] = ao[2];
                    pl.axis_dir[0]    = ad[0];
                    pl.axis_dir[1]    = ad[1];
                    pl.axis_dir[2]    = ad[2];
                }
            }
            if (!axis.object_name.empty())
            {
                feat.ext_strings["revolve_axis_ref"] = sub0.empty()
                    ? axis.object_name
                    : axis.object_name + "." + sub0;
            }

            // Base offset (PropertyVector). Almost always (0,0,0) but
            // FreeCAD will write a non-zero Base when the user types it
            // by hand instead of picking a ReferenceAxis.
            auto base_prop = FindProperty(props, "Base");
            if (base_prop)
            {
                auto pv = base_prop.child("PropertyVector");
                if (pv)
                {
                    pl.axis_origin[0] += AttrDouble(pv, "valueX", 0.0) * m_unit_scale;
                    pl.axis_origin[1] += AttrDouble(pv, "valueY", 0.0) * m_unit_scale;
                    pl.axis_origin[2] += AttrDouble(pv, "valueZ", 0.0) * m_unit_scale;
                }
            }

            double ang_deg = PropDouble(props, "Angle", 360.0);
            bool   reversed = PropBool(props, "Reversed", false);
            pl.angle = ang_deg * 3.14159265358979323846 / 180.0;
            pl.flip_direction = reversed;

            bool midplane = PropBool(props, "Midplane", false);
            if (midplane) {
                // Preserve the intent for a future Replayer pass; the
                // current revolve op is one-sided so the geometry will
                // be off-centered by half the angle until midplane is
                // wired through.
                feat.ext_params["midplane"] = 1.0;
            }

            feat.type = (pending.type == "PartDesign::Revolution")
                          ? FeatType::BossRevolve
                          : FeatType::CutRevolve;
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
        else if (pending.type == "PartDesign::AdditiveBox" ||
                 pending.type == "PartDesign::SubtractiveBox")
        {
            // PartDesign additive/subtractive box: same params as
            // Part::Box. The additive/subtractive flavour and the
            // primitive's Placement (origin + axis-angle rotation)
            // are forwarded to the Replayer via ext_strings /
            // ext_params, which then fuses or cuts against the
            // running body shape.
            FeatPayloadPrimBox pl;
            pl.length = PropDouble(props, "Length", 1.0) * m_unit_scale;
            pl.width  = PropDouble(props, "Width",  1.0) * m_unit_scale;
            pl.height = PropDouble(props, "Height", 1.0) * m_unit_scale;
            feat.type = FeatType::PrimBox;
            feat.data = std::move(pl);
            feat.ext_strings["freecad_type"] = pending.type;
            StashPlacement(feat, props, m_unit_scale);
        }
        else if (pending.type == "PartDesign::AdditiveCylinder" ||
                 pending.type == "PartDesign::SubtractiveCylinder")
        {
            FeatPayloadPrimCylinder pl;
            pl.radius = PropDouble(props, "Radius", 0.5) * m_unit_scale;
            pl.height = PropDouble(props, "Height", 1.0) * m_unit_scale;
            feat.type = FeatType::PrimCylinder;
            feat.data = std::move(pl);
            feat.ext_strings["freecad_type"] = pending.type;
            StashPlacement(feat, props, m_unit_scale);
        }
        else if (pending.type == "PartDesign::AdditiveSphere" ||
                 pending.type == "PartDesign::SubtractiveSphere")
        {
            FeatPayloadPrimSphere pl;
            pl.radius = PropDouble(props, "Radius", 0.5) * m_unit_scale;
            feat.type = FeatType::PrimSphere;
            feat.data = std::move(pl);
            feat.ext_strings["freecad_type"] = pending.type;
            StashPlacement(feat, props, m_unit_scale);
        }
        else if (pending.type == "PartDesign::AdditiveCone" ||
                 pending.type == "PartDesign::SubtractiveCone")
        {
            FeatPayloadPrimCone pl;
            pl.radius1 = PropDouble(props, "Radius1", 0.5) * m_unit_scale;
            pl.radius2 = PropDouble(props, "Radius2", 0.0) * m_unit_scale;
            pl.height  = PropDouble(props, "Height",  1.0) * m_unit_scale;
            feat.type  = FeatType::PrimCone;
            feat.data  = std::move(pl);
            feat.ext_strings["freecad_type"] = pending.type;
            StashPlacement(feat, props, m_unit_scale);
        }
        else if (pending.type == "PartDesign::AdditiveTorus" ||
                 pending.type == "PartDesign::SubtractiveTorus")
        {
            FeatPayloadPrimTorus pl;
            pl.major_radius = PropDouble(props, "Radius1", 1.0)  * m_unit_scale;
            pl.minor_radius = PropDouble(props, "Radius2", 0.25) * m_unit_scale;
            feat.type       = FeatType::PrimTorus;
            feat.data       = std::move(pl);
            feat.ext_strings["freecad_type"] = pending.type;
            StashPlacement(feat, props, m_unit_scale);
        }
        else if (pending.type == "PartDesign::Mirrored")
        {
            // FreeCAD mirror feature. MirrorPlane is a LinkSub
            // pointing at a body-origin plane (XY_Plane / XZ_Plane /
            // YZ_Plane), a sketch axis (V_Axis / H_Axis / Normal),
            // or a face on the base feature. Recognised refs resolve
            // to a world-frame plane normal; unknown ones fall back
            // to the YZ plane and the original ref is preserved in
            // ext_strings.
            FeatPayloadMirror pl;
            // sensible default: mirror across the YZ plane.
            pl.plane_origin[0] = 0.0;
            pl.plane_origin[1] = 0.0;
            pl.plane_origin[2] = 0.0;
            pl.plane_normal[0] = 1.0;
            pl.plane_normal[1] = 0.0;
            pl.plane_normal[2] = 0.0;

            LinkRef mp = PropLink(props, "MirrorPlane");
            if (!mp.sub_names.empty())
            {
                double n[3];
                if (LookupRefPlaneNormal(mp.object_name, mp.sub_names[0],
                                         data_by_name, n))
                {
                    pl.plane_normal[0] = n[0];
                    pl.plane_normal[1] = n[1];
                    pl.plane_normal[2] = n[2];
                }
                feat.ext_strings["mirror_plane_ref"] =
                    mp.object_name + "." + mp.sub_names[0];
            }
            else if (!mp.object_name.empty())
            {
                feat.ext_strings["mirror_plane_ref"] = mp.object_name;
            }

            // Originals: list of features to mirror. Stored as
            // ext_params so the Replayer can resolve them by id.
            StashOriginals(feat, PropLinkList(props, "Originals"), m_name_to_id);

            feat.type = FeatType::Mirror;
            feat.data = std::move(pl);
        }
        else if (pending.type == "PartDesign::LinearPattern")
        {
            // FreeCAD linear pattern. Direction is a LinkSub to an
            // origin axis or sketch axis; Length is total length;
            // Occurrences is the count (including original).
            FeatPayloadLinearPattern pl;
            pl.dir1[0] = 1.0;
            pl.dir1[1] = 0.0;
            pl.dir1[2] = 0.0;
            pl.dir2[0] = 0.0;
            pl.dir2[1] = 1.0;
            pl.dir2[2] = 0.0;
            pl.count2  = 1;       // FreeCAD's LinearPattern is 1D.
            pl.spacing2 = 0.0;

            LinkRef dir = PropLink(props, "Direction");
            if (!dir.sub_names.empty())
            {
                double d[3];
                if (LookupRefAxisDir(dir.object_name, dir.sub_names[0],
                                     data_by_name, d))
                {
                    pl.dir1[0] = d[0];
                    pl.dir1[1] = d[1];
                    pl.dir1[2] = d[2];
                }
                feat.ext_strings["pattern_dir_ref"] =
                    dir.object_name + "." + dir.sub_names[0];
            }
            else if (!dir.object_name.empty())
            {
                double d[3];
                if (LookupOriginAxisDir(dir.object_name, d))
                {
                    pl.dir1[0] = d[0];
                    pl.dir1[1] = d[1];
                    pl.dir1[2] = d[2];
                }
                feat.ext_strings["pattern_dir_ref"] = dir.object_name;
            }

            int    count  = PropInt   (props, "Occurrences", 2);
            double length = PropDouble(props, "Length",      0.0) * m_unit_scale;
            bool   rev    = PropBool  (props, "Reversed",    false);

            pl.count1   = (count >= 1) ? count : 2;
            // FreeCAD stores total length; we store per-step spacing.
            // count >= 2 so denominator is safe.
            pl.spacing1 = (pl.count1 > 1)
                            ? (length / (double)(pl.count1 - 1))
                            : 0.0;
            if (rev) {
                pl.dir1[0] = -pl.dir1[0];
                pl.dir1[1] = -pl.dir1[1];
                pl.dir1[2] = -pl.dir1[2];
            }

            StashOriginals(feat, PropLinkList(props, "Originals"), m_name_to_id);

            feat.type = FeatType::LinearPattern;
            feat.data = std::move(pl);
        }
        else if (pending.type == "PartDesign::PolarPattern")
        {
            // FreeCAD polar pattern. Axis is a LinkSub to an origin
            // axis, sketch axis, or edge; Angle is total angle in
            // degrees; Occurrences is the count (including original).
            FeatPayloadCircularPattern pl;
            pl.axis_origin[0] = 0.0;
            pl.axis_origin[1] = 0.0;
            pl.axis_origin[2] = 0.0;
            pl.axis_dir[0]    = 0.0;
            pl.axis_dir[1]    = 0.0;
            pl.axis_dir[2]    = 1.0;

            LinkRef axis = PropLink(props, "Axis");
            // PolarPattern axis comes in three flavours, all of which
            // we used to half-resolve:
            //   (1) Origin sub-axis: <LinkSub value="Origin"><Sub
            //       value="X_Axis"/></LinkSub> -- handled by
            //       LookupRefAxisDir; origin is body origin (0,0,0).
            //   (2) Bare X_Axis/Y_Axis/Z_Axis name with empty Sub --
            //       same as (1), origin still (0,0,0).
            //   (3) PartDesign::Line datum -- the line's Placement
            //       carries BOTH a non-zero origin and a possibly
            //       non-Z direction. Previously we fell through
            //       without reading either, so axis_origin stayed
            //       (0,0,0) and the patterned cylinders rotated
            //       around the wrong column.
            // sub_names is [""] for the empty-Sub case (PropLink keeps
            // empty Sub elements), so case (1) is "sub_names[0] is a
            // known axis token".
            std::string sub0 = axis.sub_names.empty() ? std::string()
                                                       : axis.sub_names[0];
            bool dir_set = false;
            double d[3];
            if (!sub0.empty()
                && LookupRefAxisDir(axis.object_name, sub0, data_by_name, d))
            {
                pl.axis_dir[0] = d[0];
                pl.axis_dir[1] = d[1];
                pl.axis_dir[2] = d[2];
                dir_set = true;
            }
            else if (!axis.object_name.empty()
                     && LookupOriginAxisDir(axis.object_name, d))
            {
                pl.axis_dir[0] = d[0];
                pl.axis_dir[1] = d[1];
                pl.axis_dir[2] = d[2];
                dir_set = true;
            }
            if (!dir_set && !axis.object_name.empty())
            {
                double o[3];
                if (LookupDatumLineAxis(axis.object_name, data_by_name,
                                        m_unit_scale, o, d))
                {
                    pl.axis_origin[0] = o[0];
                    pl.axis_origin[1] = o[1];
                    pl.axis_origin[2] = o[2];
                    pl.axis_dir[0]    = d[0];
                    pl.axis_dir[1]    = d[1];
                    pl.axis_dir[2]    = d[2];
                }
            }
            if (!axis.object_name.empty())
            {
                feat.ext_strings["pattern_axis_ref"] = sub0.empty()
                    ? axis.object_name
                    : axis.object_name + "." + sub0;
            }

            int    count   = PropInt   (props, "Occurrences", 2);
            double angDeg  = PropDouble(props, "Angle",     360.0);
            bool   rev     = PropBool  (props, "Reversed",  false);

            pl.count        = (count >= 1) ? count : 2;
            pl.total_angle  = angDeg * 3.14159265358979323846 / 180.0;
            if (rev) {
                pl.total_angle = -pl.total_angle;
            }

            StashOriginals(feat, PropLinkList(props, "Originals"), m_name_to_id);

            feat.type = FeatType::CircularPattern;
            feat.data = std::move(pl);
        }
        else if (pending.type == "PartDesign::MultiTransform")
        {
            // MultiTransform wraps an ordered list of Transformed
            // features (Mirrored / LinearPattern / PolarPattern). The
            // children themselves are filtered out of the emission
            // walk (see mt_children); we read their parameters here
            // and emit one MultiTransformStep per child. The Replayer
            // chains the corresponding mirror / linear_pattern /
            // circular_pattern ops in order.
            FeatPayloadMultiTransform pl;
            auto xforms = PropLinkList(props, "Transformations");
            pl.steps.reserve(xforms.size());
            for (size_t k = 0; k < xforms.size(); ++k)
            {
                auto cqit = by_name.find(xforms[k]);
                if (cqit == by_name.end()) {
                    continue;
                }
                auto cdit = data_by_name.find(xforms[k]);
                if (cdit == data_by_name.end()) {
                    continue;
                }
                auto child_props = cdit->second.child("Properties");

                MultiTransformStep step;
                if (ReadTransformedStep(cqit->second->type,
                                        child_props,
                                        m_unit_scale,
                                        data_by_name,
                                        step))
                {
                    pl.steps.push_back(step);
                }
            }

            feat.type = FeatType::MultiTransform;
            feat.data = std::move(pl);
            feat.ext_strings["freecad_type"] = pending.type;
            StashOriginals(feat, PropLinkList(props, "Originals"), m_name_to_id);
        }
        else
        {
            // Real unknown feature: container / origin / datum
            // types were already filtered out by IsSkipType /
            // IsContainerType at the top of this loop, so anything
            // reaching here is a feature kind we just don't model
            // yet (e.g. Loft, Revolution, Sweep, ...). In strict
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
