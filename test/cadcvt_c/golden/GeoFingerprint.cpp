#include "GeoFingerprint.h"

#include "brepkit_c/TopoShape.h"

#include <TopoDS_Shape.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

// ============================================================
// test/cadcvt_c/golden/GeoFingerprint.cpp
//
// Single-block grammar:
//
//   geo solids=<n> shells=<n> faces=<n> edges=<n> verts=<n>
//   bbox min=(x,y,z) max=(x,y,z)
//   mass area=<a> volume=<v>
//
// or, for a null / empty shape:
//
//   geo NULL
// ============================================================

namespace cadcvt_golden
{

namespace
{

double Round(double v, int decimals)
{
    double scale = std::pow(10.0, decimals);
    double r = std::round(v * scale) / scale;
    if (r == 0.0) {
        return 0.0;
    }
    return r;
}

std::string Num(double v, int decimals)
{
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(decimals);
    os << Round(v, decimals);
    return os.str();
}

int CountSub(const TopoDS_Shape& s, TopAbs_ShapeEnum kind)
{
    TopTools_IndexedMapOfShape m;
    TopExp::MapShapes(s, kind, m);
    return m.Extent();
}

} // anonymous namespace

std::string FingerprintShape(const std::shared_ptr<brepkit::TopoShape>& shape,
                             const GeoFingerprintOptions& opt)
{
    if (!shape) {
        return "geo NULL\n";
    }

    const TopoDS_Shape& s = shape->GetShape();
    if (s.IsNull()) {
        return "geo NULL\n";
    }

    std::ostringstream os;

    os << "geo"
       << " solids=" << CountSub(s, TopAbs_SOLID)
       << " shells=" << CountSub(s, TopAbs_SHELL)
       << " faces="  << CountSub(s, TopAbs_FACE)
       << " edges="  << CountSub(s, TopAbs_EDGE)
       << " verts="  << CountSub(s, TopAbs_VERTEX)
       << "\n";

    // Bounding box. UseTriangulation=false keeps the box tied to the
    // exact geometry rather than whatever mesh happens to exist.
    Bnd_Box box;
    BRepBndLib::Add(s, box);
    if (!box.IsVoid())
    {
        double xmin, ymin, zmin, xmax, ymax, zmax;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        os << "bbox"
           << " min=(" << Num(xmin, opt.bbox_decimals)
           << ","      << Num(ymin, opt.bbox_decimals)
           << ","      << Num(zmin, opt.bbox_decimals) << ")"
           << " max=(" << Num(xmax, opt.bbox_decimals)
           << ","      << Num(ymax, opt.bbox_decimals)
           << ","      << Num(zmax, opt.bbox_decimals) << ")"
           << "\n";
    }
    else {
        os << "bbox VOID\n";
    }

    // Surface area and volume. Volume is only meaningful for solids;
    // for an open shell it comes back ~0, which is itself a stable,
    // diffable value.
    GProp_GProps surf_props;
    BRepGProp::SurfaceProperties(s, surf_props);
    double area = surf_props.Mass();

    GProp_GProps vol_props;
    BRepGProp::VolumeProperties(s, vol_props);
    double volume = vol_props.Mass();

    // Flake triage hook: the replay is not bit-deterministic (see the
    // tolerant-compare note in golden_main.cpp), so when a mass value
    // flips at the printed precision, this shows the raw jitter.
    if (std::getenv("CAX_GEO_RAW")) {
        std::fprintf(stderr, "[geo_raw] area=%.17g volume=%.17g\n", area, volume);
    }

    os << "mass"
       << " area=" << Num(area, opt.volume_decimals)
       << " volume=" << Num(volume, opt.volume_decimals)
       << "\n";

    return os.str();
}

} // namespace cadcvt_golden
