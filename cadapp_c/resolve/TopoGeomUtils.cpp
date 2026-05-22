#include "cadapp_c/resolve/TopoGeomUtils.h"

#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <gp_Vec.hxx>

namespace cadapp
{

bool EdgeMidpoint(const TopoDS_Edge& edge, gp_Pnt& mid, gp_Dir& tangent)
{
    if (BRep_Tool::Degenerated(edge)) {
        return false;
    }

    BRepAdaptor_Curve curve(edge);
    double first = curve.FirstParameter();
    double last  = curve.LastParameter();
    double t     = (first + last) * 0.5;

    gp_Pnt p;
    gp_Vec dv;
    curve.D1(t, p, dv);

    if (dv.Magnitude() < 1e-12) {
        return false;
    }
    mid     = p;
    tangent = gp_Dir(dv);
    return true;
}

bool FaceCenter(const TopoDS_Face& face, gp_Pnt& center, gp_Dir& normal)
{
    BRepAdaptor_Surface surf(face);
    double u = (surf.FirstUParameter() + surf.LastUParameter()) * 0.5;
    double v = (surf.FirstVParameter() + surf.LastVParameter()) * 0.5;

    gp_Pnt p;
    gp_Vec du;
    gp_Vec dv;
    surf.D1(u, v, p, du, dv);

    gp_Vec n = du.Crossed(dv);
    if (n.Magnitude() < 1e-12) {
        return false;
    }
    center = p;
    normal = gp_Dir(n);
    return true;
}

double EdgeArcLength(const TopoDS_Edge& edge)
{
    if (BRep_Tool::Degenerated(edge)) {
        return 0.0;
    }
    GProp_GProps props;
    BRepGProp::LinearProperties(edge, props);
    return props.Mass();
}

double FaceArea(const TopoDS_Face& face)
{
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    return props.Mass();
}

} // namespace cadapp
