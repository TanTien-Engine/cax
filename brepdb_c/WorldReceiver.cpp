#include "brepdb_c/WorldReceiver.h"

#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Ax22d.hxx>

#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_BSplineCurve.hxx>

#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array2OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>

namespace
{

gp_Pnt PopPoint(const std::vector<double>& d, uint32_t& offset)
{
    double x = d[offset++], y = d[offset++], z = d[offset++];
    return gp_Pnt(x, y, z);
}

gp_Dir PopDir(const std::vector<double>& d, uint32_t& offset)
{
    double x = d[offset++], y = d[offset++], z = d[offset++];
    return gp_Dir(x, y, z);
}

}

namespace brepdb
{

WorldReceiver::WorldReceiver(const BRepWorld& world)
    : m_world(world)
{
}

TopoDS_Shape WorldReceiver::GetShape(uint32_t uid)
{
    auto it = m_cache.find(uid);
    if (it != m_cache.end())
        return it->second;

    const Type* t = m_world.Types().Get(uid);
    if (!t) return TopoDS_Shape();

    TopoDS_Shape result;
    switch (*t)
    {
    case Type::Vertex: result = DeserializeVertex(uid); break;
    case Type::Edge:   result = DeserializeEdge(uid);   break;
    case Type::Face:   result = DeserializeFace(uid);   break;
    case Type::Solid:  result = DeserializeSolid(uid);  break;
    default: break;
    }

    m_cache[uid] = result;
    return result;
}

TopoDS_Shape WorldReceiver::GetAll()
{
    BRep_Builder B;
    TopoDS_Compound compound;
    B.MakeCompound(compound);

    for (uint32_t id : m_world.AliveEntities())
    {
        const Type* t = m_world.Types().Get(id);
        if (t && *t == Type::Solid)
        {
            TopoDS_Shape s = GetShape(id);
            if (!s.IsNull())
                B.Add(compound, s);
        }
    }

    return compound;
}

TopoDS_Vertex WorldReceiver::DeserializeVertex(uint32_t uid)
{
    const PositionComp* pos = m_world.Positions().Get(uid);
    if (!pos) return TopoDS_Vertex();

    const ToleranceComp* tol = m_world.Tolerances().Get(uid);
    double tolerance = tol ? tol->value : 0.0;

    TopoDS_Vertex V;
    BRep_Builder B;
    B.MakeVertex(V, gp_Pnt(pos->x, pos->y, pos->z), tolerance);
    return V;
}

TopoDS_Edge WorldReceiver::DeserializeEdge(uint32_t uid)
{
    const EdgeTopoComp* et = m_world.EdgeTopos().Get(uid);
    if (!et) return TopoDS_Edge();

    TopoDS_Vertex vFirst, vLast;
    if (et->v_first != UINT32_MAX)
    {
        TopoDS_Shape s = GetShape(et->v_first);
        if (!s.IsNull()) vFirst = TopoDS::Vertex(s);
    }
    if (et->v_last != UINT32_MAX)
    {
        TopoDS_Shape s = GetShape(et->v_last);
        if (!s.IsNull()) vLast = TopoDS::Vertex(s);
    }

    const ToleranceComp* tol = m_world.Tolerances().Get(uid);
    double tolerance = tol ? tol->value : 0.0;

    TopoDS_Edge E;
    BRep_Builder B;
    B.MakeEdge(E);

    const CurveComp* curve_comp = m_world.Curves().Get(uid);
    if (curve_comp)
    {
        Handle(Geom_Curve) curve = DeserializeCurve(*curve_comp);
        if (!curve.IsNull())
        {
            B.UpdateEdge(E, curve, tolerance);
            B.Range(E, et->t_first, et->t_last);
        }
        else
        {
            B.UpdateEdge(E, tolerance);
        }
    }
    else
    {
        B.UpdateEdge(E, tolerance);
    }

    if (!vFirst.IsNull() && !vLast.IsNull())
    {
        B.Add(E, vFirst.Oriented(TopAbs_FORWARD));
        B.Add(E, vLast.Oriented(TopAbs_REVERSED));
    }

    return E;
}

TopoDS_Face WorldReceiver::DeserializeFace(uint32_t uid)
{
    const FaceTopoComp* ft = m_world.FaceTopos().Get(uid);
    if (!ft) return TopoDS_Face();

    const ToleranceComp* tol = m_world.Tolerances().Get(uid);
    double tolerance = tol ? tol->value : 0.0;

    const SurfaceComp* surf_comp = m_world.Surfaces().Get(uid);
    Handle(Geom_Surface) surf;
    if (surf_comp)
        surf = DeserializeSurface(*surf_comp);

    TopoDS_Face F;
    BRep_Builder B;
    if (!surf.IsNull())
        B.MakeFace(F, surf, tolerance);
    else
        B.MakeFace(F);

    if (ft->has_outer_wire)
    {
        TopoDS_Wire outer = DeserializeWire(
            ft->outer_wire_edges.data(), ft->outer_wire_edges.size(),
            ft->outer_wire_orientation, F);
        B.Add(F, outer);
    }

    for (auto& iw : ft->inner_wires)
    {
        TopoDS_Wire inner = DeserializeWire(
            iw.edges.data(), iw.edges.size(), iw.orientation, F);
        B.Add(F, inner);
    }

    BRepTools::Update(F);
    F.Orientation(static_cast<TopAbs_Orientation>(ft->orientation));
    return F;
}

TopoDS_Solid WorldReceiver::DeserializeSolid(uint32_t uid)
{
    const SolidTopoComp* st = m_world.SolidTopos().Get(uid);
    if (!st) return TopoDS_Solid();

    TopoDS_Solid S;
    BRep_Builder B;
    B.MakeSolid(S);

    for (auto& sh : st->shells)
    {
        TopoDS_Shell shell = DeserializeShell(sh);
        B.Add(S, shell);
    }

    return S;
}

TopoDS_Wire WorldReceiver::DeserializeWire(
    const FaceTopoComp::WireEdgeRef* edges, size_t count,
    uint8_t orientation, const TopoDS_Face& face)
{
    TopoDS_Wire W;
    BRep_Builder B;
    B.MakeWire(W);

    for (size_t i = 0; i < count; ++i)
    {
        auto& ref = edges[i];
        TopoDS_Edge E = TopoDS::Edge(GetShape(ref.edge_uid));

        if (ref.pcurve.curve_type != Type::Empty)
        {
            Handle(Geom2d_Curve) pc = DeserializeCurve2d(ref.pcurve);
            if (!pc.IsNull())
            {
                TopoDS_Edge fwd_E = TopoDS::Edge(E.Oriented(TopAbs_FORWARD));
                B.UpdateEdge(fwd_E, pc, face, BRep_Tool::Tolerance(E));
                B.Range(fwd_E, face, ref.pcurve.first, ref.pcurve.last);
                B.SameParameter(fwd_E, true);
                B.SameRange(fwd_E, true);
            }
        }

        E.Orientation(static_cast<TopAbs_Orientation>(ref.orientation));
        B.Add(W, E);
    }

    W.Orientation(static_cast<TopAbs_Orientation>(orientation));
    return W;
}

TopoDS_Shell WorldReceiver::DeserializeShell(const SolidTopoComp::ShellComp& shell)
{
    TopoDS_Shell Sh;
    BRep_Builder B;
    B.MakeShell(Sh);

    for (uint32_t fuid : shell.face_uids)
    {
        TopoDS_Shape s = GetShape(fuid);
        if (!s.IsNull())
            B.Add(Sh, TopoDS::Face(s));
    }

    Sh.Orientation(static_cast<TopAbs_Orientation>(shell.orientation));
    return Sh;
}

Handle(Geom_Curve) WorldReceiver::DeserializeCurve(const CurveComp& comp)
{
    const auto& d = comp.data;
    uint32_t offset = 0;

    if (comp.curve_type == Type::Line)
    {
        gp_Pnt loc = PopPoint(d, offset);
        gp_Dir dir = PopDir(d, offset);
        return new Geom_Line(loc, dir);
    }
    else if (comp.curve_type == Type::Circle)
    {
        gp_Pnt loc = PopPoint(d, offset);
        gp_Dir dir = PopDir(d, offset);
        gp_Dir xdir = PopDir(d, offset);
        double r = d[offset++];
        return new Geom_Circle(gp_Ax2(loc, dir, xdir), r);
    }
    else if (comp.curve_type == Type::Ellipse)
    {
        gp_Pnt loc = PopPoint(d, offset);
        gp_Dir dir = PopDir(d, offset);
        gp_Dir xdir = PopDir(d, offset);
        double majorR = d[offset++];
        double minorR = d[offset++];
        return new Geom_Ellipse(gp_Ax2(loc, dir, xdir), majorR, minorR);
    }
    else if (comp.curve_type == Type::BSplineCurve)
    {
        int degree  = static_cast<int>(d[offset++]);
        int nbPoles = static_cast<int>(d[offset++]);
        int nbKnots = static_cast<int>(d[offset++]);
        bool isRational = d[offset++] > 0.5;
        bool isPeriodic = d[offset++] > 0.5;

        TColgp_Array1OfPnt poles(1, nbPoles);
        for (int i = 1; i <= nbPoles; ++i)
            poles(i) = PopPoint(d, offset);

        TColStd_Array1OfReal weights(1, nbPoles);
        if (isRational)
            for (int i = 1; i <= nbPoles; ++i)
                weights(i) = d[offset++];
        else
            weights.Init(1.0);

        TColStd_Array1OfReal knots(1, nbKnots);
        for (int i = 1; i <= nbKnots; ++i)
            knots(i) = d[offset++];

        TColStd_Array1OfInteger mults(1, nbKnots);
        for (int i = 1; i <= nbKnots; ++i)
            mults(i) = static_cast<int>(d[offset++]);

        return new Geom_BSplineCurve(poles, weights, knots, mults, degree, isPeriodic);
    }

    return nullptr;
}

Handle(Geom2d_Curve) WorldReceiver::DeserializeCurve2d(const Curve2dComp& comp)
{
    const auto& d = comp.data;
    uint32_t offset = 0;

    if (comp.curve_type == Type::Line)
    {
        double lx = d[offset++], ly = d[offset++];
        double dx = d[offset++], dy = d[offset++];
        return new Geom2d_Line(gp_Pnt2d(lx, ly), gp_Dir2d(dx, dy));
    }
    else if (comp.curve_type == Type::Circle)
    {
        double cx = d[offset++], cy = d[offset++];
        double xx = d[offset++], xy = d[offset++];
        double r = d[offset++];
        return new Geom2d_Circle(gp_Ax22d(gp_Pnt2d(cx, cy), gp_Dir2d(xx, xy)), r);
    }
    else if (comp.curve_type == Type::Ellipse)
    {
        double cx = d[offset++], cy = d[offset++];
        double xx = d[offset++], xy = d[offset++];
        double majorR = d[offset++];
        double minorR = d[offset++];
        return new Geom2d_Ellipse(gp_Ax22d(gp_Pnt2d(cx, cy), gp_Dir2d(xx, xy)), majorR, minorR);
    }
    else if (comp.curve_type == Type::BSplineCurve)
    {
        int degree  = static_cast<int>(d[offset++]);
        int nbPoles = static_cast<int>(d[offset++]);
        int nbKnots = static_cast<int>(d[offset++]);
        bool isRational = d[offset++] > 0.5;
        bool isPeriodic = d[offset++] > 0.5;

        TColgp_Array1OfPnt2d poles(1, nbPoles);
        for (int i = 1; i <= nbPoles; ++i)
        {
            double x = d[offset++], y = d[offset++];
            poles(i) = gp_Pnt2d(x, y);
        }

        TColStd_Array1OfReal weights(1, nbPoles);
        if (isRational)
            for (int i = 1; i <= nbPoles; ++i)
                weights(i) = d[offset++];
        else
            weights.Init(1.0);

        TColStd_Array1OfReal knots(1, nbKnots);
        for (int i = 1; i <= nbKnots; ++i)
            knots(i) = d[offset++];

        TColStd_Array1OfInteger mults(1, nbKnots);
        for (int i = 1; i <= nbKnots; ++i)
            mults(i) = static_cast<int>(d[offset++]);

        return new Geom2d_BSplineCurve(poles, weights, knots, mults, degree, isPeriodic);
    }

    return nullptr;
}

Handle(Geom_Surface) WorldReceiver::DeserializeSurface(const SurfaceComp& comp)
{
    const auto& d = comp.data;
    uint32_t offset = 0;

    if (comp.surface_type == Type::Plane)
    {
        gp_Pnt loc = PopPoint(d, offset);
        gp_Dir axisDir = PopDir(d, offset);
        gp_Dir xAxisDir = PopDir(d, offset);
        return new Geom_Plane(gp_Ax3(loc, axisDir, xAxisDir));
    }
    else if (comp.surface_type == Type::Cylinder)
    {
        gp_Pnt loc = PopPoint(d, offset);
        gp_Dir axisDir = PopDir(d, offset);
        gp_Dir xAxisDir = PopDir(d, offset);
        double r = d[offset++];
        return new Geom_CylindricalSurface(gp_Ax3(loc, axisDir, xAxisDir), r);
    }
    else if (comp.surface_type == Type::Sphere)
    {
        gp_Pnt loc = PopPoint(d, offset);
        gp_Dir axisDir = PopDir(d, offset);
        gp_Dir xAxisDir = PopDir(d, offset);
        double r = d[offset++];
        return new Geom_SphericalSurface(gp_Ax3(loc, axisDir, xAxisDir), r);
    }
    else if (comp.surface_type == Type::Torus)
    {
        gp_Pnt loc = PopPoint(d, offset);
        gp_Dir axisDir = PopDir(d, offset);
        gp_Dir xAxisDir = PopDir(d, offset);
        double majorR = d[offset++];
        double minorR = d[offset++];
        return new Geom_ToroidalSurface(gp_Ax3(loc, axisDir, xAxisDir), majorR, minorR);
    }
    else if (comp.surface_type == Type::Cone)
    {
        gp_Pnt loc = PopPoint(d, offset);
        gp_Dir axisDir = PopDir(d, offset);
        gp_Dir xAxisDir = PopDir(d, offset);
        double refR = d[offset++];
        double semiAngle = d[offset++];
        return new Geom_ConicalSurface(gp_Ax3(loc, axisDir, xAxisDir), semiAngle, refR);
    }
    else if (comp.surface_type == Type::BSplineSurface)
    {
        int uDegree  = static_cast<int>(d[offset++]);
        int vDegree  = static_cast<int>(d[offset++]);
        int nbUPoles = static_cast<int>(d[offset++]);
        int nbVPoles = static_cast<int>(d[offset++]);
        int nbUKnots = static_cast<int>(d[offset++]);
        int nbVKnots = static_cast<int>(d[offset++]);
        bool isURational = d[offset++] > 0.5;
        bool isVRational = d[offset++] > 0.5;
        bool isUPeriodic = d[offset++] > 0.5;
        bool isVPeriodic = d[offset++] > 0.5;

        TColgp_Array2OfPnt poles(1, nbUPoles, 1, nbVPoles);
        for (int u = 1; u <= nbUPoles; ++u)
            for (int v = 1; v <= nbVPoles; ++v)
                poles(u, v) = PopPoint(d, offset);

        TColStd_Array2OfReal weights(1, nbUPoles, 1, nbVPoles);
        if (isURational || isVRational)
            for (int u = 1; u <= nbUPoles; ++u)
                for (int v = 1; v <= nbVPoles; ++v)
                    weights(u, v) = d[offset++];
        else
            weights.Init(1.0);

        TColStd_Array1OfReal uKnots(1, nbUKnots);
        for (int i = 1; i <= nbUKnots; ++i)
            uKnots(i) = d[offset++];

        TColStd_Array1OfInteger uMults(1, nbUKnots);
        for (int i = 1; i <= nbUKnots; ++i)
            uMults(i) = static_cast<int>(d[offset++]);

        TColStd_Array1OfReal vKnots(1, nbVKnots);
        for (int i = 1; i <= nbVKnots; ++i)
            vKnots(i) = d[offset++];

        TColStd_Array1OfInteger vMults(1, nbVKnots);
        for (int i = 1; i <= nbVKnots; ++i)
            vMults(i) = static_cast<int>(d[offset++]);

        return new Geom_BSplineSurface(poles, weights, uKnots, vKnots,
                                       uMults, vMults, uDegree, vDegree,
                                       isUPeriodic, isVPeriodic);
    }

    return nullptr;
}

}
