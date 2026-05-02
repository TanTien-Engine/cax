#include "brepir_c/Receiver.h"

#include <BRep_Builder.hxx>
#include <TopoDS.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Lin.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>
#include <gp_Cylinder.hxx>

#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_BSplineSurface.hxx>

#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array2OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>

namespace 
{

gp_Pnt PopPoint(const brepir::GeometryPool& p, uint32_t& offset)
{
    double x = p.data_pool[offset++];
    double y = p.data_pool[offset++];
    double z = p.data_pool[offset++];
    return gp_Pnt(x, y, z);
}

gp_Dir PopDir(const brepir::GeometryPool& p, uint32_t& offset)
{
    double x = p.data_pool[offset++];
    double y = p.data_pool[offset++];
    double z = p.data_pool[offset++];
    return gp_Dir(x, y, z);
}

}

namespace brepir
{

Receiver::Receiver(const GeometryPool& pool)
    : m_pool(pool)
{
    for (const auto& h : m_pool.headers) {
        m_header_map[h.persistent_id] = &h;
    }
}

TopoDS_Shape Receiver::GetShape(uint32_t uid)
{
    auto cache_it = m_rebuilt_shapes.find(uid);
    if (cache_it != m_rebuilt_shapes.end()) {
        return cache_it->second;
    }

    auto head_it = m_header_map.find(uid);
    if (head_it == m_header_map.end()) {
        return TopoDS_Shape();
    }
    
    const Header* header = head_it->second;
    uint32_t offset = header->param_offset;
    TopoDS_Shape result;

    switch (header->type)
    {
    case Type::Vertex: 
        result = DeserializeVertex(offset); 
        break;
    case Type::Edge:   
        result = DeserializeEdge(offset); 
        break;
    case Type::Face:   
        result = DeserializeFace(offset); 
        break;
    case Type::Solid:  
        result = DeserializeSolid(offset); 
        break;
    default:           
        break;
    }

    m_rebuilt_shapes[uid] = result;
    return result;
}

TopoDS_Vertex Receiver::DeserializeVertex(uint32_t& offset)
{
    gp_Pnt pt = PopPoint(m_pool, offset);
    double tol = m_pool.data_pool[offset++];

    TopoDS_Vertex V;
    BRep_Builder B;
    B.MakeVertex(V, pt, tol);
    return V;
}

TopoDS_Edge Receiver::DeserializeEdge(uint32_t& offset)
{
    double vFirst_id_val = m_pool.data_pool[offset++];
    double vLast_id_val  = m_pool.data_pool[offset++];

    TopoDS_Vertex vFirst, vLast;
    if (vFirst_id_val >= 0.0) {
        TopoDS_Shape s = GetShape(static_cast<uint32_t>(vFirst_id_val));
        if (!s.IsNull()) vFirst = TopoDS::Vertex(s);
    }
    if (vLast_id_val >= 0.0) {
        TopoDS_Shape s = GetShape(static_cast<uint32_t>(vLast_id_val));
        if (!s.IsNull()) vLast = TopoDS::Vertex(s);
    }

    double tol   = m_pool.data_pool[offset++];
    double first = m_pool.data_pool[offset++];
    double last  = m_pool.data_pool[offset++];

    TopoDS_Edge E;
    BRep_Builder B;
    B.MakeEdge(E);

    Handle(Geom_Curve) curve = DeserializeCurve(offset);
    if (!curve.IsNull()) {
        B.UpdateEdge(E, curve, tol);
        B.Range(E, first, last);
    } else {
        B.UpdateEdge(E, tol);
    }

    if (!vFirst.IsNull() && !vLast.IsNull()) 
    {
        B.Add(E, vFirst.Oriented(TopAbs_FORWARD));
        B.Add(E, vLast.Oriented(TopAbs_REVERSED));
    }

    return E;
}

TopoDS_Face Receiver::DeserializeFace(uint32_t& offset)
{
    double tol = m_pool.data_pool[offset++];
    TopAbs_Orientation ori = static_cast<TopAbs_Orientation>(m_pool.data_pool[offset++]);

    Handle(Geom_Surface) surf = DeserializeSurface(offset);

    TopoDS_Face F;
    BRep_Builder B;
    B.MakeFace(F, surf, tol);

    double has_outer_wire = m_pool.data_pool[offset++];
    if (has_outer_wire > 0.5) 
    {
        TopoDS_Wire outer_wire = DeserializeWire(offset);
        B.Add(F, outer_wire);
    }

    int inner_count = static_cast<int>(m_pool.data_pool[offset++]);
    for (int i = 0; i < inner_count; ++i) 
    {
        TopoDS_Wire inner_wire = DeserializeWire(offset);
        B.Add(F, inner_wire);
    }

    F.Orientation(ori);

    return F;
}

TopoDS_Solid Receiver::DeserializeSolid(uint32_t& offset)
{
    TopoDS_Solid S;
    BRep_Builder B;
    B.MakeSolid(S);

    int count = static_cast<int>(m_pool.data_pool[offset++]);
    for (int i = 0; i < count; ++i) 
    {
        TopoDS_Shell shell = DeserializeShell(offset);
        B.Add(S, shell);
    }

    return S;
}

TopoDS_Wire Receiver::DeserializeWire(uint32_t& offset)
{
    TopoDS_Wire W;
    BRep_Builder B;
    B.MakeWire(W);

    TopAbs_Orientation wire_ori = static_cast<TopAbs_Orientation>(m_pool.data_pool[offset++]);

    int count = static_cast<int>(m_pool.data_pool[offset++]);
    for (int i = 0; i < count; ++i) 
    {
        uint32_t edge_uid = static_cast<uint32_t>(m_pool.data_pool[offset++]);
        TopAbs_Orientation ori = static_cast<TopAbs_Orientation>(m_pool.data_pool[offset++]);
        
        TopoDS_Edge E = TopoDS::Edge(GetShape(edge_uid));
        E.Orientation(ori);
        B.Add(W, E);
    }

    W.Orientation(wire_ori);

    return W;
}

TopoDS_Shell Receiver::DeserializeShell(uint32_t& offset)
{
    TopAbs_Orientation ori = static_cast<TopAbs_Orientation>(m_pool.data_pool[offset++]);
    int count = static_cast<int>(m_pool.data_pool[offset++]);

    TopoDS_Shell Sh;
    BRep_Builder B;
    B.MakeShell(Sh);

    for (int i = 0; i < count; ++i) 
    {
        uint32_t face_uid = static_cast<uint32_t>(m_pool.data_pool[offset++]);
        TopoDS_Face F = TopoDS::Face(GetShape(face_uid));
        B.Add(Sh, F); 
    }

    Sh.Orientation(ori);

    return Sh;
}

Handle(Geom_Curve) Receiver::DeserializeCurve(uint32_t& offset)
{
    Type c_type = static_cast<Type>(m_pool.data_pool[offset++]);
    
    if (c_type == Type::Empty) {
        return nullptr;
    }

    if (c_type == Type::Line) 
    {
        gp_Pnt loc = PopPoint(m_pool, offset);
        gp_Dir dir = PopDir(m_pool, offset);
        return new Geom_Line(loc, dir);
    }
    else if (c_type == Type::Circle) 
    {
        gp_Pnt loc = PopPoint(m_pool, offset);
        gp_Dir dir = PopDir(m_pool, offset);
        double r = m_pool.data_pool[offset++];
        gp_Ax2 ax2(loc, dir);
        return new Geom_Circle(ax2, r);
    }
    else if (c_type == Type::BSplineCurve) 
    {
        int degree = static_cast<int>(m_pool.data_pool[offset++]);
        int nbPoles = static_cast<int>(m_pool.data_pool[offset++]);
        int nbKnots = static_cast<int>(m_pool.data_pool[offset++]);
        bool isRational = (m_pool.data_pool[offset++] > 0.5);
        bool isPeriodic = (m_pool.data_pool[offset++] > 0.5);

        TColgp_Array1OfPnt poles(1, nbPoles);
        for (int i = 1; i <= nbPoles; ++i) {
            poles(i) = PopPoint(m_pool, offset);
        }

        TColStd_Array1OfReal weights(1, nbPoles);
        if (isRational) 
        {
            for (int i = 1; i <= nbPoles; ++i) {
                weights(i) = m_pool.data_pool[offset++];
            }
        } 
        else 
        {
            weights.Init(1.0);
        }

        TColStd_Array1OfReal knots(1, nbKnots);
        for (int i = 1; i <= nbKnots; ++i) {
            knots(i) = m_pool.data_pool[offset++];
        }

        TColStd_Array1OfInteger mults(1, nbKnots);
        for (int i = 1; i <= nbKnots; ++i) {
            mults(i) = static_cast<int>(m_pool.data_pool[offset++]);
        }

        return new Geom_BSplineCurve(poles, weights, knots, mults, degree, isPeriodic);
    }
    
    return nullptr;
}

Handle(Geom_Surface) Receiver::DeserializeSurface(uint32_t& offset)
{
    Type s_type = static_cast<Type>(m_pool.data_pool[offset++]);
    
    if (s_type == Type::Plane) 
    {
        gp_Pnt loc = PopPoint(m_pool, offset);
        gp_Dir axisDir = PopDir(m_pool, offset);
        gp_Dir xAxisDir = PopDir(m_pool, offset);
        gp_Ax3 ax3(loc, axisDir, xAxisDir);
        return new Geom_Plane(ax3);
    }
    else if (s_type == Type::Cylinder) {
        gp_Pnt loc = PopPoint(m_pool, offset);
        gp_Dir axisDir = PopDir(m_pool, offset);
        gp_Dir xAxisDir = PopDir(m_pool, offset);
        double r = m_pool.data_pool[offset++];
        gp_Ax3 ax3(loc, axisDir, xAxisDir);
        return new Geom_CylindricalSurface(ax3, r);
    }
    else if (s_type == Type::BSplineSurface) 
    {
        int uDegree = static_cast<int>(m_pool.data_pool[offset++]);
        int vDegree = static_cast<int>(m_pool.data_pool[offset++]);
        int nbUPoles = static_cast<int>(m_pool.data_pool[offset++]);
        int nbVPoles = static_cast<int>(m_pool.data_pool[offset++]);
        int nbUKnots = static_cast<int>(m_pool.data_pool[offset++]);
        int nbVKnots = static_cast<int>(m_pool.data_pool[offset++]);
        bool isURational = (m_pool.data_pool[offset++] > 0.5);
        bool isVRational = (m_pool.data_pool[offset++] > 0.5);
        bool isUPeriodic = (m_pool.data_pool[offset++] > 0.5);
        bool isVPeriodic = (m_pool.data_pool[offset++] > 0.5);

        TColgp_Array2OfPnt poles(1, nbUPoles, 1, nbVPoles);
        for (int u = 1; u <= nbUPoles; ++u) {
            for (int v = 1; v <= nbVPoles; ++v) {
                poles(u, v) = PopPoint(m_pool, offset);
            }
        }

        TColStd_Array2OfReal weights(1, nbUPoles, 1, nbVPoles);
        if (isURational || isVRational) {
            for (int u = 1; u <= nbUPoles; ++u) {
                for (int v = 1; v <= nbVPoles; ++v) {
                    weights(u, v) = m_pool.data_pool[offset++];
                }
            }
        } else {
            weights.Init(1.0);
        }

        TColStd_Array1OfReal uKnots(1, nbUKnots);
        for (int i = 1; i <= nbUKnots; ++i) {
            uKnots(i) = m_pool.data_pool[offset++];
        }

        TColStd_Array1OfInteger uMults(1, nbUKnots);
        for (int i = 1; i <= nbUKnots; ++i) {
            uMults(i) = static_cast<int>(m_pool.data_pool[offset++]);
        }

        TColStd_Array1OfReal vKnots(1, nbVKnots);
        for (int i = 1; i <= nbVKnots; ++i) {
            vKnots(i) = m_pool.data_pool[offset++];
        }

        TColStd_Array1OfInteger vMults(1, nbVKnots);
        for (int i = 1; i <= nbVKnots; ++i) {
            vMults(i) = static_cast<int>(m_pool.data_pool[offset++]);
        }

        return new Geom_BSplineSurface(poles, weights, uKnots, vKnots, uMults, vMults, uDegree, vDegree, isUPeriodic, isVPeriodic);
    }
    
    return nullptr;
}

}