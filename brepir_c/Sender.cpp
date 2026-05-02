#include "brepir_c/Sender.h"
#include "brepir_c/Data.h"

#include "partgraph_c/GlobalConfig.h"
#include "partgraph_c/TopoShape.h"
#include "breptopo_c/HistGraph.h"
#include "breptopo_c/TopoNaming.h"
#include "breptopo_c/NodeId.h"
#include "graph/Node.h"

#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <gp_Lin.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>
#include <gp_Cylinder.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>

namespace
{

void PushPoint(brepir::GeometryPool& p, const gp_Pnt& pt)
{
    p.data_pool.push_back(pt.X()); p.data_pool.push_back(pt.Y()); p.data_pool.push_back(pt.Z());
}

void PushDir(brepir::GeometryPool& p, const gp_Dir& d)
{
    p.data_pool.push_back(d.X()); p.data_pool.push_back(d.Y()); p.data_pool.push_back(d.Z());
}

}

namespace brepir
{

Sender::Sender(const std::shared_ptr<breptopo::TopoNaming>& tn)
    : m_tn(tn)
{
}

void Sender::SerializeVertex(const TopoDS_Vertex& vertex, uint32_t uid, GeometryPool& pool)
{
    Header header{ Type::Vertex, uid, (uint32_t)pool.data_pool.size(), 0 };

    gp_Pnt pt = BRep_Tool::Pnt(vertex);
    PushPoint(pool, pt);

    pool.data_pool.push_back(BRep_Tool::Tolerance(vertex));

    header.param_count = (uint32_t)pool.data_pool.size() - header.param_offset;
    pool.headers.push_back(header);
}

void Sender::SerializeEdge(const TopoDS_Edge& edge, uint32_t uid, GeometryPool& pool)
{
    Header header{ Type::Edge, uid, (uint32_t)pool.data_pool.size(), 0 };

    TopoDS_Vertex vFirst, vLast;
    TopExp::Vertices(edge, vFirst, vLast);

    pool.data_pool.push_back(vFirst.IsNull() ? -1.0 : static_cast<double>(GetUID(vFirst)));
    pool.data_pool.push_back(vLast.IsNull() ? -1.0 : static_cast<double>(GetUID(vLast)));

    Standard_Real first, last;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

    pool.data_pool.push_back(BRep_Tool::Tolerance(edge));
    pool.data_pool.push_back(first);
    pool.data_pool.push_back(last);

    if (!curve.IsNull()) {
        SerializeCurve(curve, pool);
    } else {
        pool.data_pool.push_back(static_cast<double>(Type::Empty));
    }

    header.param_count = (uint32_t)pool.data_pool.size() - header.param_offset;
    pool.headers.push_back(header);
}

void Sender::SerializeFace(const TopoDS_Face& face, uint32_t uid, GeometryPool& pool)
{
    Header header{ Type::Face, uid, (uint32_t)pool.data_pool.size(), 0 };

    pool.data_pool.push_back(BRep_Tool::Tolerance(face));
    pool.data_pool.push_back(static_cast<double>(face.Orientation()));

    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    SerializeSurface(surf, pool);

    TopoDS_Wire outer_wire = BRepTools::OuterWire(face);
    pool.data_pool.push_back(outer_wire.IsNull() ? 0.0 : 1.0);
    if (!outer_wire.IsNull()) {
        SerializeWire(outer_wire, pool);
    }

    uint32_t countIndex = pool.data_pool.size();
    pool.data_pool.push_back(0.0);
    int count = 0;
    for (TopExp_Explorer exp(face, TopAbs_WIRE); exp.More(); exp.Next()) 
    {
        const TopoDS_Wire& current_wire = TopoDS::Wire(exp.Current());
        if (outer_wire.IsNull() || !current_wire.IsSame(outer_wire)) 
        {
            SerializeWire(current_wire, pool);
            ++count;
        }
    }
    pool.data_pool[countIndex] = static_cast<double>(count);

    header.param_count = (uint32_t)pool.data_pool.size() - header.param_offset;
    pool.headers.push_back(header);
}

void Sender::SerializeSolid(const TopoDS_Solid& solid, uint32_t uid, GeometryPool& pool)
{
    Header header{ Type::Solid, uid, (uint32_t)pool.data_pool.size(), 0 };

    uint32_t countIndex = pool.data_pool.size();
    pool.data_pool.push_back(0.0);

    int count = 0;
    for (TopExp_Explorer exp(solid, TopAbs_SHELL); exp.More(); exp.Next())
    {
        const TopoDS_Shell& shell = TopoDS::Shell(exp.Current());
        SerializeShell(shell, pool);
        ++count;
    }
    pool.data_pool[countIndex] = static_cast<double>(count);

    header.param_count = (uint32_t)pool.data_pool.size() - header.param_offset;
    pool.headers.push_back(header);
}

uint32_t Sender::GetUID(const TopoDS_Shape& shape) const
{
    std::shared_ptr<breptopo::HistGraph> hg = nullptr;
    switch (shape.ShapeType())
    {
    case TopAbs_VERTEX:
        hg = m_tn->GetVertexGraph();
        break;
    case TopAbs_EDGE:
        hg = m_tn->GetEdgeGraph();
        break;
    case TopAbs_FACE:
        hg = m_tn->GetFaceGraph();
        break;
    case TopAbs_SOLID:
        hg = m_tn->GetSolidGraph();
        break;
    }

    if (!hg)
        return 0xffffffff;

    auto node = hg->QueryNode(std::make_shared<partgraph::TopoShape>(shape));
    if (!node)
        return 0xffffffff;

    auto& cid = node->GetComponent<breptopo::NodeId>();
    return cid.GetUID();
}

void Sender::SerializeCurve(const Handle(Geom_Curve)& curve, GeometryPool& pool)
{
    if (curve->IsKind(STANDARD_TYPE(Geom_Line))) 
    {
        pool.data_pool.push_back(static_cast<double>(Type::Line));
        auto g = Handle(Geom_Line)::DownCast(curve)->Lin();
        PushPoint(pool, g.Location());
        PushDir(pool, g.Direction());
    }
    else if (curve->IsKind(STANDARD_TYPE(Geom_Circle))) 
    {
        pool.data_pool.push_back(static_cast<double>(Type::Circle));
        auto g = Handle(Geom_Circle)::DownCast(curve)->Circ();
        PushPoint(pool, g.Position().Location());
        PushDir(pool, g.Position().Direction());
        pool.data_pool.push_back(g.Radius());
    }
    else if (curve->IsKind(STANDARD_TYPE(Geom_BSplineCurve))) 
    {
        pool.data_pool.push_back(static_cast<double>(Type::BSplineCurve));

        auto g = Handle(Geom_BSplineCurve)::DownCast(curve);
        
        pool.data_pool.push_back(g->Degree());
        pool.data_pool.push_back(g->NbPoles());
        pool.data_pool.push_back(g->NbKnots());
        pool.data_pool.push_back(g->IsRational() ? 1.0 : 0.0);
        pool.data_pool.push_back(g->IsPeriodic() ? 1.0 : 0.0);

        for (int i = 1; i <= g->NbPoles(); ++i) 
            PushPoint(pool, g->Pole(i));
        if (g->IsRational()) {
            for (int i = 1; i <= g->NbPoles(); ++i) {
                pool.data_pool.push_back(g->Weight(i));
            }
        }
        for (int i = 1; i <= g->NbKnots(); ++i) 
            pool.data_pool.push_back(g->Knot(i));
        for (int i = 1; i <= g->NbKnots(); ++i) 
            pool.data_pool.push_back(static_cast<double>(g->Multiplicity(i)));
    }
}

void Sender::SerializeSurface(const Handle(Geom_Surface)& surf, GeometryPool& pool) 
{
    if (surf->IsKind(STANDARD_TYPE(Geom_Plane))) 
    {
        pool.data_pool.push_back(static_cast<double>(Type::Plane));
        gp_Pln g = Handle(Geom_Plane)::DownCast(surf)->Pln();
        PushPoint(pool, g.Location());
        PushDir(pool, g.Axis().Direction());
        PushDir(pool, g.XAxis().Direction());
    }
    else if (surf->IsKind(STANDARD_TYPE(Geom_CylindricalSurface))) 
    {
        pool.data_pool.push_back(static_cast<double>(Type::Cylinder));
        gp_Cylinder g = Handle(Geom_CylindricalSurface)::DownCast(surf)->Cylinder();
        PushPoint(pool, g.Location());
        PushDir(pool, g.Axis().Direction());
        PushDir(pool, g.XAxis().Direction());
        pool.data_pool.push_back(g.Radius());
    }
    else if (surf->IsKind(STANDARD_TYPE(Geom_BSplineSurface))) 
    {
        pool.data_pool.push_back(static_cast<double>(Type::BSplineSurface));
        auto g = Handle(Geom_BSplineSurface)::DownCast(surf);

        pool.data_pool.push_back(static_cast<double>(g->UDegree()));    // [0]
        pool.data_pool.push_back(static_cast<double>(g->VDegree()));    // [1]
        pool.data_pool.push_back(static_cast<double>(g->NbUPoles()));   // [2]
        pool.data_pool.push_back(static_cast<double>(g->NbVPoles()));   // [3]
        pool.data_pool.push_back(static_cast<double>(g->NbUKnots()));   // [4]
        pool.data_pool.push_back(static_cast<double>(g->NbVKnots()));   // [5]
        pool.data_pool.push_back(g->IsURational() ? 1.0 : 0.0);         // [6]
        pool.data_pool.push_back(g->IsVRational() ? 1.0 : 0.0);         // [7]
        pool.data_pool.push_back(g->IsUPeriodic() ? 1.0 : 0.0);         // [8]
        pool.data_pool.push_back(g->IsVPeriodic() ? 1.0 : 0.0);         // [9]

        for (int u = 1; u <= g->NbUPoles(); ++u) {
            for (int v = 1; v <= g->NbVPoles(); ++v) {
                PushPoint(pool, g->Pole(u, v));
            }
        }

        if (g->IsURational() || g->IsVRational()) {
            for (int u = 1; u <= g->NbUPoles(); ++u) {
                for (int v = 1; v <= g->NbVPoles(); ++v) {
                    pool.data_pool.push_back(g->Weight(u, v));
                }
            }
        }

        for (int i = 1; i <= g->NbUKnots(); ++i) {
            pool.data_pool.push_back(g->UKnot(i));
        }
        for (int i = 1; i <= g->NbUKnots(); ++i) {
            pool.data_pool.push_back(static_cast<double>(g->UMultiplicity(i)));
        }

        for (int i = 1; i <= g->NbVKnots(); ++i) {
            pool.data_pool.push_back(g->VKnot(i));
        }
        for (int i = 1; i <= g->NbVKnots(); ++i) {
            pool.data_pool.push_back(static_cast<double>(g->VMultiplicity(i)));
        }
    }
}

void Sender::SerializeShell(const TopoDS_Shell& shell, GeometryPool& pool)
{
    pool.data_pool.push_back(static_cast<double>(shell.Orientation()));

    std::vector<uint32_t> faceUIDs;
    for (TopExp_Explorer exp(shell, TopAbs_FACE); exp.More(); exp.Next()) {
        faceUIDs.push_back(GetUID(exp.Current()));
    }

    pool.data_pool.push_back(static_cast<double>(faceUIDs.size()));
    for (uint32_t f_uid : faceUIDs) {
        pool.data_pool.push_back(static_cast<double>(f_uid));
    }
}

void Sender::SerializeWire(const TopoDS_Wire& wire, GeometryPool& pool)
{
    pool.data_pool.push_back(static_cast<double>(wire.Orientation()));

    uint32_t countIndex = pool.data_pool.size();
    pool.data_pool.push_back(0.0);

    int count = 0;
    for (BRepTools_WireExplorer exp(wire); exp.More(); exp.Next())
    {
        const TopoDS_Shape& edge = exp.Current();
        pool.data_pool.push_back(static_cast<double>(GetUID(edge)));
        pool.data_pool.push_back(static_cast<double>(edge.Orientation()));
        count++;
    }
    pool.data_pool[countIndex] = static_cast<double>(count);
}

}