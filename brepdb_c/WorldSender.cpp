#include "brepdb_c/WorldSender.h"

#include "partgraph_c/GlobalConfig.h"
#include "partgraph_c/TopoShape.h"
#include "breptopo_c/HistGraph.h"
#include "breptopo_c/TopoNaming.h"
#include "breptopo_c/NodeId.h"
#include "graph/Node.h"

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
#include <gp_Lin.hxx>
#include <gp_Circ.hxx>
#include <gp_Elips.hxx>
#include <gp_Pln.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Sphere.hxx>
#include <gp_Torus.hxx>
#include <gp_Cone.hxx>
#include <gp_Lin2d.hxx>
#include <gp_Circ2d.hxx>
#include <gp_Elips2d.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>

#include <assert.h>

namespace
{

void PushPoint(std::vector<double>& d, const gp_Pnt& pt)
{
    d.push_back(pt.X()); d.push_back(pt.Y()); d.push_back(pt.Z());
}

void PushDir(std::vector<double>& d, const gp_Dir& dir)
{
    d.push_back(dir.X()); d.push_back(dir.Y()); d.push_back(dir.Z());
}

brepdb::AabbComp ComputeAabb(const TopoDS_Shape& shape)
{
    brepdb::AabbComp aabb{};
    Bnd_Box bbox;
    BRepBndLib::Add(shape, bbox);
    if (bbox.IsVoid())
    {
        for (int i = 0; i < 3; ++i)
        {
            aabb.min_pt[i] = 0.0;
            aabb.max_pt[i] = 0.0;
        }
    }
    else
    {
        bbox.Get(aabb.min_pt[0], aabb.min_pt[1], aabb.min_pt[2],
                 aabb.max_pt[0], aabb.max_pt[1], aabb.max_pt[2]);
    }
    return aabb;
}

} // anonymous

namespace brepdb
{

WorldSender::WorldSender(const std::shared_ptr<breptopo::TopoNaming>& tn)
    : m_tn(tn)
{
}

void WorldSender::Serialize(const TopoDS_Shape& shape, BRepWorld& world)
{
    TopTools_IndexedMapOfShape all_shapes;
    TopExp::MapShapes(shape, all_shapes);

    // First pass: build auto-uid map for shapes that have no TopoNaming record
    for (int i = 1; i <= all_shapes.Extent(); ++i)
    {
        const TopoDS_Shape& s = all_shapes(i);
        TopAbs_ShapeEnum type = s.ShapeType();
        if (type != TopAbs_VERTEX && type != TopAbs_EDGE &&
            type != TopAbs_FACE && type != TopAbs_SOLID)
            continue;

        uint32_t uid = 0xffffffff;
        if (m_tn) uid = GetUID(s);
        if (uid == 0xffffffff)
        {
            m_auto_uid_map.Add(s);
        }
    }

    for (int i = 1; i <= all_shapes.Extent(); ++i)
    {
        const TopoDS_Shape& s = all_shapes(i);
        uint32_t uid = ResolveUID(s);
        if (uid == 0xffffffff)
            continue;

        switch (s.ShapeType())
        {
        case TopAbs_SOLID:
            SerializeSolid(TopoDS::Solid(s), uid, world);
            break;
        case TopAbs_FACE:
            SerializeFace(TopoDS::Face(s), uid, world);
            break;
        case TopAbs_EDGE:
            SerializeEdge(TopoDS::Edge(s), uid, world);
            break;
        case TopAbs_VERTEX:
            SerializeVertex(TopoDS::Vertex(s), uid, world);
            break;
        default:
            break;
        }
    }
}

void WorldSender::SerializeVertex(const TopoDS_Vertex& vertex, uint32_t uid, BRepWorld& world)
{
    world.RegisterEntity(uid);
    world.Types().Set(uid, Type::Vertex);
    world.Aabbs().Set(uid, ComputeAabb(vertex));

    gp_Pnt pt = BRep_Tool::Pnt(vertex);
    world.Positions().Set(uid, { pt.X(), pt.Y(), pt.Z() });
    world.Tolerances().Set(uid, { BRep_Tool::Tolerance(vertex) });
}

void WorldSender::SerializeEdge(const TopoDS_Edge& edge, uint32_t uid, BRepWorld& world)
{
    world.RegisterEntity(uid);
    world.Types().Set(uid, Type::Edge);
    world.Aabbs().Set(uid, ComputeAabb(edge));
    world.Tolerances().Set(uid, { BRep_Tool::Tolerance(edge) });

    // Topology: vertex references + parameter range
    EdgeTopoComp topo;
    TopoDS_Vertex vFirst, vLast;
    TopExp::Vertices(edge, vFirst, vLast);
    topo.v_first = vFirst.IsNull() ? UINT32_MAX : ResolveUID(vFirst);
    topo.v_last  = vLast.IsNull()  ? UINT32_MAX : ResolveUID(vLast);

    Standard_Real first, last;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
    topo.t_first = first;
    topo.t_last  = last;
    world.EdgeTopos().Set(uid, topo);

    // Curve geometry
    if (!curve.IsNull())
        world.Curves().Set(uid, SerializeCurve(curve));
    else
        world.Curves().Set(uid, { Type::Empty, {} });
}

void WorldSender::SerializeFace(const TopoDS_Face& face, uint32_t uid, BRepWorld& world)
{
    world.RegisterEntity(uid);
    world.Types().Set(uid, Type::Face);
    world.Aabbs().Set(uid, ComputeAabb(face));
    world.Tolerances().Set(uid, { BRep_Tool::Tolerance(face) });

    // Surface geometry
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    world.Surfaces().Set(uid, SerializeSurface(surf));

    // Face topology: orientation + wires
    FaceTopoComp ftopo;
    ftopo.orientation = static_cast<uint8_t>(face.Orientation());

    TopoDS_Wire outer_wire = BRepTools::OuterWire(face);
    ftopo.has_outer_wire = !outer_wire.IsNull();
    if (ftopo.has_outer_wire)
    {
        auto wc = SerializeWire(outer_wire, face);
        ftopo.outer_wire_orientation = wc.orientation;
        ftopo.outer_wire_edges = std::move(wc.edges);
    }

    for (TopExp_Explorer exp(face, TopAbs_WIRE); exp.More(); exp.Next())
    {
        const TopoDS_Wire& w = TopoDS::Wire(exp.Current());
        if (!outer_wire.IsNull() && w.IsSame(outer_wire))
            continue;
        ftopo.inner_wires.push_back(SerializeWire(w, face));
    }

    world.FaceTopos().Set(uid, std::move(ftopo));
}

void WorldSender::SerializeSolid(const TopoDS_Solid& solid, uint32_t uid, BRepWorld& world)
{
    world.RegisterEntity(uid);
    world.Types().Set(uid, Type::Solid);
    world.Aabbs().Set(uid, ComputeAabb(solid));

    SolidTopoComp stopo;
    for (TopExp_Explorer exp(solid, TopAbs_SHELL); exp.More(); exp.Next())
    {
        const TopoDS_Shell& shell = TopoDS::Shell(exp.Current());
        SolidTopoComp::ShellComp sc;
        sc.orientation = static_cast<uint8_t>(shell.Orientation());

        for (TopExp_Explorer fexp(shell, TopAbs_FACE); fexp.More(); fexp.Next())
            sc.face_uids.push_back(ResolveUID(fexp.Current()));

        stopo.shells.push_back(std::move(sc));
    }

    world.SolidTopos().Set(uid, std::move(stopo));
}

uint32_t WorldSender::ResolveUID(const TopoDS_Shape& shape)
{
    if (m_tn)
    {
        uint32_t uid = GetUID(shape);
        if (uid != 0xffffffff)
            return uid;
    }

    int idx = m_auto_uid_map.FindIndex(shape);
    if (idx >= 1)
        return static_cast<uint32_t>(idx);

    return 0xffffffff;
}

uint32_t WorldSender::GetUID(const TopoDS_Shape& shape) const
{
    std::shared_ptr<breptopo::HistGraph> hg = nullptr;
    switch (shape.ShapeType())
    {
    case TopAbs_VERTEX: hg = m_tn->GetVertexGraph(); break;
    case TopAbs_EDGE:   hg = m_tn->GetEdgeGraph();   break;
    case TopAbs_FACE:   hg = m_tn->GetFaceGraph();   break;
    case TopAbs_SOLID:  hg = m_tn->GetSolidGraph();  break;
    default: break;
    }

    if (!hg)
        return 0xffffffff;

    auto node = hg->QueryNode(std::make_shared<partgraph::TopoShape>(shape));
    if (!node)
        return 0xffffffff;

    auto& cid = node->GetComponent<breptopo::NodeId>();
    return cid.GetUID();
}

CurveComp WorldSender::SerializeCurve(const Handle(Geom_Curve)& curve)
{
    CurveComp comp;

    if (curve->IsKind(STANDARD_TYPE(Geom_Line)))
    {
        comp.curve_type = Type::Line;
        auto g = Handle(Geom_Line)::DownCast(curve)->Lin();
        PushPoint(comp.data, g.Location());
        PushDir(comp.data, g.Direction());
    }
    else if (curve->IsKind(STANDARD_TYPE(Geom_Circle)))
    {
        comp.curve_type = Type::Circle;
        auto g = Handle(Geom_Circle)::DownCast(curve)->Circ();
        PushPoint(comp.data, g.Position().Location());
        PushDir(comp.data, g.Position().Direction());
        PushDir(comp.data, g.Position().XDirection());
        comp.data.push_back(g.Radius());
    }
    else if (curve->IsKind(STANDARD_TYPE(Geom_Ellipse)))
    {
        comp.curve_type = Type::Ellipse;
        auto g = Handle(Geom_Ellipse)::DownCast(curve)->Elips();
        PushPoint(comp.data, g.Position().Location());
        PushDir(comp.data, g.Position().Direction());
        PushDir(comp.data, g.Position().XDirection());
        comp.data.push_back(g.MajorRadius());
        comp.data.push_back(g.MinorRadius());
    }
    else if (curve->IsKind(STANDARD_TYPE(Geom_BSplineCurve)))
    {
        comp.curve_type = Type::BSplineCurve;
        auto g = Handle(Geom_BSplineCurve)::DownCast(curve);

        comp.data.push_back(g->Degree());
        comp.data.push_back(g->NbPoles());
        comp.data.push_back(g->NbKnots());
        comp.data.push_back(g->IsRational() ? 1.0 : 0.0);
        comp.data.push_back(g->IsPeriodic() ? 1.0 : 0.0);

        for (int i = 1; i <= g->NbPoles(); ++i)
            PushPoint(comp.data, g->Pole(i));
        if (g->IsRational())
            for (int i = 1; i <= g->NbPoles(); ++i)
                comp.data.push_back(g->Weight(i));
        for (int i = 1; i <= g->NbKnots(); ++i)
            comp.data.push_back(g->Knot(i));
        for (int i = 1; i <= g->NbKnots(); ++i)
            comp.data.push_back(static_cast<double>(g->Multiplicity(i)));
    }

    return comp;
}

Curve2dComp WorldSender::SerializeCurve2d(const Handle(Geom2d_Curve)& curve,
                                            double first, double last)
{
    Curve2dComp comp;
    comp.first = first;
    comp.last  = last;

    if (curve.IsNull())
    {
        comp.curve_type = Type::Empty;
        return comp;
    }

    if (curve->IsKind(STANDARD_TYPE(Geom2d_Line)))
    {
        comp.curve_type = Type::Line;
        auto g = Handle(Geom2d_Line)::DownCast(curve)->Lin2d();
        comp.data.push_back(g.Location().X());
        comp.data.push_back(g.Location().Y());
        comp.data.push_back(g.Direction().X());
        comp.data.push_back(g.Direction().Y());
    }
    else if (curve->IsKind(STANDARD_TYPE(Geom2d_Circle)))
    {
        comp.curve_type = Type::Circle;
        auto g = Handle(Geom2d_Circle)::DownCast(curve)->Circ2d();
        comp.data.push_back(g.Location().X());
        comp.data.push_back(g.Location().Y());
        comp.data.push_back(g.XAxis().Direction().X());
        comp.data.push_back(g.XAxis().Direction().Y());
        comp.data.push_back(g.Radius());
    }
    else if (curve->IsKind(STANDARD_TYPE(Geom2d_Ellipse)))
    {
        comp.curve_type = Type::Ellipse;
        auto g = Handle(Geom2d_Ellipse)::DownCast(curve)->Elips2d();
        comp.data.push_back(g.Location().X());
        comp.data.push_back(g.Location().Y());
        comp.data.push_back(g.XAxis().Direction().X());
        comp.data.push_back(g.XAxis().Direction().Y());
        comp.data.push_back(g.MajorRadius());
        comp.data.push_back(g.MinorRadius());
    }
    else if (curve->IsKind(STANDARD_TYPE(Geom2d_BSplineCurve)))
    {
        comp.curve_type = Type::BSplineCurve;
        auto g = Handle(Geom2d_BSplineCurve)::DownCast(curve);
        comp.data.push_back(g->Degree());
        comp.data.push_back(g->NbPoles());
        comp.data.push_back(g->NbKnots());
        comp.data.push_back(g->IsRational() ? 1.0 : 0.0);
        comp.data.push_back(g->IsPeriodic() ? 1.0 : 0.0);
        for (int i = 1; i <= g->NbPoles(); ++i)
        {
            comp.data.push_back(g->Pole(i).X());
            comp.data.push_back(g->Pole(i).Y());
        }
        if (g->IsRational())
            for (int i = 1; i <= g->NbPoles(); ++i)
                comp.data.push_back(g->Weight(i));
        for (int i = 1; i <= g->NbKnots(); ++i)
            comp.data.push_back(g->Knot(i));
        for (int i = 1; i <= g->NbKnots(); ++i)
            comp.data.push_back(static_cast<double>(g->Multiplicity(i)));
    }
    else
    {
        comp.curve_type = Type::Empty;
    }

    return comp;
}

SurfaceComp WorldSender::SerializeSurface(const Handle(Geom_Surface)& surf)
{
    SurfaceComp comp;

    if (surf->IsKind(STANDARD_TYPE(Geom_Plane)))
    {
        comp.surface_type = Type::Plane;
        gp_Pln g = Handle(Geom_Plane)::DownCast(surf)->Pln();
        PushPoint(comp.data, g.Location());
        PushDir(comp.data, g.Axis().Direction());
        PushDir(comp.data, g.XAxis().Direction());
    }
    else if (surf->IsKind(STANDARD_TYPE(Geom_CylindricalSurface)))
    {
        comp.surface_type = Type::Cylinder;
        gp_Cylinder g = Handle(Geom_CylindricalSurface)::DownCast(surf)->Cylinder();
        PushPoint(comp.data, g.Location());
        PushDir(comp.data, g.Axis().Direction());
        PushDir(comp.data, g.XAxis().Direction());
        comp.data.push_back(g.Radius());
    }
    else if (surf->IsKind(STANDARD_TYPE(Geom_SphericalSurface)))
    {
        comp.surface_type = Type::Sphere;
        gp_Sphere g = Handle(Geom_SphericalSurface)::DownCast(surf)->Sphere();
        PushPoint(comp.data, g.Location());
        PushDir(comp.data, g.Position().Direction());
        PushDir(comp.data, g.Position().XDirection());
        comp.data.push_back(g.Radius());
    }
    else if (surf->IsKind(STANDARD_TYPE(Geom_ToroidalSurface)))
    {
        comp.surface_type = Type::Torus;
        gp_Torus g = Handle(Geom_ToroidalSurface)::DownCast(surf)->Torus();
        PushPoint(comp.data, g.Location());
        PushDir(comp.data, g.Axis().Direction());
        PushDir(comp.data, g.XAxis().Direction());
        comp.data.push_back(g.MajorRadius());
        comp.data.push_back(g.MinorRadius());
    }
    else if (surf->IsKind(STANDARD_TYPE(Geom_ConicalSurface)))
    {
        comp.surface_type = Type::Cone;
        gp_Cone g = Handle(Geom_ConicalSurface)::DownCast(surf)->Cone();
        PushPoint(comp.data, g.Location());
        PushDir(comp.data, g.Axis().Direction());
        PushDir(comp.data, g.XAxis().Direction());
        comp.data.push_back(g.RefRadius());
        comp.data.push_back(g.SemiAngle());
    }
    else if (surf->IsKind(STANDARD_TYPE(Geom_BSplineSurface)))
    {
        comp.surface_type = Type::BSplineSurface;
        auto g = Handle(Geom_BSplineSurface)::DownCast(surf);

        comp.data.push_back(static_cast<double>(g->UDegree()));
        comp.data.push_back(static_cast<double>(g->VDegree()));
        comp.data.push_back(static_cast<double>(g->NbUPoles()));
        comp.data.push_back(static_cast<double>(g->NbVPoles()));
        comp.data.push_back(static_cast<double>(g->NbUKnots()));
        comp.data.push_back(static_cast<double>(g->NbVKnots()));
        comp.data.push_back(g->IsURational() ? 1.0 : 0.0);
        comp.data.push_back(g->IsVRational() ? 1.0 : 0.0);
        comp.data.push_back(g->IsUPeriodic() ? 1.0 : 0.0);
        comp.data.push_back(g->IsVPeriodic() ? 1.0 : 0.0);

        for (int u = 1; u <= g->NbUPoles(); ++u)
            for (int v = 1; v <= g->NbVPoles(); ++v)
                PushPoint(comp.data, g->Pole(u, v));

        if (g->IsURational() || g->IsVRational())
            for (int u = 1; u <= g->NbUPoles(); ++u)
                for (int v = 1; v <= g->NbVPoles(); ++v)
                    comp.data.push_back(g->Weight(u, v));

        for (int i = 1; i <= g->NbUKnots(); ++i)
            comp.data.push_back(g->UKnot(i));
        for (int i = 1; i <= g->NbUKnots(); ++i)
            comp.data.push_back(static_cast<double>(g->UMultiplicity(i)));

        for (int i = 1; i <= g->NbVKnots(); ++i)
            comp.data.push_back(g->VKnot(i));
        for (int i = 1; i <= g->NbVKnots(); ++i)
            comp.data.push_back(static_cast<double>(g->VMultiplicity(i)));
    }

    return comp;
}

FaceTopoComp::WireComp WorldSender::SerializeWire(const TopoDS_Wire& wire,
                                                    const TopoDS_Face& face)
{
    FaceTopoComp::WireComp wc;
    wc.orientation = static_cast<uint8_t>(wire.Orientation());

    for (TopExp_Explorer exp(wire, TopAbs_EDGE); exp.More(); exp.Next())
    {
        const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());

        FaceTopoComp::WireEdgeRef ref;
        ref.edge_uid    = ResolveUID(edge);
        ref.orientation = static_cast<uint8_t>(edge.Orientation());

        TopoDS_Edge fwd_edge = TopoDS::Edge(edge.Oriented(TopAbs_FORWARD));
        Standard_Real pcf, pcl;
        Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(fwd_edge, face, pcf, pcl);
        ref.pcurve = SerializeCurve2d(pc, pcf, pcl);

        wc.edges.push_back(std::move(ref));
    }

    return wc;
}

} // namespace brepdb
