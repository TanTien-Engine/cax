#pragma once

#include "brepdb_c/TypedPool.h"

#include <Geom_Curve.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Surface.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

namespace breptopo { class TopoNaming; }

namespace brepdb
{

class WorldSender
{
public:
    WorldSender(const std::shared_ptr<breptopo::TopoNaming>& tn);

    void Serialize(const TopoDS_Shape& shape, BRepWorld& world);

    void SerializeVertex(const TopoDS_Vertex& vertex, uint32_t uid, BRepWorld& world);
    void SerializeEdge(const TopoDS_Edge& edge, uint32_t uid, BRepWorld& world);
    void SerializeFace(const TopoDS_Face& face, uint32_t uid, BRepWorld& world);
    void SerializeSolid(const TopoDS_Solid& solid, uint32_t uid, BRepWorld& world);

    uint32_t GetUID(const TopoDS_Shape& shape) const;
    uint32_t ResolveUID(const TopoDS_Shape& shape);

private:
    CurveComp   SerializeCurve(const Handle(Geom_Curve)& curve);
    Curve2dComp SerializeCurve2d(const Handle(Geom2d_Curve)& curve,
                                  double first, double last);
    SurfaceComp SerializeSurface(const Handle(Geom_Surface)& surf);

    FaceTopoComp::WireComp SerializeWire(const TopoDS_Wire& wire,
                                          const TopoDS_Face& face);

private:
    std::shared_ptr<breptopo::TopoNaming> m_tn;

    uint32_t m_next_auto_uid = 1;
    TopTools_IndexedMapOfShape m_auto_uid_map;

}; // WorldSender

} // namespace brepdb
