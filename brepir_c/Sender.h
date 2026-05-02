#pragma once

#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>

namespace breptopo { class TopoNaming; }

namespace brepir
{

class GeometryPool;

class Sender
{
public:
    Sender(const std::shared_ptr<breptopo::TopoNaming>& tn);

    void SerializeVertex(const TopoDS_Vertex& vertex, uint32_t uid, GeometryPool& pool);
    void SerializeEdge(const TopoDS_Edge& edge, uint32_t uid, GeometryPool& pool);
    void SerializeFace(const TopoDS_Face& face, uint32_t uid, GeometryPool& pool);
    void SerializeSolid(const TopoDS_Solid& solid, uint32_t uid, GeometryPool& pool);

    uint32_t GetUID(const TopoDS_Shape& shape) const;

private:
    void SerializeCurve(const Handle(Geom_Curve)& curve, GeometryPool& pool);
    void SerializeSurface(const Handle(Geom_Surface)& surf, GeometryPool& pool);

    void SerializeShell(const TopoDS_Shell& shell, GeometryPool& pool);
    void SerializeWire(const TopoDS_Wire& wire, GeometryPool& pool);

private:
    std::shared_ptr<breptopo::TopoNaming> m_tn;

}; // Sender

}