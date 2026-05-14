#pragma once

#include "brepdb_c/TypedPool.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>

#include <Geom_Curve.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Surface.hxx>

#include <unordered_map>

namespace brepdb
{

class WorldReceiver
{
public:
    WorldReceiver(const BRepWorld& world);

    TopoDS_Shape GetShape(uint32_t uid);
    TopoDS_Shape GetAll();

    const std::unordered_map<uint32_t, TopoDS_Shape>& GetCache() const { return m_cache; }

private:
    TopoDS_Vertex DeserializeVertex(uint32_t uid);
    TopoDS_Edge   DeserializeEdge(uint32_t uid);
    TopoDS_Face   DeserializeFace(uint32_t uid);
    TopoDS_Solid  DeserializeSolid(uint32_t uid);

    TopoDS_Wire   DeserializeWire(const FaceTopoComp::WireEdgeRef* edges, size_t count,
                                  uint8_t orientation, const TopoDS_Face& face);
    TopoDS_Shell  DeserializeShell(const SolidTopoComp::ShellComp& shell);

    Handle(Geom_Curve)   DeserializeCurve(const CurveComp& comp);
    Handle(Geom2d_Curve) DeserializeCurve2d(const Curve2dComp& comp);
    Handle(Geom_Surface) DeserializeSurface(const SurfaceComp& comp);

private:
    const BRepWorld& m_world;
    std::unordered_map<uint32_t, TopoDS_Shape> m_cache;

}; // WorldReceiver

}
