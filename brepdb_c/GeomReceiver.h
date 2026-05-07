#pragma once

#include "brepdb_c/GeomPool.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>

#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>

#include <unordered_map>

namespace brepdb
{

class GeomReceiver
{
public:
    GeomReceiver(const GeometryPool& pool);

    TopoDS_Shape GetShape(uint32_t uid);

private:
    TopoDS_Vertex DeserializeVertex(uint32_t& offset);
    TopoDS_Edge   DeserializeEdge(uint32_t& offset);
    TopoDS_Face   DeserializeFace(uint32_t& offset);
    TopoDS_Solid  DeserializeSolid(uint32_t& offset);

    TopoDS_Wire   DeserializeWire(uint32_t& offset);
    TopoDS_Shell  DeserializeShell(uint32_t& offset);

    Handle(Geom_Curve)   DeserializeCurve(uint32_t& offset);
    Handle(Geom_Surface) DeserializeSurface(uint32_t& offset);

private:
    const GeometryPool& m_pool;
    
    std::unordered_map<uint32_t, const GeomHeader*> m_header_map;
    
    std::unordered_map<uint32_t, TopoDS_Shape> m_rebuilt_shapes;

}; // GeomReceiver

}