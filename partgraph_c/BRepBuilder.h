#pragma once

#include "Standard_Handle.hxx"

#include <memory>
#include <vector>

namespace gs { class Arc3D; class Line3D; }

namespace partgraph
{

class TopoEdge;
class TopoWire;
class TopoFace;
class TopoShape;

class TrimmedCurve;
class CylindricalSurface;

class BRepBuilder
{
public:
	static std::shared_ptr<TopoEdge> MakeEdge(const gs::Line3D& l);
	static std::shared_ptr<TopoEdge> MakeEdge(const gs::Arc3D& arc);
	static std::shared_ptr<TopoEdge> MakeEdge(const TrimmedCurve& c, const CylindricalSurface& s);
	static std::shared_ptr<TopoWire> MakeWire(const std::vector<std::shared_ptr<TopoEdge>>& edges);
	static std::shared_ptr<TopoFace> MakeFace(const TopoWire& wire);
	static std::shared_ptr<TopoShape> MakeCompound(const std::vector<std::shared_ptr<TopoShape>>& shapes);

}; // BRepBuilder

}