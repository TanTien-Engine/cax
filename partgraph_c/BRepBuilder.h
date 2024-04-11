#pragma once

#include "Standard_Handle.hxx"

#include <memory>
#include <vector>

namespace gs { class Arc3D; class Line3D; }

namespace partgraph
{

class TopoShape;

class TrimmedCurve;
class CylindricalSurface;

class BRepBuilder
{
public:
	static std::shared_ptr<TopoShape> MakeEdge(const gs::Line3D& l);
	static std::shared_ptr<TopoShape> MakeEdge(const gs::Arc3D& arc);
	static std::shared_ptr<TopoShape> MakeEdge(const TrimmedCurve& c, const CylindricalSurface& s);
	static std::shared_ptr<TopoShape> MakeWire(const std::vector<std::shared_ptr<TopoShape>>& edges);
	static std::shared_ptr<TopoShape> MakeFace(const TopoShape& wire);
	static std::shared_ptr<TopoShape> MakeShell(const std::vector<std::shared_ptr<TopoShape>>& faces);
	static std::shared_ptr<TopoShape> MakeCompound(const std::vector<std::shared_ptr<TopoShape>>& shapes);

}; // BRepBuilder

}