#pragma once

#include <memory>
#include <vector>

namespace gs { class Arc; class Line3D; }

namespace partgraph
{

class TopoEdge;
class TopoWire;
class TopoFace;

class BRepBuilder
{
public:
	static std::shared_ptr<TopoEdge> MakeEdge(const gs::Line3D& l);
	static std::shared_ptr<TopoEdge> MakeEdge(const gs::Arc& arc);
	static std::shared_ptr<TopoWire> MakeWire(const std::vector<std::shared_ptr<TopoEdge>>& edges);
	static std::shared_ptr<TopoFace> MakeFace(const TopoWire& wire);

}; // BRepBuilder

}