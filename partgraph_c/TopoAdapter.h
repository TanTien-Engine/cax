#pragma once

#include <memory>

namespace ur { class VertexArray; }
namespace gs { class Line3D; class Polyline3D; }

class TopoDS_Shape;

namespace partgraph
{

class TopoShape;
class TopoEdge;
class TopoWire;
class TopoShell;

class TopoAdapter
{
public:
	static std::shared_ptr<ur::VertexArray> BuildMesh(const TopoShape& shape);
	static std::shared_ptr<ur::VertexArray> BuildMesh(const TopoShell& shell);
	static std::shared_ptr<gs::Line3D> BuildGeo(const TopoEdge& edge);
	static std::shared_ptr<gs::Polyline3D> BuildGeo(const TopoWire& wire);

	static std::shared_ptr<TopoWire> ToWire(const TopoShape& shape);

private:
	static std::shared_ptr<ur::VertexArray> BuildMesh(const TopoDS_Shape& shape);

}; // TopoAdapter

}