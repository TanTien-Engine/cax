#pragma once

#include <memory>

namespace ur { class VertexArray; }
namespace gs { class Line3D; class Polyline3D; }

namespace partgraph
{

class TopoShape;
class TopoEdge;
class TopoWire;

class TopoAdapter
{
public:
	static std::shared_ptr<ur::VertexArray> BuildMesh(const TopoShape& shape);
	static std::shared_ptr<gs::Line3D> BuildGeo(const TopoEdge& edge);
	static std::shared_ptr<gs::Polyline3D> BuildGeo(const TopoWire& wire);

	static std::shared_ptr<TopoWire> ToWire(const TopoShape& shape);

}; // TopoAdapter

}