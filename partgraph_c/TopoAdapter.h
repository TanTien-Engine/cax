#pragma once

#include <memory>

namespace ur { class VertexArray; }
namespace gs { class Line3D; class Polyline3D; }

class TopoDS_Shape;

namespace partgraph
{

class TopoShape;

class TopoAdapter
{
public:
	static std::shared_ptr<ur::VertexArray> BuildMeshFromShape(const TopoShape& shape);
	static std::shared_ptr<ur::VertexArray> BuildMeshFromShell(const TopoShape& shell);
	static std::shared_ptr<gs::Line3D> BuildGeoFromEdge(const TopoShape& edge);
	static std::shared_ptr<gs::Polyline3D> BuildGeoFromWire(const TopoShape& wire);

	static std::shared_ptr<TopoShape> ToWire(const TopoShape& shape);

private:
	static std::shared_ptr<ur::VertexArray> BuildMesh(const TopoDS_Shape& shape);

}; // TopoAdapter

}