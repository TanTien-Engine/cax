#pragma once

#include <SM_Vector.h>

#include <memory>
#include <vector>

namespace ur { class Device; class VertexArray; }
namespace gs { class Line3D; class Polyline3D; }

class TopoDS_Shape;

namespace brepkit
{

class TopoShape;

class TopoAdapter
{
public:
	// alpha is baked into every surface vertex (attribute location 2)
	// so the GBuffer shader can emit per-part transparency without a
	// uniform. 1.0 = opaque (default); a transparent part passes
	// 1 - MaterialIR.transparency.
	static std::shared_ptr<ur::VertexArray> BuildMeshFromShape(const std::shared_ptr<ur::Device>& dev, const TopoShape& shape, float alpha = 1.0f);
	static std::shared_ptr<ur::VertexArray> BuildMeshFromShell(const std::shared_ptr<ur::Device>& dev, const TopoShape& shell, float alpha = 1.0f);
	// alpha is baked into every edge vertex (same attr location 2 as the
	// surface mesh) so transparent parts can draw their topological edges
	// at the same opacity as their faces. 1.0 = opaque (default).
	static std::shared_ptr<ur::VertexArray> BuildEdgesFromShape(const std::shared_ptr<ur::Device>& dev, const TopoShape& shape, float alpha = 1.0f);
	static std::shared_ptr<gs::Line3D> BuildGeoFromEdge(const TopoShape& edge);
	static std::shared_ptr<gs::Polyline3D> BuildGeoFromWire(const TopoShape& wire);

	static std::shared_ptr<TopoShape> ToWire(const TopoShape& shape);

private:
	static std::shared_ptr<ur::VertexArray> BuildMesh(const std::shared_ptr<ur::Device>& dev, const TopoDS_Shape& shape, float alpha = 1.0f);

	struct Vertex
	{
		sm::vec3 pos;
		sm::vec3 normal;
		float    alpha = 1.0f;   // per-vertex opacity, attr location 2
		//sm::vec2 texcoord;
	};
	static void TriangulationFaces(const TopoDS_Shape& shape, std::vector<Vertex>& vertices, float alpha = 1.0f);
	static void TriangulationEdges(const TopoDS_Shape& shape, std::vector<Vertex>& vertices);

}; // TopoAdapter

}