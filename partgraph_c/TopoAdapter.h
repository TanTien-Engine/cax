#pragma once

#include <memory>

namespace ur { class VertexArray; }
namespace gs { class Line3D; }

namespace partgraph
{

class TopoShape;
class TopoEdge;

class TopoAdapter
{
public:
	static std::shared_ptr<ur::VertexArray> BuildMesh(const TopoShape& shape);

	static std::shared_ptr<gs::Line3D> BuildGeo(const TopoEdge& edge);

}; // TopoAdapter

}