#pragma once

#include <TopoDS_Shape.hxx>

class TopoDS_Edge;
class TopoDS_Wire;

namespace partgraph
{

class TopoShape
{
public:
	TopoShape() {}
	TopoShape(const TopoDS_Shape& shape)
		: m_shape(shape)
	{
	}

	auto& GetShape() const { return m_shape; }

	const TopoDS_Edge& ToEdge() const;
	const TopoDS_Wire& ToWire() const;

private:
	TopoDS_Shape m_shape;

}; // TopoShape

}