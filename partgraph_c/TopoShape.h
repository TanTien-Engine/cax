#pragma once

#include <TopoDS_Shape.hxx>

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

private:
	TopoDS_Shape m_shape;

}; // TopoShape

}