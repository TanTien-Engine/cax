#include "TopoShape.h"

#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>

namespace partgraph
{

const TopoDS_Edge& TopoShape::ToEdge() const
{
	if (m_shape.ShapeType() == TopAbs_EDGE)
	{
		return static_cast<const TopoDS_Edge&>(m_shape);
	}
	else
	{
		throw std::runtime_error("Not edge!");
	}
}

const TopoDS_Wire& TopoShape::ToWire() const
{
	if (m_shape.ShapeType() == TopAbs_WIRE)
	{
		return static_cast<const TopoDS_Wire&>(m_shape);
	}
	else
	{
		throw std::runtime_error("Not wire!");
	}
}

}