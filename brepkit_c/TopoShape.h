#pragma once

#include <TopoDS_Shape.hxx>

#include <cstdint>

class TopoDS_Edge;
class TopoDS_Wire;

namespace brepkit
{

class TopoShape
{
public:
	static constexpr uint32_t NO_VERSION = UINT32_MAX;

	TopoShape() {}
	TopoShape(const TopoDS_Shape& shape)
		: m_shape(shape)
	{
	}

	auto& GetShape() const { return m_shape; }

	uint32_t GetVersionId() const { return m_version_id; }
	void     SetVersionId(uint32_t id) { m_version_id = id; }

	const TopoDS_Edge& ToEdge() const;
	const TopoDS_Wire& ToWire() const;

private:
	TopoDS_Shape m_shape;
	uint32_t     m_version_id = NO_VERSION;

}; // TopoShape

}