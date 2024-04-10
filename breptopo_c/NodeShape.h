#pragma once

#include <objcomp/Component.h>

namespace partgraph { class TopoShape; }

namespace breptopo
{

class NodeShape : public objcomp::Component
{
public:
	NodeShape(const std::shared_ptr<partgraph::TopoShape>& shape)
		: m_shape(shape) {}

	virtual const char* Type() const override { return "node_shp"; }
	virtual objcomp::CompID TypeID() const override { return objcomp::GetCompTypeID<NodeShape>(); }
	virtual NodeShape* Clone() const override { return nullptr; }

	auto GetShape() const { return m_shape; }
	void SetShape(const std::shared_ptr<partgraph::TopoShape>& shape) { m_shape = shape; }

private:
	std::shared_ptr<partgraph::TopoShape> m_shape;

}; // NodeShape

}