#pragma once

#include <objcomp/Component.h>

namespace partgraph { class TopoFace; }

namespace breptopo
{

class NodeShape : public objcomp::Component
{
public:
	NodeShape(const std::shared_ptr<partgraph::TopoFace>& face)
		: m_face(face) {}

	virtual const char* Type() const override { return "node_shp"; }
	virtual objcomp::CompID TypeID() const override { return objcomp::GetCompTypeID<NodeShape>(); }
	virtual NodeShape* Clone() const override { return nullptr; }

	auto GetFace() const { return m_face; }

private:
	std::shared_ptr<partgraph::TopoFace> m_face;

}; // NodeShape

}