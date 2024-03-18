#pragma once

#include <objcomp/Component.h>

namespace partgraph { class TopoFace; }

namespace breptopo
{

class TopoNodeShape : public objcomp::Component
{
public:
	TopoNodeShape(const std::shared_ptr<partgraph::TopoFace>& face)
		: m_face(face) {}

	virtual const char* Type() const override { return "topo_node_shp"; }
	virtual objcomp::CompID TypeID() const override { return objcomp::GetCompTypeID<TopoNodeShape>(); }
	virtual TopoNodeShape* Clone() const override { return nullptr; }

	auto GetFace() const { return m_face; }

private:
	std::shared_ptr<partgraph::TopoFace> m_face;

}; // TopoNodeShape

}