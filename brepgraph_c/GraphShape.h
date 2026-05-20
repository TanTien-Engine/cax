#pragma once

#include <objcomp/Component.h>

#include <vector>

namespace brepkit { class TopoShape; }

namespace brepgraph
{

class GraphShape : public objcomp::Component
{
public:
	GraphShape(const std::vector<std::shared_ptr<brepkit::TopoShape>>& shapes)
		: m_shapes(shapes) {}

	virtual const char* Type() const override { return "topo_graph_shp"; }
	virtual objcomp::CompID TypeID() const override { return objcomp::GetCompTypeID<GraphShape>(); }
	virtual GraphShape* Clone() const override { return nullptr; }

private:
	std::vector<std::shared_ptr<brepkit::TopoShape>> m_shapes;

}; // GraphShape

}