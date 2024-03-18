#pragma once

#include <objcomp/Component.h>

#include <vector>

namespace partgraph { class TopoShape; }

namespace breptopo
{

class TopoGraphShape : public objcomp::Component
{
public:
	TopoGraphShape(const std::vector<std::shared_ptr<partgraph::TopoShape>>& shapes)
		: m_shapes(shapes) {}

	virtual const char* Type() const override { return "topo_graph_shp"; }
	virtual objcomp::CompID TypeID() const override { return objcomp::GetCompTypeID<TopoGraphShape>(); }
	virtual TopoGraphShape* Clone() const override { return nullptr; }

private:
	std::vector<std::shared_ptr<partgraph::TopoShape>> m_shapes;

}; // TopoGraphShape

}