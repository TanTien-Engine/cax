#pragma once

#include <objcomp/Component.h>

#include <vector>

namespace brepkit { class TopoShape; }

namespace brepgraph
{

// Shape component used in two cardinalities:
//   - per-node: attach to a graph node, holds a single TopoShape
//                  -> use NodeShape(shape) + GetShape()/SetShape()
//   - graph-level: attach to the whole graph, holds a list of TopoShapes
//                  -> use NodeShape(shapes) + GetShapes()
class NodeShape : public objcomp::Component
{
public:
	NodeShape(const std::shared_ptr<brepkit::TopoShape>& shape)
		: m_shapes{shape} {}

	NodeShape(const std::vector<std::shared_ptr<brepkit::TopoShape>>& shapes)
		: m_shapes(shapes) {}

	virtual const char* Type() const override { return "node_shp"; }
	virtual objcomp::CompID TypeID() const override { return objcomp::GetCompTypeID<NodeShape>(); }
	virtual NodeShape* Clone() const override { return nullptr; }

	// Single-shape accessors (returns the first shape; meaningful when the
	// component holds exactly one).
	auto GetShape() const { return m_shapes.empty() ? nullptr : m_shapes.front(); }
	void SetShape(const std::shared_ptr<brepkit::TopoShape>& shape) { m_shapes = { shape }; }

	// Multi-shape accessor.
	const auto& GetShapes() const { return m_shapes; }

private:
	std::vector<std::shared_ptr<brepkit::TopoShape>> m_shapes;

}; // NodeShape

}
