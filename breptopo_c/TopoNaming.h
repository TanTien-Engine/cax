#pragma once

#include <Standard_Handle.hxx>

#include <memory>

class BRepBuilderAPI_MakeShape;
class BRepTools_History;
class BRepOffset_MakeSimpleOffset;

namespace partgraph { class TopoShape; }

namespace breptopo
{

class HistGraph;

class TopoNaming
{
public:
	TopoNaming();

	void Update(const partgraph::TopoShape& new_shape, uint32_t op_id);
	void Update(BRepBuilderAPI_MakeShape& builder, const partgraph::TopoShape& new_shape, 
		const partgraph::TopoShape& old_shape, uint32_t op_id);
	void Update(opencascade::handle<BRepTools_History> hist, const partgraph::TopoShape& new_shape,
		const partgraph::TopoShape& old_shape, uint32_t op_id);
	void Update(const BRepOffset_MakeSimpleOffset& builder, const partgraph::TopoShape& old_shape, uint32_t op_id);

	auto GetVertexGraph() const { return m_vertex_hg; }
	auto GetEdgeGraph() const { return m_edge_hg; }
	auto GetFaceGraph() const { return m_face_hg; }
	auto GetSolidGraph() const { return m_solid_hg; }

	uint32_t NextOpId();

private:
	std::shared_ptr<HistGraph> m_vertex_hg;
	std::shared_ptr<HistGraph> m_edge_hg;
	std::shared_ptr<HistGraph> m_face_hg;
	std::shared_ptr<HistGraph> m_solid_hg;

	uint32_t m_next_op = 0;

}; // TopoNaming

}