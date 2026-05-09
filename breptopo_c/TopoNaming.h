#pragma once

#include <Standard_Handle.hxx>

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

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
	// Combined per-op pid mapping across all 4 sub-shape types
	// (vertex/edge/face/solid). Plugs straight into
	// brepdb::VersionTree::BuildDiffFromPidMapping.
	using PidMap = std::map<uint32_t, std::vector<uint32_t>>;

	TopoNaming();

	PidMap Update(const partgraph::TopoShape& new_shape, uint32_t op_id);
	PidMap Update(BRepBuilderAPI_MakeShape& builder, const partgraph::TopoShape& new_shape,
		const partgraph::TopoShape& old_shape, uint32_t op_id);
	PidMap Update(opencascade::handle<BRepTools_History> hist, const partgraph::TopoShape& new_shape,
		const partgraph::TopoShape& old_shape, uint32_t op_id);
	PidMap Update(const BRepOffset_MakeSimpleOffset& builder, const partgraph::TopoShape& old_shape, uint32_t op_id);

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
