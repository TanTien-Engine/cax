#pragma once

#include "brepgraph_c/history/HistGraph.h"

#include <Standard_Handle.hxx>

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

class BRepBuilderAPI_MakeShape;
class BRepTools_History;
class BRepOffset_MakeSimpleOffset;

namespace brepkit { class TopoShape; }

class TopoDS_Shape;

namespace brepgraph
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

	PidMap Update(const brepkit::TopoShape& new_shape, uint32_t op_id);
	PidMap Update(BRepBuilderAPI_MakeShape& builder, const brepkit::TopoShape& new_shape,
		const brepkit::TopoShape& old_shape, uint32_t op_id);
	PidMap Update(opencascade::handle<BRepTools_History> hist, const brepkit::TopoShape& new_shape,
		const brepkit::TopoShape& old_shape, uint32_t op_id);
	PidMap Update(const BRepOffset_MakeSimpleOffset& builder, const brepkit::TopoShape& old_shape, uint32_t op_id);


	auto GetVertexGraph() const { return m_vertex_hg; }
	auto GetEdgeGraph() const { return m_edge_hg; }
	auto GetFaceGraph() const { return m_face_hg; }
	auto GetSolidGraph() const { return m_solid_hg; }

	void MergeFrom(const TopoNaming& other);

	// Parallel-fork primitives. RunParallel uses these to give each fork
	// its own clone so concurrent HistGraph::Update calls don't race on
	// the shared unordered_map, then absorbs each fork's additions back
	// into the main TN at the join.
	struct Snapshot;
	std::shared_ptr<TopoNaming> Clone() const;
	Snapshot                    TakeSnapshot() const;
	void                        AbsorbFork(const TopoNaming& fork, const Snapshot& base);

	void BindShape(uint32_t uid, const TopoDS_Shape& shape);
	void BindShapes(const std::unordered_map<uint32_t, TopoDS_Shape>& uid2shape);

	void StoreToByteArray(uint8_t** buf, uint32_t& len) const;
	bool LoadFromByteArray(const uint8_t* buf, uint32_t len);

	uint32_t NextOpId();

private:
	std::shared_ptr<HistGraph> m_vertex_hg;
	std::shared_ptr<HistGraph> m_edge_hg;
	std::shared_ptr<HistGraph> m_face_hg;
	std::shared_ptr<HistGraph> m_solid_hg;

	uint32_t m_next_op = 0;

}; // TopoNaming

struct TopoNaming::Snapshot
{
	HistGraph::Snapshot vertex;
	HistGraph::Snapshot edge;
	HistGraph::Snapshot face;
	HistGraph::Snapshot solid;
	uint32_t            next_op = 0;
};

}
