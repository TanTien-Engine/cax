#pragma once

#include <TopoDS_Shape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <NCollection_DataMap.hxx>
#include <TopTools_ShapeMapHasher.hxx>

#include <memory>
#include <vector>
#include <unordered_map>
#include <map>

namespace partgraph { class TopoShape; class BRepHistory; }
namespace graph { class Graph; class Node; }

namespace breptopo
{

// ---------------------------------------------------------------
//  HistGraph
//
//  Tracks the lineage of sub-shape UIDs across modeling operations.
//  Conceptually three maps:
//
//    m_op2uids : op_id -> UIDs that op produced (latest run)
//    m_forward : uid   -> direct successor UIDs (split / transform target)
//    m_backward: uid   -> direct predecessor UIDs
//
//  All persistent. Plus two transient bind maps (NOT serialized):
//
//    m_uid2shape : uid -> current TopoDS_Shape (alive uids only)
//    m_shape2uid : shape -> uid (mirror of m_uid2shape, for sender lookup)
//
//  After deserialization the bind maps are empty. They get repopulated
//  either by WarmUpFromVT (in CompGraph) or lazily as ops re-run.
//
//  The visualization-only graph::Graph is rebuilt on demand by
//  BuildDebugGraph(); it is never stored as ground-truth.
// ---------------------------------------------------------------

class HistGraph
{
public:
	// Per-type partial pid mapping produced by an Update:
	//   old uid -> new uid(s); empty vector means deletion.
	using PartialPidMap = std::map<uint32_t, std::vector<uint32_t>>;

	// Snapshot captured before a parallel fork -- used by AbsorbFork to
	// integrate the fork's additions back into this graph.
	struct Snapshot
	{
		std::unordered_map<uint32_t, std::vector<uint32_t>> op2uids;
		std::unordered_map<uint32_t, std::vector<uint32_t>> forward;
	};

	HistGraph();

	// Apply an OCC modeling step to the graph. type_id is one of TopAbs_*.
	// Returns the old-uid -> new-uids map (used to build a per-op pid map).
	PartialPidMap Update(const partgraph::BRepHistory& hist,
	                     uint32_t type_id, uint32_t op_id);

	// Lookups -----------------------------------------------------

	// Return the UID assigned to `shape` (looked up via m_shape2uid).
	// 0xFFFFFFFF if unknown.
	uint32_t GetUID(const std::shared_ptr<partgraph::TopoShape>& shape) const;
	uint32_t GetUID(const TopoDS_Shape& shape) const;

	// Resolve uid to the current live TopoDS_Shape. If `uid` itself was
	// consumed by a later op (no entry in m_uid2shape), follow m_forward
	// BFS to find live descendants. Caller-facing version returns
	// partgraph::TopoShape pointers for compatibility with old callers.
	std::vector<std::shared_ptr<partgraph::TopoShape>>
	QueryCurrentShapes(uint32_t uid) const;
	bool QueryCurrentShapes(uint32_t uid,
	                        std::vector<std::shared_ptr<partgraph::TopoShape>>& out) const;

	// Compatibility wrappers around the new lookups (return synthetic
	// graph::Node objects so existing wrap code keeps compiling).
	std::shared_ptr<graph::Node> QueryNode(
		const std::shared_ptr<partgraph::TopoShape>& shape) const;
	bool QueryNodes(uint32_t uid,
	                std::vector<std::shared_ptr<graph::Node>>& results) const;

	// "Did this UID ever exist in any tracked op?"  True for both alive and
	// consumed (inactive) UIDs -- historical record never gets pruned.
	bool   IsKnown(uint32_t uid)   const;
	// "Is this UID's geometry currently bound (still alive in the model)?"
	bool   IsActive(uint32_t uid)  const;
	// Direct successors / predecessors in the lineage DAG. Useful for UI
	// navigation of historical (now inactive) UIDs.
	const std::vector<uint32_t>* Successors(uint32_t uid)   const;
	const std::vector<uint32_t>* Predecessors(uint32_t uid) const;

	// (re)Bind a uid to a fresh OCC shape (used by VT restore).
	void BindShape(uint32_t uid, const TopoDS_Shape& shape);

	// Merge another HistGraph's state in (used by MergeFrom and the
	// parallel fork AbsorbFork; uid space is global so a union suffices).
	void MergeFrom(const HistGraph& other);

	// Deep-copy entire (persistent + transient) state for a parallel fork.
	std::shared_ptr<HistGraph> Clone() const;

	// Capture sizes for AbsorbFork. With uid-keyed maps, the snapshot is
	// just the keysets at fork time so absorb can identify additions.
	Snapshot TakeSnapshot() const;
	void     AbsorbFork(const HistGraph& fork, const Snapshot& base);

	// On-demand debug visualization. Each call returns a fresh Graph with
	// one node per uid, edges for m_forward; meant for UI inspection.
	std::shared_ptr<graph::Graph> GetGraph() const;

	// Persistence -------------------------------------------------

	void StoreToByteArray(uint8_t** buf, uint32_t& len) const;
	bool LoadFromByteArray(const uint8_t* buf, uint32_t len);

	// UID codec ---------------------------------------------------
	// Layout: [type:3 | op:14 | index:15]. type is a TopAbs_ShapeEnum value.
	// All bit twiddling lives here -- callers must not decode uids by hand.

	static uint32_t CalcUID(uint32_t type_id, uint32_t op_id, uint32_t index);
	static uint32_t TypeOf (uint32_t uid);
	static uint32_t OpOf   (uint32_t uid);
	static uint32_t IndexOf(uint32_t uid);

	// Raw accessors (debug / test) --------------------------------

	const std::unordered_map<uint32_t, std::vector<uint32_t>>& Op2Uids()  const { return m_op2uids; }
	const std::unordered_map<uint32_t, std::vector<uint32_t>>& Forward()  const { return m_forward; }
	const std::unordered_map<uint32_t, std::vector<uint32_t>>& Backward() const { return m_backward; }
	const std::unordered_map<uint32_t, TopoDS_Shape>&          Uid2Shape() const { return m_uid2shape; }

private:

	// Persistent state -------------------------------------------

	std::unordered_map<uint32_t, std::vector<uint32_t>> m_op2uids;
	std::unordered_map<uint32_t, std::vector<uint32_t>> m_forward;
	std::unordered_map<uint32_t, std::vector<uint32_t>> m_backward;

	// Transient state --------------------------------------------

	std::unordered_map<uint32_t, TopoDS_Shape> m_uid2shape;
	NCollection_DataMap<TopoDS_Shape, uint32_t, TopTools_ShapeMapHasher> m_shape2uid;

}; // HistGraph

}  // namespace breptopo
