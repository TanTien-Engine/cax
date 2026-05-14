#include "GlobalConfig.h"
#include "TopoShape.h"

#include <breptopo_c/TopoNaming.h>
#include <breptopo_c/CompGraph.h>
#include <brepdb_c/VersionTree.h>
#include <brepdb_c/WorldSender.h>
#include <brepdb_c/WorldReceiver.h>

#include <mutex>

namespace partgraph
{

GlobalConfig* GlobalConfig::m_instance = nullptr;

GlobalConfig* GlobalConfig::Instance()
{
	if (!m_instance) {
		m_instance = new GlobalConfig();
	}
	return m_instance;
}

GlobalConfig::GlobalConfig()
{
	m_topo_naming = std::make_shared<breptopo::TopoNaming>();
	m_version_tree = std::make_shared<brepdb::VersionTree>();
	m_comp_graph = std::make_shared<breptopo::CompGraph>();

	// Share the same TopoNaming so direct .ves code and CompGraph::Eval
	// see the same naming state.
	m_comp_graph->SetTopoNaming(m_topo_naming);

	auto vt = m_version_tree;
	auto cg = m_comp_graph;
	auto tn = m_topo_naming;
	auto vt_mutex = std::make_shared<std::mutex>();

	cg->SetCommitFn(
		[vt, cg, tn, vt_mutex](uint32_t /*nref_id*/, const breptopo::Val& val) -> uint32_t
		{
			if (!std::holds_alternative<breptopo::ShapeVal>(val)) return UINT32_MAX;
			const auto& sv = std::get<breptopo::ShapeVal>(val);
			if (!sv.shape) return UINT32_MAX;

			auto cg_tn = cg->GetTopoNaming();
			auto eff_tn = cg_tn ? cg_tn : tn;

			brepdb::BRepWorld world;
			brepdb::WorldSender sender(eff_tn);
			sender.Serialize(sv.shape->GetShape(), world);

			std::lock_guard<std::mutex> lk(*vt_mutex);
			return vt->AddRoot(world, "");
		});

	cg->SetRestoreFn(
		[vt, vt_mutex](uint32_t vt_node_id,
		               const std::shared_ptr<breptopo::TopoNaming>& curr_tn) -> breptopo::Val
		{
			if (vt_node_id == UINT32_MAX) return {};
			std::lock_guard<std::mutex> lk(*vt_mutex);
			uint32_t root_id = vt->FindRootOf(vt_node_id);
			if (root_id == UINT32_MAX) return {};
			auto world = vt->Checkout(root_id, vt_node_id);
			if (!world) return {};

			brepdb::WorldReceiver receiver(*world);
			TopoDS_Shape shape = receiver.GetAll();
			if (shape.IsNull()) return {};

			// Bind to the TopoNaming currently in use by the evaluator (could be
			// a parallel fork's tn). This ensures selector ops running in the
			// same evaluator see the sub-shapes of the restored solid.
			if (curr_tn) {
				for (auto& kv : receiver.GetCache())
					curr_tn->BindShape(kv.first, kv.second);
			}

			auto topo = std::make_shared<partgraph::TopoShape>(shape);
			return breptopo::ShapeVal{topo, 0};
		});
}

GlobalConfig::~GlobalConfig()
{
}

}