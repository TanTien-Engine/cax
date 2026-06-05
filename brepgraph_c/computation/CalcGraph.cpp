#include "brepgraph_c/computation/CalcGraph.h"
#include "brepgraph_c/computation/calc_ops.h"
#include "brepgraph_c/history/TopoNaming.h"

#include <cstring>

namespace brepgraph
{

// ---------------------------------------------------------------
//  CalcGraph -- facade
// ---------------------------------------------------------------

CalcGraph::CalcGraph()
	: m_ir(m_reg)
	, m_eval(m_reg)
{
	RegisterBuiltinOps(m_reg);
	m_opt.AddDefaultRules();
}

int CalcGraph::Register(NRef ref, const std::string& desc)
{
	int ext = static_cast<int>(m_nodes.size());
	m_nodes.push_back({ref, desc});
	m_ref2ext[ref.id] = ext;
	return ext;
}

int CalcGraph::AddConst(int v, const std::string& desc)
{
	m_lowered = false;
	return m_history.AddConst(v, desc);
}

int CalcGraph::AddConst(double v, const std::string& desc)
{
	m_lowered = false;
	return m_history.AddConst(v, desc);
}

int CalcGraph::AddConst(bool v, const std::string& desc)
{
	m_lowered = false;
	return m_history.AddConst(v, desc);
}

int CalcGraph::AddConst(Vec3 v, const std::string& desc)
{
	m_lowered = false;
	return m_history.AddConst(v, desc);
}

int CalcGraph::AddConst(const std::shared_ptr<brepkit::TopoShape>& shp, const std::string& desc)
{
	m_lowered = false;
	return m_history.AddConst(shp, desc);
}

int CalcGraph::AddConst(const std::shared_ptr<void>& sketch_handle, const std::string& desc)
{
	m_lowered = false;
	return m_history.AddConst(sketch_handle, desc);
}

int CalcGraph::AddConst(TopoRefVal v, const std::string& desc)
{
	m_lowered = false;
	return m_history.AddConst(std::move(v), desc);
}

int CalcGraph::AddOp(const std::string& op,
                     const std::vector<int>& inputs,
                     const std::vector<int>& var_inputs,
                     const std::string& desc)
{
	m_lowered = false;
	return m_history.AddOp(op, inputs, var_inputs, desc);
}

void CalcGraph::SetRestoreFn(RestoreFn fn)
{
	m_eval.SetRestoreFn(std::move(fn));
}

void CalcGraph::SetCommitFn(CommitFn fn)
{
	m_eval.SetCommitFn(std::move(fn));
}

void CalcGraph::SetNodeVersion(int ext_id, uint32_t vt_node_id)
{
	if (!m_lowered) Lower();
	if (ext_id < 0 || ext_id >= (int)m_nodes.size()) return;
	auto* nd = m_ir.Get(m_nodes[ext_id].ref);
	if (nd) {
		nd->vt_node_id = vt_node_id;
		nd->dirty      = false;
	}
}

void CalcGraph::Truncate(size_t keep)
{
	if (keep >= m_history.Size()) return;

	if (m_lowered && keep <= m_lowered_count)
	{
		for (size_t i = keep; i < m_nodes.size(); ++i)
		{
			NRef ref = m_nodes[i].ref;
			m_eval.GetShapeCache().Remove(ref.id);
			m_ir.Erase(ref);
			m_ref2ext.erase(ref.id);
		}
		m_nodes.resize(keep);
		m_lowered_count = keep;
	}
	else
	{
		m_lowered = false;
		m_lowered_count = 0;
	}

	m_history.Truncate(keep);
	m_op_id_map.clear();
	// Don't reset m_tn -- it's shared with GlobalConfig. If a true reset is
	// needed (e.g., naming is stale after truncate), do it explicitly elsewhere.
	m_lowered = false;
}

void CalcGraph::UpdateConst(int ext_id, Val v)
{
	m_history.UpdateConst(ext_id, v);

	if (m_lowered && ext_id >= 0 && ext_id < (int)m_nodes.size())
	{
		NRef ref = m_nodes[ext_id].ref;
		m_ir.UpdateImmediate(ref, v);
		m_eval.Invalidate(m_ir, ref);
	}
	else
	{
		m_lowered = false;
	}
}

void CalcGraph::RebuildIR()
{
	m_ir.Clear();
	m_nodes.clear();
	m_ref2ext.clear();
	m_op_id_map.clear();
	// Keep m_tn intact across IR rebuilds: op_ids are deterministic so the
	// existing naming data stays valid. Resetting it would create a fresh
	// empty TopoNaming on the next Eval, diverging from the one used by
	// .ves direct calls (GlobalConfig) and discarding loaded HistGraph data.
	m_lowered_count = 0;
	AppendNewSteps();
}

void CalcGraph::AppendNewSteps()
{
	auto& steps = m_history.Steps();
	for (size_t i = m_lowered_count; i < steps.size(); ++i)
	{
		auto& step = steps[i];
		NRef ref = NREF_NULL;
		if (!step.op_name.empty() && step.op_name[0] == '$')
		{
			if (step.op_name == "$int")        ref = m_ir.Const(std::get<int>(step.imm));
			else if (step.op_name == "$num")    ref = m_ir.Const(std::get<double>(step.imm));
			else if (step.op_name == "$bool")   ref = m_ir.Const(std::get<bool>(step.imm));
			else if (step.op_name == "$vec3")   ref = m_ir.Const(std::get<Vec3>(step.imm));
			else if (step.op_name == "$shape")  ref = m_ir.Const(std::get<ShapeVal>(step.imm));
			else if (step.op_name == "$sketch") ref = m_ir.Const(std::get<SketchVal>(step.imm));
			else if (step.op_name == "$toporef") ref = m_ir.Const(std::get<TopoRefVal>(step.imm));
			Register(ref, step.desc);
		}
		else
		{
			std::vector<NRef> refs;
			for (int i : step.inputs)
			{
				if (i >= 0 && i < (int)m_nodes.size())
					refs.push_back(m_nodes[i].ref);
				else
					refs.push_back(NREF_NULL);
			}
			std::vector<NRef> var_refs;
			for (int i : step.var_inputs)
			{
				if (i >= 0 && i < (int)m_nodes.size())
					var_refs.push_back(m_nodes[i].ref);
				else
					var_refs.push_back(NREF_NULL);
			}
			ref = m_ir.Add(step.op_name, refs, var_refs);
			Register(ref, step.desc);
		}

		// Bring this node to a fully "as-if just-evaluated" state so the
		// demand-driven EvalNode's freshness check passes on the first
		// post-load Eval. Done for every step (not just those with a
		// vt_node_id) -- otherwise loaded const-number / non-shape op
		// nodes would default to dirty=true on first Eval, bump their
		// result_rev, and cascade-invalidate everything that reads them.
		{
			auto* nd = m_ir.Get(ref);
			if (nd) {
				if (step.vt_node_id != UINT32_MAX)
					nd->vt_node_id = step.vt_node_id;
				nd->dirty = false;
				nd->input_revs_at_eval.clear();
				nd->input_revs_at_eval.reserve(nd->inputs.size());
				for (auto& inp : nd->inputs) {
					auto* s = m_ir.Get(inp);
					nd->input_revs_at_eval.push_back(s ? s->result_rev : 0);
				}
				// Stamp loaded node as validated for the current epoch, so
				// the first post-load Eval can return its cache without
				// recursing into the subtree.
				m_eval.StampValidated(*nd);
			}
		}
	}
	m_lowered_count = steps.size();
}

void CalcGraph::Lower()
{
	if (m_lowered) return;
	if (m_lowered_count > 0)
		AppendNewSteps();
	else
		RebuildIR();
	m_lowered = true;
}

void CalcGraph::Optimize()
{
	if (!m_lowered) Lower();
	m_opt.Run(m_ir);
}

Val CalcGraph::Eval(int ext_id)
{
	if (!m_lowered) Lower();
	if (ext_id < 0 || ext_id >= (int)m_nodes.size()) return {};
	if (!m_tn) m_tn = std::make_shared<TopoNaming>();
	NRef ref = m_nodes[ext_id].ref;
	if (m_parallel)
	{
		// Each fork gets a deep clone of m_tn so concurrent HistGraph::Update
		// calls don't race on the shared unordered_maps. A snapshot taken
		// once per fork pattern lets the merge absorb only the fork's
		// additions back into the main TN at the join.
		struct ForkState {
			TopoNaming::Snapshot snap;
			int clones_issued = 0;
			int merges_done   = 0;
		};
		auto state = std::make_shared<ForkState>();
		auto tn    = m_tn;

		auto factory = [tn, state]() -> std::shared_ptr<TopoNaming> {
			if (state->clones_issued == state->merges_done)
				state->snap = tn->TakeSnapshot();
			state->clones_issued++;
			return tn->Clone();
		};
		auto merge = [state](const std::shared_ptr<TopoNaming>& dst,
		                     const std::shared_ptr<TopoNaming>& src) {
			dst->AbsorbFork(*src, state->snap);
			state->merges_done++;
		};

		m_eval.RunParallel(m_ir, ref, m_tn, factory, merge);
	}
	else
		m_eval.Run(m_ir, ref, m_tn);
	return m_eval.ResolveVal(m_ir, ref);
}

size_t CalcGraph::NodeCount() const
{
	return m_ir.LiveCount();
}

uint32_t CalcGraph::CalcOpId(int ext_id, int sub_op_id) const
{
	auto key = std::make_pair(ext_id, sub_op_id);
	auto itr = m_op_id_map.find(key);
	if (itr == m_op_id_map.end())
	{
		uint32_t val = static_cast<uint32_t>(m_op_id_map.size());
		m_op_id_map.insert({ key, val });
		return val;
	}
	return itr->second;
}

NRef CalcGraph::Ref(int ext_id) const
{
	if (ext_id < 0 || ext_id >= (int)m_nodes.size()) return NREF_NULL;
	return m_nodes[ext_id].ref;
}

std::unordered_map<uint32_t, std::string> CalcGraph::GetIROpIdMap() const
{
	std::unordered_map<uint32_t, std::string> result;
	for (auto ref : m_ir.TopoSort()) {
		auto* nd = m_ir.Get(ref);
		if (!nd || nd->dead) continue;
		if (nd->op_name.empty() || nd->op_name[0] == '$') continue;
		if (nd->op_id != UINT32_MAX) {
			result[nd->op_id] = nd->op_name;
		}
	}
	return result;
}

// ---------------------------------------------------------------
//  CalcGraph reconnection support
// ---------------------------------------------------------------

static const std::string s_empty_str;

const std::string& CalcGraph::GetStepOpName(int step_id) const
{
	auto* s = m_history.Get(step_id);
	return s ? s->op_name : s_empty_str;
}

std::vector<int> CalcGraph::GetStepInputs(int step_id) const
{
	auto* s = m_history.Get(step_id);
	if (!s) {
		return std::vector<int>{};
	}
	// Fixed inputs FOLLOWED BY variadic inputs, so a host gets the step's
	// COMPLETE input list. The history rebuild (hist_graph_builder.wire_node)
	// slots them as [shapes..., params..., variadic...]; a dressup's edges live
	// in var_inputs, and returning only `inputs` left them unwired (chamfer's
	// Selector never connected to the Chamfer's edges pin). Only caller is the
	// ves binding, so this widening is safe.
	std::vector<int> all = s->inputs;
	all.insert(all.end(), s->var_inputs.begin(), s->var_inputs.end());
	return all;
}

// ---------------------------------------------------------------
//  CalcGraph persistence
//
//  vt_node_id is stored on OpStep (persistent) rather than IRNode
//  (rebuilt on every Lower). StoreToByteArray syncs IR vt_node_ids
//  back to history steps before serializing.
// ---------------------------------------------------------------

void CalcGraph::StoreToByteArray(uint8_t** buf, uint32_t& len) const
{
	auto& hist = const_cast<OpHistory&>(m_history);
	for (size_t ext = 0; ext < m_nodes.size(); ++ext)
	{
		auto* nd = m_ir.Get(m_nodes[ext].ref);
		if (nd)
			hist.SetStepVtNodeId(static_cast<int>(ext), nd->vt_node_id);
	}
	m_history.StoreToByteArray(buf, len);
}

bool CalcGraph::LoadFromByteArray(const uint8_t* buf, uint32_t len)
{
	m_history.Clear();
	m_lowered = false;
	m_preloaded = false;

	if (!m_history.LoadFromByteArray(buf, len))
		return false;

	Lower();
	m_preloaded = true;
	return true;
}

} // namespace brepgraph
