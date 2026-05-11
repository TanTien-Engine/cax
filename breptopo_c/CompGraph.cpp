#include "CompGraph.h"
#include "comp_ops.h"
#include "NodeInfo.h"
#include "TopoNaming.h"

#include <graph/Graph.h>
#include <graph/Node.h>
#include <graph/Edge.h>
#include <graph/GraphLayout.h>

#include <sstream>

namespace breptopo
{

// ---------------------------------------------------------------
//  CompGraph -- facade
// ---------------------------------------------------------------

CompGraph::CompGraph()
	: m_ir(m_reg)
	, m_eval(m_reg)
{
	RegisterBuiltinOps(m_reg);
	m_opt.AddDefaultRules();
}

int CompGraph::Register(NRef ref, const std::string& desc)
{
	int ext = static_cast<int>(m_nodes.size());
	m_nodes.push_back({ref, desc});
	m_ref2ext[ref.id] = ext;
	return ext;
}

int CompGraph::AddConst(int v, const std::string& desc)
{
	m_lowered = false;
	return m_history.AddConst(v, desc);
}

int CompGraph::AddConst(double v, const std::string& desc)
{
	m_lowered = false;
	return m_history.AddConst(v, desc);
}

int CompGraph::AddConst(bool v, const std::string& desc)
{
	m_lowered = false;
	return m_history.AddConst(v, desc);
}

int CompGraph::AddConst(Vec3 v, const std::string& desc)
{
	m_lowered = false;
	return m_history.AddConst(v, desc);
}

int CompGraph::AddConst(const std::shared_ptr<partgraph::TopoShape>& shp, const std::string& desc)
{
	m_lowered = false;
	return m_history.AddConst(shp, desc);
}

int CompGraph::AddOp(const std::string& op,
                     const std::vector<int>& inputs,
                     const std::vector<int>& var_inputs,
                     const std::string& desc)
{
	m_lowered = false;
	return m_history.AddOp(op, inputs, var_inputs, desc);
}

void CompGraph::SetRestoreFn(RestoreFn fn)
{
	m_eval.SetRestoreFn(std::move(fn));
}

void CompGraph::SetNodeVersion(int ext_id, uint32_t vt_node_id)
{
	if (!m_lowered) Lower();
	if (ext_id < 0 || ext_id >= (int)m_nodes.size()) return;
	auto* nd = m_ir.Get(m_nodes[ext_id].ref);
	if (nd) nd->vt_node_id = vt_node_id;
}

void CompGraph::UpdateConst(int ext_id, Val v)
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

void CompGraph::RebuildIR()
{
	m_ir.Clear();
	m_nodes.clear();
	m_ref2ext.clear();
	m_op_id_map.clear();

	for (auto& step : m_history.Steps())
	{
		if (!step.op_name.empty() && step.op_name[0] == '$')
		{
			NRef ref = NREF_NULL;
			if (step.op_name == "$int")        ref = m_ir.Const(std::get<int>(step.imm));
			else if (step.op_name == "$num")    ref = m_ir.Const(std::get<double>(step.imm));
			else if (step.op_name == "$bool")   ref = m_ir.Const(std::get<bool>(step.imm));
			else if (step.op_name == "$vec3")   ref = m_ir.Const(std::get<Vec3>(step.imm));
			else if (step.op_name == "$shape")  ref = m_ir.Const(std::get<ShapeVal>(step.imm));
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
			Register(m_ir.Add(step.op_name, refs, var_refs), step.desc);
		}
	}
}

void CompGraph::Lower()
{
	if (m_lowered) return;
	RebuildIR();
	m_lowered = true;
}

void CompGraph::Lower(int root_step)
{
	Lower();
}

void CompGraph::Optimize()
{
	if (!m_lowered) Lower();
	m_opt.Run(m_ir);
}

Val CompGraph::Eval(int ext_id)
{
	if (!m_lowered) Lower();
	if (ext_id < 0 || ext_id >= (int)m_nodes.size()) return {};
	if (!m_tn) m_tn = std::make_shared<TopoNaming>();
	NRef ref = m_nodes[ext_id].ref;
	if (m_parallel)
		m_eval.RunParallel(m_ir, ref, m_tn,
			[]() { return std::make_shared<TopoNaming>(); },
			[](const std::shared_ptr<TopoNaming>& dst,
			   const std::shared_ptr<TopoNaming>& src) { dst->MergeFrom(*src); });
	else
		m_eval.Run(m_ir, ref, m_tn);
	return m_eval.ResolveVal(m_ir, ref);
}

size_t CompGraph::NodeCount() const
{
	return m_ir.LiveCount();
}

uint32_t CompGraph::CalcOpId(int ext_id, int sub_op_id) const
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

NRef CompGraph::Ref(int ext_id) const
{
	if (ext_id < 0 || ext_id >= (int)m_nodes.size()) return NREF_NULL;
	return m_nodes[ext_id].ref;
}

static std::string FormatVal(const Val& v)
{
	std::string s;
	std::visit([&](auto&& val) {
		using T = std::decay_t<decltype(val)>;
		if constexpr (std::is_same_v<T, int>)      s = std::to_string(val);
		else if constexpr (std::is_same_v<T, double>) {
			std::ostringstream os; os << val; s = os.str();
		}
		else if constexpr (std::is_same_v<T, bool>)    s = val ? "true" : "false";
		else if constexpr (std::is_same_v<T, Vec3>) {
			std::ostringstream os;
			os << "(" << val[0] << ", " << val[1] << ", " << val[2] << ")";
			s = os.str();
		}
		else if constexpr (std::is_same_v<T, ShapeVal>) s = "[shape]";
	}, v);
	return s;
}

static std::string BuildNodeDesc(const IRNode& nd)
{
	std::ostringstream os;
	os << nd.op_name;
	auto val_str = FormatVal(nd.imm);
	if (!val_str.empty()) {
		os << " = " << val_str;
	}
	return os.str();
}

std::shared_ptr<graph::Graph> CompGraph::BuildVisGraph() const
{
	auto g = std::make_shared<graph::Graph>();

	auto order = m_ir.TopoSort();
	std::unordered_map<uint32_t, size_t> ref2gid;

	for (auto ref : order)
	{
		auto* nd = m_ir.Get(ref);
		if (!nd) continue;
		size_t gid = g->GetNodesNum();
		ref2gid[ref.id] = gid;

		auto node = std::make_shared<graph::Node>();
		node->SetValue(static_cast<int>(gid));
		node->SetName(nd->op_name);
		node->AddComponent<NodeInfo>(BuildNodeDesc(*nd));
		g->AddNode(node);
	}

	for (auto ref : order)
	{
		auto* nd = m_ir.Get(ref);
		if (!nd) continue;
		size_t to_gid = ref2gid[ref.id];
		for (auto& inp : nd->inputs)
		{
			auto it = ref2gid.find(inp.id);
			if (it != ref2gid.end())
				g->AddEdge(it->second, to_gid);
		}
		for (auto& inp : nd->var_inputs)
		{
			auto it = ref2gid.find(inp.id);
			if (it != ref2gid.end())
				g->AddEdge(it->second, to_gid);
		}
	}

	graph::GraphLayout::OptimalHierarchy(*g);
	return g;
}

} // namespace breptopo
