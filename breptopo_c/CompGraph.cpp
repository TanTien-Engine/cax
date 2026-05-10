#include "CompGraph.h"
#include "comp_ops.h"
#include "NodeInfo.h"
#include "TopoNaming.h"
#include <algorithm>
#include <queue>
#include <sstream>

#include <graph/Graph.h>
#include <graph/Node.h>
#include <graph/Edge.h>
#include <graph/GraphLayout.h>

namespace breptopo
{

// ---------------------------------------------------------------
//  EvalCtx
// ---------------------------------------------------------------

double EvalCtx::Num(size_t i) const {
	if (i >= inputs.size()) return 0.0;
	if (auto* d = std::get_if<double>(&inputs[i])) return *d;
	if (auto* n = std::get_if<int>(&inputs[i]))    return (double)*n;
	return 0.0;
}
int EvalCtx::Int(size_t i) const {
	if (i >= inputs.size()) return 0;
	if (auto* n = std::get_if<int>(&inputs[i]))    return *n;
	if (auto* d = std::get_if<double>(&inputs[i])) return (int)*d;
	return 0;
}
bool EvalCtx::Bool(size_t i) const {
	if (i >= inputs.size()) return false;
	if (auto* b = std::get_if<bool>(&inputs[i])) return *b;
	return false;
}
Vec3 EvalCtx::GetVec3(size_t i) const {
	if (i >= inputs.size()) return {0,0,0};
	if (auto* v = std::get_if<Vec3>(&inputs[i])) return *v;
	return {0,0,0};
}
ShapeVal EvalCtx::GetShape(size_t i) const {
	if (i >= inputs.size()) return {};
	if (auto* s = std::get_if<ShapeVal>(&inputs[i])) return *s;
	return {};
}
std::vector<ShapeVal> EvalCtx::VarShapes() const {
	std::vector<ShapeVal> out;
	for (auto& v : var_inputs)
		if (auto* s = std::get_if<ShapeVal>(&v)) out.push_back(*s);
	return out;
}

// ---------------------------------------------------------------
//  OpRegistry
// ---------------------------------------------------------------

void OpRegistry::Define(const std::string& name,
                        std::vector<std::string> inputs,
                        std::vector<std::string> var_inputs,
                        EvalFn eval, OpFlags flags)
{
	size_t idx = m_ops.size();
	m_ops.push_back({name, std::move(inputs), std::move(var_inputs), std::move(eval), flags});
	m_index[name] = idx;
}

const OpDesc* OpRegistry::Find(const std::string& name) const
{
	auto it = m_index.find(name);
	return it != m_index.end() ? &m_ops[it->second] : nullptr;
}

// ---------------------------------------------------------------
//  IRGraph
// ---------------------------------------------------------------

NRef IRGraph::Alloc() { return NRef{m_next_id++}; }

NRef IRGraph::Const(int v)      { auto r=Alloc(); m_nodes[r.id]={r,"$int",v};       return r; }
NRef IRGraph::Const(double v)   { auto r=Alloc(); m_nodes[r.id]={r,"$num",v};       return r; }
NRef IRGraph::Const(bool v)     { auto r=Alloc(); m_nodes[r.id]={r,"$bool",v};      return r; }
NRef IRGraph::Const(Vec3 v)     { auto r=Alloc(); m_nodes[r.id]={r,"$vec3",v};      return r; }
NRef IRGraph::Const(ShapeVal v) { auto r=Alloc(); m_nodes[r.id]={r,"$shape",std::move(v)}; return r; }

NRef IRGraph::Add(const std::string& op,
                  const std::vector<NRef>& inputs,
                  const std::vector<NRef>& var_inputs)
{
	auto r = Alloc();
	IRNode nd;
	nd.ref        = r;
	nd.op_name    = op;
	nd.inputs     = inputs;
	nd.var_inputs = var_inputs;
	m_nodes[r.id] = std::move(nd);
	return r;
}

void IRGraph::UpdateImmediate(NRef ref, Val v)
{
	auto it = m_nodes.find(ref.id);
	if (it == m_nodes.end()) return;
	it->second.imm = std::move(v);
	it->second.version++;
}

void IRGraph::Kill(NRef ref)
{
	auto it = m_nodes.find(ref.id);
	if (it != m_nodes.end()) it->second.dead = true;
}

void IRGraph::ReplaceAllUses(NRef old_ref, NRef new_ref)
{
	for (auto& [id, nd] : m_nodes)
	{
		if (nd.dead) continue;
		for (auto& inp : nd.inputs)
			if (inp == old_ref) inp = new_ref;
		for (auto& inp : nd.var_inputs)
			if (inp == old_ref) inp = new_ref;
	}
}

void IRGraph::Compact()
{
	std::vector<uint32_t> to_erase;
	for (auto& [id, nd] : m_nodes)
		if (nd.dead) to_erase.push_back(id);
	for (auto id : to_erase)
		m_nodes.erase(id);
}

const IRNode* IRGraph::Get(NRef ref) const
{
	auto it = m_nodes.find(ref.id);
	return it != m_nodes.end() && !it->second.dead ? &it->second : nullptr;
}

IRNode* IRGraph::Get(NRef ref)
{
	auto it = m_nodes.find(ref.id);
	return it != m_nodes.end() && !it->second.dead ? &it->second : nullptr;
}

size_t IRGraph::LiveCount() const
{
	size_t n = 0;
	for (auto& [id, nd] : m_nodes)
		if (!nd.dead) n++;
	return n;
}

std::vector<NRef> IRGraph::UsersOf(NRef ref) const
{
	std::vector<NRef> result;
	for (auto& [id, nd] : m_nodes)
	{
		if (nd.dead) continue;
		bool found = false;
		for (auto& inp : nd.inputs) {
			if (inp == ref) { found = true; break; }
		}
		if (!found) {
			for (auto& inp : nd.var_inputs) {
				if (inp == ref) { found = true; break; }
			}
		}
		if (found) result.push_back(nd.ref);
	}
	return result;
}

std::vector<NRef> IRGraph::TopoSort() const
{
	std::unordered_map<uint32_t, int> in_deg;
	std::unordered_map<uint32_t, std::vector<uint32_t>> succs;

	for (auto& [id, nd] : m_nodes)
	{
		if (nd.dead) continue;
		if (in_deg.find(id) == in_deg.end()) in_deg[id] = 0;

		auto add = [&](NRef src) {
			if (!src.valid()) return;
			auto sit = m_nodes.find(src.id);
			if (sit == m_nodes.end() || sit->second.dead) return;
			succs[src.id].push_back(id);
			in_deg[id]++;
			if (in_deg.find(src.id) == in_deg.end()) in_deg[src.id] = 0;
		};
		for (auto& inp : nd.inputs)     add(inp);
		for (auto& inp : nd.var_inputs) add(inp);
	}

	std::queue<uint32_t> q;
	for (auto& [id, deg] : in_deg)
		if (deg == 0) q.push(id);

	std::vector<NRef> order;
	while (!q.empty())
	{
		uint32_t cur = q.front(); q.pop();
		order.push_back(NRef{cur});
		for (auto s : succs[cur])
			if (--in_deg[s] == 0) q.push(s);
	}
	return order;
}

std::string IRGraph::Dump() const
{
	auto order = TopoSort();
	std::ostringstream os;
	os << "  IRGraph (" << LiveCount() << " live)\n";
	for (auto ref : order)
	{
		auto* nd = Get(ref);
		if (!nd) continue;
		os << "    #" << ref.id << " " << nd->op_name;
		std::visit([&](auto&& v) {
			using T = std::decay_t<decltype(v)>;
			if constexpr (std::is_same_v<T,int>)    os << " =" << v;
			else if constexpr (std::is_same_v<T,double>) os << " =" << v;
			else if constexpr (std::is_same_v<T,bool>)   os << " =" << (v?"T":"F");
			else if constexpr (std::is_same_v<T,Vec3>)   os << " =("<<v[0]<<","<<v[1]<<","<<v[2]<<")";
			else if constexpr (std::is_same_v<T,ShapeVal>) os << " shp:"<<v.tag;
		}, nd->imm);
		if (!nd->inputs.empty() || !nd->var_inputs.empty())
		{
			os << " <- [";
			for (size_t i=0; i<nd->inputs.size(); ++i) { if(i) os<<","; os<<"#"<<nd->inputs[i].id; }
			for (size_t i=0; i<nd->var_inputs.size(); ++i) { if(!nd->inputs.empty()||i>0) os<<","; os<<"+"<<nd->var_inputs[i].id; }
			os << "]";
		}
		os << "\n";
	}
	return os.str();
}

// ---------------------------------------------------------------
//  Predicate builders
// ---------------------------------------------------------------

MatchPred ByName(const std::string& name)
{
	return [name](const IRNode& nd, const OpDesc*) { return nd.op_name == name; };
}

MatchPred ByFlag(std::function<bool(const OpFlags&)> test)
{
	return [test](const IRNode&, const OpDesc* d) { return d && test(d->flags); };
}

MatchPred Any()
{
	return [](const IRNode&, const OpDesc*) { return true; };
}

// ---------------------------------------------------------------
//  Optimizer
// ---------------------------------------------------------------

void Optimizer::AddRule(RewriteRule rule)
{
	m_rules.push_back(std::move(rule));
	std::sort(m_rules.begin(), m_rules.end(),
		[](auto& a, auto& b) { return a.priority > b.priority; });
}

bool Optimizer::TryMatch(const IRGraph& g, NRef start,
                         const RewriteRule& rule, MatchResult& m) const
{
	if (rule.pattern.empty()) return false;
	NRef cur = start;
	for (size_t i = 0; i < rule.pattern.size(); ++i)
	{
		auto& step = rule.pattern[i];
		auto* nd = g.Get(cur);
		if (!nd || nd->dead) return false;
		auto* desc = g.Registry().Find(nd->op_name);
		if (!step.pred(*nd, desc)) return false;
		if (!step.capture.empty()) m.caps[step.capture] = cur;
		if (i == 0) m.root = cur;
		if (step.follow_input >= 0)
		{
			size_t slot = (size_t)step.follow_input;
			if (slot >= nd->inputs.size()) return false;
			cur = nd->inputs[slot];
		}
	}
	return true;
}

void Optimizer::AddDefaultRules()
{
	// dressup(pattern(shape, ...), ...) -> pattern(dressup(shape, ...), ...)
	// e.g. fillet(array(box)) -> array(fillet(box))
	AddRule({
		"pattern_dressup_commute",
		{
			{ByFlag([](auto& f){return f.is_dressup;}), "du",  0},
			{ByFlag([](auto& f){return f.is_pattern;}), "pat", -1},
		},
		[](MatchResult& m, IRGraph& g) -> bool {
			auto* du  = g.Get(m["du"]);
			auto* pat = g.Get(m["pat"]);
			if (!du || !pat) return false;
			if (!du->var_inputs.empty()) return false;
			if (g.UsersOf(m["pat"]).size() != 1) return false;

			NRef orig_shape = pat->inputs[0];
			std::vector<NRef> du_params(du->inputs.begin()+1, du->inputs.end());
			std::vector<NRef> pat_params(pat->inputs.begin()+1, pat->inputs.end());
			std::string du_op = du->op_name;
			std::string pat_op = pat->op_name;

			auto* p = g.Get(m["pat"]);
			p->op_name = du_op;
			p->inputs.clear();
			p->inputs.push_back(orig_shape);
			for (auto& x : du_params) p->inputs.push_back(x);
			p->var_inputs.clear();
			p->version++;

			auto* d = g.Get(m["du"]);
			d->op_name = pat_op;
			d->inputs.clear();
			d->inputs.push_back(m["pat"]);
			for (auto& x : pat_params) d->inputs.push_back(x);
			d->var_inputs.clear();
			d->version++;

			return true;
		},
		10
	});
}

bool Optimizer::Run(IRGraph& g, int max_iter) const
{
	for (int iter = 0; iter < max_iter; ++iter)
	{
		bool changed = false;
		auto order = g.TopoSort();
		for (auto& rule : m_rules)
			for (auto ref : order)
			{
				MatchResult m;
				if (TryMatch(g, ref, rule, m))
					if (rule.rewrite(m, g)) changed = true;
			}
		if (CSE(g)) changed = true;
		if (DCE(g)) changed = true;
		if (!changed) break;
	}
	g.Compact();
	return true;
}

bool Optimizer::DCE(IRGraph& g)
{
	bool changed = false;
	auto order = g.TopoSort();
	if (order.empty()) return false;
	NRef root = order.back();

	std::unordered_map<uint32_t, int> uses;
	for (auto ref : order) uses[ref.id] = 0;
	for (auto ref : order)
	{
		auto* nd = g.Get(ref);
		if (!nd) continue;
		for (auto& inp : nd->inputs)     if (inp.valid()) uses[inp.id]++;
		for (auto& inp : nd->var_inputs) if (inp.valid()) uses[inp.id]++;
	}
	for (auto ref : order)
	{
		if (ref == root) continue;
		if (uses[ref.id] == 0) { g.Kill(ref); changed = true; }
	}
	return changed;
}

bool Optimizer::CSE(IRGraph& g)
{
	struct Key {
		std::string op; size_t imm_idx; std::vector<uint32_t> ids;
		bool operator==(const Key& o) const { return op==o.op && imm_idx==o.imm_idx && ids==o.ids; }
	};
	struct KH {
		size_t operator()(const Key& k) const {
			size_t h = std::hash<std::string>{}(k.op) ^ (k.imm_idx*0x9e3779b9);
			for (auto i : k.ids) h ^= std::hash<uint32_t>{}(i)+0x9e3779b9+(h<<6)+(h>>2);
			return h;
		}
	};
	std::unordered_map<Key,NRef,KH> seen;
	bool changed = false;
	for (auto ref : g.TopoSort())
	{
		auto* nd = g.Get(ref);
		if (!nd || nd->op_name == "$shape") continue;
		Key key; key.op = nd->op_name; key.imm_idx = nd->imm.index();
		for (auto& inp : nd->inputs)     key.ids.push_back(inp.id);
		for (auto& inp : nd->var_inputs) key.ids.push_back(inp.id);
		auto it = seen.find(key);
		if (it != seen.end()) { g.ReplaceAllUses(ref, it->second); g.Kill(ref); changed = true; }
		else seen[key] = ref;
	}
	return changed;
}

// ---------------------------------------------------------------
//  Evaluator
// ---------------------------------------------------------------

uint64_t Evaluator::InputHash(const IRGraph& g, const IRNode& node) const
{
	uint64_t h = 0;
	for (auto& inp : node.inputs) {
		auto* s = g.Get(inp);
		if (s) h ^= s->version * 0x517cc1b727220a95ULL + inp.id;
	}
	for (auto& inp : node.var_inputs) {
		auto* s = g.Get(inp);
		if (s) h ^= s->version * 0x6c62272e07bb0142ULL + inp.id;
	}
	return h;
}

Val Evaluator::EvalNode(IRGraph& g, NRef ref,
                        const std::shared_ptr<TopoNaming>& tn,
                        uint32_t& op_counter)
{
	auto* nd = g.Get(ref);
	if (!nd) return {};

	if (!nd->op_name.empty() && nd->op_name[0] == '$')
	{
		nd->cached = nd->imm;
		nd->eval_version = nd->version;
		return nd->imm;
	}

	uint64_t ih = InputHash(g, *nd);
	if (nd->eval_version == nd->version && ih != 0)
	{
		m_hits++;
		return nd->cached;
	}

	std::vector<Val> resolved;
	for (auto& inp : nd->inputs) {
		auto* s = g.Get(inp);
		resolved.push_back(s ? s->cached : Val{});
	}
	std::vector<Val> var_resolved;
	for (auto& inp : nd->var_inputs) {
		auto* s = g.Get(inp);
		var_resolved.push_back(s ? s->cached : Val{});
	}

	auto* desc = m_reg.Find(nd->op_name);
	Val result;
	if (desc && desc->eval)
	{
		EvalCtx ctx{resolved, var_resolved, tn, op_counter++};
		result = desc->eval(ctx);
	}
	nd->cached = result;
	nd->eval_version = nd->version;
	m_misses++;
	return result;
}

Val Evaluator::Run(IRGraph& g, NRef root, const std::shared_ptr<TopoNaming>& tn)
{
	uint32_t op_counter = 0;
	for (auto ref : g.TopoSort())
		EvalNode(g, ref, tn, op_counter);
	auto* nd = g.Get(root);
	return nd ? nd->cached : Val{};
}

void Evaluator::Invalidate(IRGraph& g, NRef ref)
{
	auto* nd = g.Get(ref);
	if (!nd) return;
	nd->version++;
	for (auto user : g.UsersOf(ref))
		Invalidate(g, user);
}

// ---------------------------------------------------------------
//  OpHistory
// ---------------------------------------------------------------

int OpHistory::Append(OpStep step)
{
	int id = static_cast<int>(m_steps.size());
	step.step_id = id;
	m_steps.push_back(std::move(step));
	return id;
}

int OpHistory::AddConst(int v, const std::string& desc)
{
	return Append({-1, "$int", Val(v), {}, {}, desc});
}
int OpHistory::AddConst(double v, const std::string& desc)
{
	return Append({-1, "$num", Val(v), {}, {}, desc});
}
int OpHistory::AddConst(bool v, const std::string& desc)
{
	return Append({-1, "$bool", Val(v), {}, {}, desc});
}
int OpHistory::AddConst(Vec3 v, const std::string& desc)
{
	return Append({-1, "$vec3", Val(v), {}, {}, desc});
}
int OpHistory::AddConst(const std::shared_ptr<partgraph::TopoShape>& shp, const std::string& desc)
{
	ShapeVal sv;
	sv.shape = shp;
	return Append({-1, "$shape", Val(std::move(sv)), {}, {}, desc});
}

int OpHistory::AddOp(const std::string& op,
                     const std::vector<int>& inputs,
                     const std::vector<int>& var_inputs,
                     const std::string& desc)
{
	return Append({-1, op, {}, inputs, var_inputs, desc});
}

void OpHistory::UpdateConst(int step_id, Val v)
{
	if (step_id >= 0 && step_id < (int)m_steps.size())
		m_steps[step_id].imm = std::move(v);
}

const OpStep* OpHistory::Get(int step_id) const
{
	if (step_id >= 0 && step_id < (int)m_steps.size())
		return &m_steps[step_id];
	return nullptr;
}

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
	auto eval_tn = std::make_shared<TopoNaming>();
	return m_eval.Run(m_ir, m_nodes[ext_id].ref, eval_tn);
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
