#include "CompGraph.h"

#include <algorithm>
#include <future>
#include <queue>
#include <sstream>

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

void IRGraph::Erase(NRef ref)
{
	m_nodes.erase(ref.id);
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

	for (auto& kv : m_nodes)
	{
		uint32_t id = kv.first;
		auto& nd = kv.second;
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

std::unordered_set<uint32_t> IRGraph::CollectDeps(NRef ref) const
{
	std::unordered_set<uint32_t> deps;
	std::vector<uint32_t> stack;
	stack.push_back(ref.id);
	while (!stack.empty())
	{
		uint32_t cur = stack.back(); stack.pop_back();
		if (!deps.insert(cur).second) continue;
		auto it = m_nodes.find(cur);
		if (it == m_nodes.end() || it->second.dead) continue;
		for (auto& inp : it->second.inputs)
			if (inp.valid()) stack.push_back(inp.id);
		for (auto& inp : it->second.var_inputs)
			if (inp.valid()) stack.push_back(inp.id);
	}
	return deps;
}

bool IRGraph::AreIndependent(NRef a, NRef b) const
{
	auto deps_a = CollectDeps(a);
	auto deps_b = CollectDeps(b);
	for (auto id : deps_a)
		if (deps_b.count(id)) return false;
	return true;
}

void IRGraph::DepIndex::Build(const IRGraph& g)
{
	auto order = g.TopoSort();

	// assign each leaf a unique bit index
	std::unordered_map<uint32_t, size_t> leaf_idx;
	for (auto ref : order)
	{
		auto* nd = g.Get(ref);
		if (!nd) continue;
		if (nd->inputs.empty() && nd->var_inputs.empty())
			leaf_idx[ref.id] = leaf_idx.size();
	}

	m_words = (leaf_idx.size() + 63) / 64;
	if (m_words == 0) m_words = 1;

	for (auto ref : order)
	{
		auto* nd = g.Get(ref);
		if (!nd) continue;

		auto& bits = m_bits[ref.id];
		bits.assign(m_words, 0);

		auto it = leaf_idx.find(ref.id);
		if (it != leaf_idx.end())
		{
			bits[it->second / 64] |= Word(1) << (it->second % 64);
		}

		// OR in all inputs' bitsets
		auto merge = [&](NRef inp) {
			auto jt = m_bits.find(inp.id);
			if (jt == m_bits.end()) return;
			for (size_t w = 0; w < m_words; ++w)
				bits[w] |= jt->second[w];
		};
		for (auto& inp : nd->inputs)     merge(inp);
		for (auto& inp : nd->var_inputs) merge(inp);
	}
}

bool IRGraph::DepIndex::AreIndependent(NRef a, NRef b) const
{
	auto ia = m_bits.find(a.id);
	auto ib = m_bits.find(b.id);
	if (ia == m_bits.end() || ib == m_bits.end()) return true;
	for (size_t w = 0; w < m_words; ++w)
		if (ia->second[w] & ib->second[w]) return false;
	return true;
}

IRGraph::DepIndex IRGraph::BuildDepIndex() const
{
	DepIndex idx;
	idx.Build(*this);
	return idx;
}

std::vector<std::vector<NRef>> IRGraph::TopoLevels() const
{
	std::unordered_map<uint32_t, int> in_deg;
	std::unordered_map<uint32_t, std::vector<uint32_t>> succs;

	for (auto& kv : m_nodes)
	{
		uint32_t id = kv.first;
		auto& nd = kv.second;
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

	std::vector<std::vector<NRef>> levels;
	std::vector<uint32_t> frontier;
	for (auto& [id, deg] : in_deg)
		if (deg == 0) frontier.push_back(id);

	while (!frontier.empty())
	{
		std::vector<NRef> level;
		for (auto id : frontier) level.push_back(NRef{id});
		levels.push_back(std::move(level));

		std::vector<uint32_t> next;
		for (auto id : frontier)
			for (auto s : succs[id])
				if (--in_deg[s] == 0) next.push_back(s);
		frontier = std::move(next);
	}
	return levels;
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

void IRGraph::AssignOpIds()
{
	uint32_t counter = 0;
	for (auto ref : TopoSort()) {
		auto* nd = Get(ref);
		if (!nd || nd->dead) continue;
		if (!nd->op_name.empty() && nd->op_name[0] == '$') continue;
		auto* desc = m_reg.Find(nd->op_name);
		if (desc && desc->eval)
			nd->op_id = counter++;
	}
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

Val Evaluator::ResolveVal(const IRGraph& g, NRef ref,
                          const std::shared_ptr<TopoNaming>& tn) const
{
	auto* nd = g.Get(ref);
	if (!nd) return {};
	if (!std::holds_alternative<std::monostate>(nd->cached))
		return nd->cached;
	auto* v = m_shape_cache.Get(ref.id);
	if (v) return *v;
	if (nd->vt_node_id != UINT32_MAX && m_restore_fn)
	{
		Val restored = m_restore_fn(nd->vt_node_id, tn);
		if (!std::holds_alternative<std::monostate>(restored))
		{
			m_shape_cache.Put(ref.id, restored);
			m_restores++;
			return restored;
		}
	}
	// fallback for constants
	if (!nd->op_name.empty() && nd->op_name[0] == '$')
		return nd->imm;
	return {};
}

Val Evaluator::EvalNode(IRGraph& g, NRef ref,
                        const std::shared_ptr<TopoNaming>& tn)
{
	auto* nd = g.Get(ref);
	if (!nd) return {};

	// Constants: imm is the value. Cache it on first encounter; the imm
	// itself only changes when version bumps via UpdateImmediate.
	if (!nd->op_name.empty() && nd->op_name[0] == '$')
	{
		if (nd->eval_version != nd->version)
		{
			if (std::holds_alternative<ShapeVal>(nd->imm))
			{
				m_shape_cache.Put(ref.id, nd->imm);
				if (m_commit_fn && nd->vt_node_id == UINT32_MAX)
					nd->vt_node_id = m_commit_fn(ref.id, nd->imm, tn);
			}
			else
			{
				nd->cached = nd->imm;
			}
			nd->eval_version = nd->version;
			nd->result_rev++;
		}
		return nd->imm;
	}

	const size_t n_in    = nd->inputs.size();
	const size_t n_var   = nd->var_inputs.size();
	const size_t n_total = n_in + n_var;

	// Epoch-based fast path: if this node was validated under the current
	// global epoch (no Invalidate has fired since), trust the cache without
	// touching the subtree. Critical for "load + render" -- after Lower
	// seeds last_validated_epoch on every loaded node, the very first Eval
	// of root returns immediately from its restored cache; selectors and
	// other downstream no_vt_cache ops are never visited.
	if (nd->last_validated_epoch == m_eval_epoch)
	{
		Val cached = ResolveVal(g, ref, tn);
		if (!std::holds_alternative<std::monostate>(cached))
		{
			m_hits++;
			return cached;
		}
		// Cache absent (eviction or no_vt_cache op): fall through, but
		// since nothing has been invalidated, the recompute below won't
		// bump result_rev.
	}

	// Recurse into inputs so the subtree gets a chance to detect staleness.
	std::vector<Val> resolved;     resolved.reserve(n_in);
	std::vector<Val> var_resolved; var_resolved.reserve(n_var);
	std::vector<uint64_t> in_revs; in_revs.reserve(n_total);

	for (auto& inp : nd->inputs)
	{
		resolved.push_back(EvalNode(g, inp, tn));
		auto* s = g.Get(inp);
		in_revs.push_back(s ? s->result_rev : 0);
	}
	for (auto& inp : nd->var_inputs)
	{
		var_resolved.push_back(EvalNode(g, inp, tn));
		auto* s = g.Get(inp);
		in_revs.push_back(s ? s->result_rev : 0);
	}

	bool self_fresh   = (nd->eval_version == nd->version);
	bool inputs_fresh = (nd->input_revs_at_eval.size() == n_total);
	if (inputs_fresh)
	{
		for (size_t i = 0; i < n_total; ++i)
		{
			if (nd->input_revs_at_eval[i] != in_revs[i])
			{
				inputs_fresh = false;
				break;
			}
		}
	}

	const bool stale = !self_fresh || !inputs_fresh;

	if (!stale)
	{
		Val cached = ResolveVal(g, ref, tn);
		if (!std::holds_alternative<std::monostate>(cached))
		{
			m_hits++;
			nd->input_revs_at_eval   = std::move(in_revs);
			nd->last_validated_epoch = m_eval_epoch;
			return cached;
		}
		// Fresh-but-no-cache: re-run lambda for materialization. Don't bump
		// result_rev (output is semantically the same as last time).
	}

	auto* desc = m_reg.Find(nd->op_name);
	Val result;
	if (desc && desc->eval)
	{
		EvalCtx ctx{resolved, var_resolved, tn, nd->op_id};
		result = desc->eval(ctx);
	}

	bool no_vt_cache = desc && desc->flags.no_vt_cache;
	if (std::holds_alternative<ShapeVal>(result))
	{
		m_shape_cache.Put(ref.id, result);
		if (!no_vt_cache && m_commit_fn)
			nd->vt_node_id = m_commit_fn(ref.id, result, tn);
	}
	else
	{
		nd->cached = result;
	}
	nd->eval_version         = nd->version;
	nd->input_revs_at_eval   = std::move(in_revs);
	nd->last_validated_epoch = m_eval_epoch;
	if (stale)
		nd->result_rev++;   // semantic output may have changed; tell downstream
	m_misses++;
	return result;
}

Val Evaluator::Run(IRGraph& g, NRef root, const std::shared_ptr<TopoNaming>& tn)
{
	// EvalNode is now demand-driven: it recursively pulls inputs and
	// short-circuits when self + inputs are fresh. Starting from root
	// is sufficient -- unused branches of the IR aren't visited.
	g.AssignOpIds();
	return EvalNode(g, root, tn);
}

void Evaluator::EvalSubtree(IRGraph& g, NRef root,
                            const std::shared_ptr<TopoNaming>& tn)
{
	auto deps = g.CollectDeps(root);
	auto order = g.TopoSort();
	for (auto ref : order)
		if (deps.count(ref.id))
			EvalNode(g, ref, tn);
}

Val Evaluator::RunParallel(IRGraph& g, NRef root,
                           const std::shared_ptr<TopoNaming>& tn,
                           TnFactory tn_factory, TnMerge tn_merge)
{
	g.AssignOpIds();
	auto dep_idx = g.BuildDepIndex();
	auto order = g.TopoSort();

	struct ForkPoint {
		NRef bool_ref;
		NRef inA, inB;
		std::unordered_set<uint32_t> depsA, depsB;
	};
	std::vector<ForkPoint> forks;
	std::unordered_set<uint32_t> deferred;

	// pre-scan: find boolean ops with independent inputs, defer their subtrees
	for (auto ref : order)
	{
		auto* nd = g.Get(ref);
		if (!nd) continue;
		auto* desc = m_reg.Find(nd->op_name);
		if (!desc || !desc->flags.is_boolean) continue;
		if (nd->inputs.size() != 2) continue;
		if (!dep_idx.AreIndependent(nd->inputs[0], nd->inputs[1])) continue;

		ForkPoint fp;
		fp.bool_ref = ref;
		fp.inA = nd->inputs[0];
		fp.inB = nd->inputs[1];
		fp.depsA = g.CollectDeps(fp.inA);
		fp.depsB = g.CollectDeps(fp.inB);
		for (auto id : fp.depsA) deferred.insert(id);
		for (auto id : fp.depsB) deferred.insert(id);
		forks.push_back(std::move(fp));
	}

	// map boolean node id -> fork index
	std::unordered_map<uint32_t, size_t> fork_map;
	for (size_t i = 0; i < forks.size(); ++i)
		fork_map[forks[i].bool_ref.id] = i;

	std::unordered_set<uint32_t> done;

	for (auto ref : order)
	{
		if (done.count(ref.id)) continue;

		// skip nodes belonging to a fork's subtree -- they'll be evaluated in parallel
		if (deferred.count(ref.id) && !fork_map.count(ref.id)) continue;

		auto fit = fork_map.find(ref.id);
		if (fit != fork_map.end())
		{
			auto& fp = forks[fit->second];

			{
				// each subtree gets its own tn + evaluator, runs in parallel
				auto tnA = (tn && tn_factory) ? tn_factory() : nullptr;
				auto tnB = (tn && tn_factory) ? tn_factory() : nullptr;
				Evaluator evalA(m_reg);
				Evaluator evalB(m_reg);
				evalA.SetCommitFn(m_commit_fn);
				evalA.SetRestoreFn(m_restore_fn);
				evalB.SetCommitFn(m_commit_fn);
				evalB.SetRestoreFn(m_restore_fn);

				// Seed sub-evaluator caches for fully-clean branches
				// so that unchanged ops are not re-evaluated.
				// Branches with any dirty node are left un-seeded:
				// every op re-runs and registers with the branch tn.
				auto seed_if_clean = [&](const std::unordered_set<uint32_t>& deps,
				                         Evaluator& eval) {
					for (auto id : deps) {
						auto* nd = g.Get(NRef{id});
						if (nd && nd->eval_version != nd->version)
							return;  // dirty node found -- don't seed
					}
					for (auto id : deps) {
						auto* v = m_shape_cache.Get(id);
						if (v) eval.GetShapeCache().Put(id, *v);
					}
				};
				seed_if_clean(fp.depsA, evalA);
				seed_if_clean(fp.depsB, evalB);

				// Demand-driven: just call EvalNode on each branch's root
				// (fp.inA / fp.inB). EvalNode recurses only into nodes
				// whose values are actually needed -- clean nodes that
				// hit the fast path return without visiting their inputs.
				auto futB = std::async(std::launch::async,
					[&evalB, &g, &fp, &tnB]() {
						evalB.EvalNode(g, fp.inB, tnB);
					});
				evalA.EvalNode(g, fp.inA, tnA);
				futB.get();

				// merge shape caches
				for (auto id : fp.depsA) {
					auto* v = evalA.GetShapeCache().Get(id);
					if (v) m_shape_cache.Put(id, *v);
				}
				for (auto id : fp.depsB) {
					auto* v = evalB.GetShapeCache().Get(id);
					if (v) m_shape_cache.Put(id, *v);
				}

				// merge naming state into main tn
				if (tn && tn_merge) {
					if (tnA) tn_merge(tn, tnA);
					if (tnB) tn_merge(tn, tnB);
				}
			}

			for (auto id : fp.depsA) done.insert(id);
			for (auto id : fp.depsB) done.insert(id);
		}

		EvalNode(g, ref, tn);
		done.insert(ref.id);
	}

	return ResolveVal(g, root);
}

void Evaluator::Invalidate(IRGraph& g, NRef ref)
{
	// O(1): bump self version + drop self cache + drop vt link. Bumping
	// the global eval-epoch tells EvalNode it can no longer trust any
	// node's "validated" stamp -- downstream staleness will be discovered
	// lazily by the recursive demand-driven path on the next Eval.
	auto* nd = g.Get(ref);
	if (!nd) return;
	nd->version++;
	nd->vt_node_id = UINT32_MAX;
	m_shape_cache.Remove(ref.id);
	m_eval_epoch++;
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

void OpHistory::SetStepVtNodeId(int step_id, uint32_t vt_node_id)
{
	if (step_id >= 0 && step_id < (int)m_steps.size())
		m_steps[step_id].vt_node_id = vt_node_id;
}

void OpHistory::Truncate(size_t keep)
{
	if (keep < m_steps.size())
		m_steps.resize(keep);
}

const OpStep* OpHistory::Get(int step_id) const
{
	if (step_id >= 0 && step_id < (int)m_steps.size())
		return &m_steps[step_id];
	return nullptr;
}

// ---------------------------------------------------------------
//  OpHistory serialization
// ---------------------------------------------------------------

static constexpr uint8_t VAL_MONO   = 0;
static constexpr uint8_t VAL_INT    = 1;
static constexpr uint8_t VAL_DOUBLE = 2;
static constexpr uint8_t VAL_BOOL   = 3;
static constexpr uint8_t VAL_VEC3   = 4;
static constexpr uint8_t VAL_SHAPE  = 5;

static constexpr uint32_t HIST_MAGIC   = 0x48495354;  // "HIST"
static constexpr uint32_t HIST_VERSION = 1;

struct HistWriter
{
	std::vector<uint8_t> buf;

	void Write(const void* data, size_t size)
	{
		const auto* p = reinterpret_cast<const uint8_t*>(data);
		buf.insert(buf.end(), p, p + size);
	}

	void W8(uint8_t v) { Write(&v, 1); }
	void W32(uint32_t v) { Write(&v, 4); }

	void WStr(const std::string& s)
	{
		W32(static_cast<uint32_t>(s.size()));
		if (!s.empty())
			Write(s.data(), s.size());
	}

	void WVal(const Val& v)
	{
		uint8_t idx = static_cast<uint8_t>(v.index());
		W8(idx);

		switch (idx)
		{
		case VAL_INT:
		{
			int32_t iv = std::get<int>(v);
			Write(&iv, 4);
			break;
		}
		case VAL_DOUBLE:
		{
			double dv = std::get<double>(v);
			Write(&dv, 8);
			break;
		}
		case VAL_BOOL:
		{
			uint8_t bv = std::get<bool>(v) ? 1 : 0;
			W8(bv);
			break;
		}
		case VAL_VEC3:
		{
			auto& vec = std::get<Vec3>(v);
			Write(vec.data(), 3 * sizeof(double));
			break;
		}
		default:
			break;
		}
	}
};

struct HistReader
{
	const uint8_t* data = nullptr;
	uint32_t size = 0;
	uint32_t pos  = 0;
	bool     ok   = true;

	bool Read(void* dst, size_t n)
	{
		if (!ok || pos + n > size) { ok = false; return false; }
		std::memcpy(dst, data + pos, n);
		pos += static_cast<uint32_t>(n);
		return true;
	}

	uint8_t R8()
	{
		uint8_t v = 0;
		Read(&v, 1);
		return v;
	}

	uint32_t R32()
	{
		uint32_t v = 0;
		Read(&v, 4);
		return v;
	}

	std::string RStr()
	{
		uint32_t len = R32();
		if (!ok || len == 0)
			return {};
		std::string s(len, '\0');
		Read(s.data(), len);
		return s;
	}

	Val RVal()
	{
		uint8_t idx = R8();
		if (!ok) return Val{};

		switch (idx)
		{
		case VAL_INT:
		{
			int32_t iv;
			Read(&iv, 4);
			return Val(static_cast<int>(iv));
		}
		case VAL_DOUBLE:
		{
			double dv;
			Read(&dv, 8);
			return Val(dv);
		}
		case VAL_BOOL:
		{
			uint8_t bv = R8();
			return Val(static_cast<bool>(bv));
		}
		case VAL_VEC3:
		{
			Vec3 vec;
			Read(vec.data(), 3 * sizeof(double));
			return Val(vec);
		}
		default:
			return Val{};
		}
	}
};

void OpHistory::StoreToByteArray(uint8_t** buf, uint32_t& len) const
{
	HistWriter w;

	w.W32(HIST_MAGIC);
	w.W32(HIST_VERSION);
	w.W32(static_cast<uint32_t>(m_steps.size()));

	for (const auto& step : m_steps)
	{
		w.W32(static_cast<uint32_t>(step.step_id));
		w.WStr(step.op_name);
		w.WVal(step.imm);

		w.W32(static_cast<uint32_t>(step.inputs.size()));
		for (int inp : step.inputs)
			w.W32(static_cast<uint32_t>(inp));

		w.W32(static_cast<uint32_t>(step.var_inputs.size()));
		for (int inp : step.var_inputs)
			w.W32(static_cast<uint32_t>(inp));

		w.WStr(step.desc);
		w.W32(step.vt_node_id);
	}

	len  = static_cast<uint32_t>(w.buf.size());
	*buf = new uint8_t[len];
	std::memcpy(*buf, w.buf.data(), len);
}

bool OpHistory::LoadFromByteArray(const uint8_t* buf, uint32_t len)
{
	HistReader r;
	r.data = buf;
	r.size = len;

	uint32_t magic   = r.R32();
	uint32_t version = r.R32();
	if (!r.ok || magic != HIST_MAGIC || version > HIST_VERSION)
		return false;

	uint32_t count = r.R32();
	if (!r.ok) return false;

	m_steps.clear();
	m_steps.reserve(count);

	for (uint32_t i = 0; i < count; ++i)
	{
		OpStep step;
		step.step_id = static_cast<int>(r.R32());
		step.op_name = r.RStr();
		step.imm     = r.RVal();

		uint32_t inp_count = r.R32();
		step.inputs.resize(inp_count);
		for (uint32_t j = 0; j < inp_count; ++j)
			step.inputs[j] = static_cast<int>(r.R32());

		uint32_t var_count = r.R32();
		step.var_inputs.resize(var_count);
		for (uint32_t j = 0; j < var_count; ++j)
			step.var_inputs[j] = static_cast<int>(r.R32());

		step.desc = r.RStr();
		step.vt_node_id = r.R32();

		if (!r.ok)
		{
			m_steps.clear();
			return false;
		}

		m_steps.push_back(std::move(step));
	}

	return true;
}

} // namespace breptopo
