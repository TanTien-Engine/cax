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

Val Evaluator::ResolveVal(const IRGraph& g, NRef ref) const
{
	auto* nd = g.Get(ref);
	if (!nd) return {};
	// non-shape values live in IRNode::cached
	if (!std::holds_alternative<std::monostate>(nd->cached))
		return nd->cached;
	// shape values: try in-memory LRU first
	auto* v = m_shape_cache.Get(ref.id);
	if (v) return *v;
	// LRU miss: try restoring from VersionTree
	if (nd->vt_node_id != UINT32_MAX && m_restore_fn)
	{
		Val restored = m_restore_fn(nd->vt_node_id);
		if (!std::holds_alternative<std::monostate>(restored))
		{
			m_shape_cache.Put(ref.id, restored);
			m_restores++;
			return restored;
		}
	}
	return {};
}

Val Evaluator::EvalNode(IRGraph& g, NRef ref,
                        const std::shared_ptr<TopoNaming>& tn,
                        uint32_t& op_counter)
{
	auto* nd = g.Get(ref);
	if (!nd) return {};

	if (!nd->op_name.empty() && nd->op_name[0] == '$')
	{
		if (std::holds_alternative<ShapeVal>(nd->imm))
		{
			m_shape_cache.Put(ref.id, nd->imm);
			if (m_commit_fn && nd->vt_node_id == UINT32_MAX)
				nd->vt_node_id = m_commit_fn(ref.id, nd->imm);
		}
		else
		{
			nd->cached = nd->imm;
		}
		nd->eval_version = nd->version;
		return nd->imm;
	}

	uint64_t ih = InputHash(g, *nd);
	if (nd->eval_version == nd->version && ih != 0)
	{
		Val resolved = ResolveVal(g, ref);
		if (!std::holds_alternative<std::monostate>(resolved))
		{
			m_hits++;
			op_counter++;
			return resolved;
		}
		// shape was evicted from LRU (and VersionTree restore not yet
		// available) -- fall through to re-evaluate
	}

	std::vector<Val> resolved;
	for (auto& inp : nd->inputs)
		resolved.push_back(ResolveVal(g, inp));
	std::vector<Val> var_resolved;
	for (auto& inp : nd->var_inputs)
		var_resolved.push_back(ResolveVal(g, inp));

	auto* desc = m_reg.Find(nd->op_name);
	Val result;
	if (desc && desc->eval)
	{
		EvalCtx ctx{resolved, var_resolved, tn, op_counter++};
		result = desc->eval(ctx);
	}

	if (std::holds_alternative<ShapeVal>(result))
	{
		m_shape_cache.Put(ref.id, result);
		// optional: persist for future restore (e.g. to VersionTree)
		if (m_commit_fn)
			nd->vt_node_id = m_commit_fn(ref.id, result);
	}
	else
	{
		nd->cached = result;
	}

	nd->eval_version = nd->version;
	m_misses++;
	return result;
}

Val Evaluator::Run(IRGraph& g, NRef root, const std::shared_ptr<TopoNaming>& tn)
{
	uint32_t op_counter = 0;
	for (auto ref : g.TopoSort())
		EvalNode(g, ref, tn, op_counter);
	return ResolveVal(g, root);
}

void Evaluator::EvalSubtree(IRGraph& g, NRef root,
                            const std::shared_ptr<TopoNaming>& tn,
                            uint32_t& op_counter)
{
	auto deps = g.CollectDeps(root);
	auto order = g.TopoSort();
	for (auto ref : order)
		if (deps.count(ref.id))
			EvalNode(g, ref, tn, op_counter);
}

Val Evaluator::RunParallel(IRGraph& g, NRef root,
                           const std::shared_ptr<TopoNaming>& tn,
                           TnFactory tn_factory, TnMerge tn_merge)
{
	auto dep_idx = g.BuildDepIndex();
	auto order = g.TopoSort();
	uint32_t op_counter = 0;

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

			std::vector<NRef> orderA, orderB;
			for (auto r : order)
			{
				if (fp.depsA.count(r.id) && !done.count(r.id)) orderA.push_back(r);
				if (fp.depsB.count(r.id) && !done.count(r.id)) orderB.push_back(r);
			}

			{
				// each subtree gets its own tn + evaluator, runs in parallel
				auto tnA = (tn && tn_factory) ? tn_factory() : nullptr;
				auto tnB = (tn && tn_factory) ? tn_factory() : nullptr;
				Evaluator evalA(m_reg);
				Evaluator evalB(m_reg);
				uint32_t opA = op_counter, opB = op_counter;

				auto futB = std::async(std::launch::async,
					[&evalB, &g, &orderB, &tnB, &opB]() {
						for (auto r : orderB) evalB.EvalNode(g, r, tnB, opB);
					});
				for (auto r : orderA) evalA.EvalNode(g, r, tnA, opA);
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
				op_counter = std::max(opA, opB);

				// merge naming state into main tn
				if (tn && tn_merge) {
					if (tnA) tn_merge(tn, tnA);
					if (tnB) tn_merge(tn, tnB);
				}
			}

			for (auto id : fp.depsA) done.insert(id);
			for (auto id : fp.depsB) done.insert(id);
		}

		EvalNode(g, ref, tn, op_counter);
		done.insert(ref.id);
	}

	return ResolveVal(g, root);
}

void Evaluator::Invalidate(IRGraph& g, NRef ref)
{
	auto* nd = g.Get(ref);
	if (!nd) return;
	nd->version++;
	nd->vt_node_id = UINT32_MAX;  // stale shape, must re-commit after re-eval
	m_shape_cache.Remove(ref.id);
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

} // namespace breptopo
