#pragma once

#include "ShapeCache.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>
#include <map>

namespace partgraph { class TopoShape; }
namespace graph { class Graph; }

namespace breptopo
{

class TopoNaming;

// ---------------------------------------------------------------
//  NRef -- stable typed handle for IR nodes
// ---------------------------------------------------------------

struct NRef
{
	uint32_t id = 0;
	bool valid() const { return id != 0; }
	bool operator==(NRef o) const { return id == o.id; }
	bool operator!=(NRef o) const { return id != o.id; }
};

struct NRefHash { size_t operator()(NRef r) const { return std::hash<uint32_t>{}(r.id); } };

static constexpr NRef NREF_NULL{ 0 };

// ---------------------------------------------------------------
//  Val -- variant value flowing through the graph
// ---------------------------------------------------------------

using Vec3 = std::array<double, 3>;

struct ShapeVal
{
	std::shared_ptr<partgraph::TopoShape> shape;
	uint32_t tag = 0;
	bool operator==(const ShapeVal& o) const { return tag == o.tag; }
};

using Val = std::variant<std::monostate, int, double, bool, Vec3, ShapeVal>;

// ---------------------------------------------------------------
//  EvalCtx -- passed to operation eval functions
// ---------------------------------------------------------------

struct EvalCtx
{
	const std::vector<Val>& inputs;
	const std::vector<Val>& var_inputs;
	std::shared_ptr<TopoNaming> tn;
	uint32_t op_id = 0;

	double Num(size_t i) const;
	int    Int(size_t i) const;
	bool   Bool(size_t i) const;
	Vec3   GetVec3(size_t i) const;
	ShapeVal GetShape(size_t i) const;
	std::vector<ShapeVal> VarShapes() const;
};

// ---------------------------------------------------------------
//  OpDesc + OpRegistry
// ---------------------------------------------------------------

using EvalFn = std::function<Val(EvalCtx&)>;

struct OpFlags
{
	bool is_dressup  = false;
	bool is_pattern  = false;
	bool is_boolean  = false;
	bool is_const    = false;
	// Skip committing the result to the VersionTree. Within a session the
	// result is still kept in m_shape_cache, so subsequent evals are fast.
	// After reload, vt_node_id == UINT32_MAX (not stored), so the op is
	// re-executed once to populate the in-session cache from current state.
	bool no_vt_cache = false;
};

struct OpDesc
{
	std::string name;
	std::vector<std::string> input_names;
	std::vector<std::string> var_input_names;
	EvalFn eval;
	OpFlags flags;
};

class OpRegistry
{
public:
	void Define(const std::string& name,
	            std::vector<std::string> inputs,
	            std::vector<std::string> var_inputs,
	            EvalFn eval, OpFlags flags = {});
	const OpDesc* Find(const std::string& name) const;

private:
	std::vector<OpDesc> m_ops;
	std::unordered_map<std::string, size_t> m_index;
};

// ---------------------------------------------------------------
//  IRNode + IRGraph
// ---------------------------------------------------------------

struct IRNode
{
	NRef        ref;
	std::string op_name;
	Val         imm = {};
	std::vector<NRef> inputs;
	std::vector<NRef> var_inputs;
	bool        dead = false;
	uint64_t    version      = 1;
	uint64_t    eval_version = 0;
	Val         cached       = {};         // non-shape values only
	uint32_t    vt_node_id   = UINT32_MAX; // VersionTree node for shape restore
	uint32_t    op_id        = UINT32_MAX; // deterministic, assigned by AssignOpIds
};

class IRGraph
{
public:
	explicit IRGraph(const OpRegistry& reg) : m_reg(reg) {}

	NRef Const(int v);
	NRef Const(double v);
	NRef Const(bool v);
	NRef Const(Vec3 v);
	NRef Const(ShapeVal v);
	NRef Add(const std::string& op,
	         const std::vector<NRef>& inputs,
	         const std::vector<NRef>& var_inputs = {});

	void UpdateImmediate(NRef ref, Val v);
	void Kill(NRef ref);
	void Erase(NRef ref);
	void ReplaceAllUses(NRef old_ref, NRef new_ref);
	void Compact();

	const IRNode* Get(NRef ref) const;
	IRNode*       Get(NRef ref);
	size_t        LiveCount() const;
	std::vector<NRef> TopoSort() const;
	std::vector<NRef> UsersOf(NRef ref) const;
	std::unordered_set<uint32_t> CollectDeps(NRef ref) const;
	bool AreIndependent(NRef a, NRef b) const;
	std::vector<std::vector<NRef>> TopoLevels() const;
	void AssignOpIds();

	// O(n) precompute, O(leaves/64) per query alternative to AreIndependent.
	// Each node stores a bitset of which leaves it transitively depends on.
	// Two nodes are independent iff their bitsets AND to zero.
	class DepIndex
	{
	public:
		void Build(const IRGraph& g);
		bool AreIndependent(NRef a, NRef b) const;
	private:
		using Word = uint64_t;
		size_t m_words = 0;
		std::unordered_map<uint32_t, std::vector<Word>> m_bits;
	};
	DepIndex BuildDepIndex() const;
	void Clear() { m_nodes.clear(); m_next_id = 1; }
	const OpRegistry& Registry() const { return m_reg; }
	std::string Dump() const;

private:
	const OpRegistry& m_reg;
	uint32_t m_next_id = 1;
	std::unordered_map<uint32_t, IRNode> m_nodes;
	NRef Alloc();
};

// ---------------------------------------------------------------
//  Optimizer
// ---------------------------------------------------------------

struct MatchResult
{
	NRef root = NREF_NULL;
	std::unordered_map<std::string, NRef> caps;
	NRef operator[](const std::string& name) const
	{
		auto it = caps.find(name);
		return it != caps.end() ? it->second : NREF_NULL;
	}
};

using MatchPred = std::function<bool(const IRNode&, const OpDesc*)>;
using RewriteFn = std::function<bool(MatchResult&, IRGraph&)>;

struct PatternStep
{
	MatchPred   pred;
	std::string capture;
	int         follow_input = -1;
};

struct RewriteRule
{
	std::string name;
	std::vector<PatternStep> pattern;
	RewriteFn rewrite;
	int priority = 0;
};

MatchPred ByName(const std::string& name);
MatchPred ByFlag(std::function<bool(const OpFlags&)> test);
MatchPred Any();

class Optimizer
{
public:
	void AddRule(RewriteRule rule);
	void AddDefaultRules();
	bool Run(IRGraph& g, int max_iter = 16) const;
	static bool DCE(IRGraph& g);
	static bool CSE(IRGraph& g);

private:
	std::vector<RewriteRule> m_rules;
	bool TryMatch(const IRGraph& g, NRef start,
	              const RewriteRule& rule, MatchResult& m) const;
};

// ---------------------------------------------------------------
//  Evaluator
// ---------------------------------------------------------------

// Callback: persist a shape result, return a version-tree node id.
//   (nref_id, val) -> vt_node_id   (UINT32_MAX = not stored)
using CommitFn  = std::function<uint32_t(uint32_t nref_id, const Val& val)>;

// Callback: restore a shape from version-tree node id.
//   (vt_node_id, tn) -> Val            (monostate = restore failed)
// The current TopoNaming is passed so the callback can bind the restored
// shape's sub-shapes (via WorldReceiver's uid cache) into the correct
// HistGraph — important for parallel eval where each fork has its own tn.
using RestoreFn = std::function<Val(uint32_t vt_node_id,
                                    const std::shared_ptr<TopoNaming>& tn)>;

class Evaluator
{
public:
	Evaluator(const OpRegistry& reg) : m_reg(reg) {}

	using TnFactory = std::function<std::shared_ptr<TopoNaming>()>;
	using TnMerge   = std::function<void(const std::shared_ptr<TopoNaming>& dst,
	                                     const std::shared_ptr<TopoNaming>& src)>;

	Val Run(IRGraph& g, NRef root, const std::shared_ptr<TopoNaming>& tn);
	Val RunParallel(IRGraph& g, NRef root, const std::shared_ptr<TopoNaming>& tn,
	                TnFactory tn_factory = {}, TnMerge tn_merge = {});
	void Invalidate(IRGraph& g, NRef ref);

	Val ResolveVal(const IRGraph& g, NRef ref,
	               const std::shared_ptr<TopoNaming>& tn = nullptr) const;

	LruCache<Val>&       GetShapeCache()       { return m_shape_cache; }
	const LruCache<Val>& GetShapeCache() const { return m_shape_cache; }

	void SetCommitFn(CommitFn fn)   { m_commit_fn = std::move(fn); }
	void SetRestoreFn(RestoreFn fn) { m_restore_fn = std::move(fn); }

	size_t CacheHits()    const { return m_hits; }
	size_t CacheMisses()  const { return m_misses; }
	size_t CacheRestores() const { return m_restores; }
	void ResetStats() { m_hits = m_misses = m_restores = 0; }

private:
	const OpRegistry& m_reg;
	mutable LruCache<Val> m_shape_cache{64};
	CommitFn  m_commit_fn;
	RestoreFn m_restore_fn;
	size_t m_hits = 0, m_misses = 0;
	mutable size_t m_restores = 0;

	Val EvalNode(IRGraph& g, NRef ref,
	             const std::shared_ptr<TopoNaming>& tn);
	void EvalSubtree(IRGraph& g, NRef root,
	                 const std::shared_ptr<TopoNaming>& tn);
	uint64_t InputHash(const IRGraph& g, const IRNode& node) const;
};

// ---------------------------------------------------------------
//  OpStep -- records one user operation (design-intent level)
// ---------------------------------------------------------------

struct OpStep
{
	int                step_id = -1;
	std::string        op_name;
	Val                imm = {};
	std::vector<int>   inputs;
	std::vector<int>   var_inputs;
	std::string        desc;
	uint32_t           vt_node_id = UINT32_MAX;
};

// ---------------------------------------------------------------
//  OpHistory -- ordered list of user operations
//
//  This is the "source of truth" for the user's design intent.
//  Each blueprint node's compile_graph() appends steps here.
//  The lowering pass converts this into an optimisable IRGraph.
// ---------------------------------------------------------------

class OpHistory
{
public:
	int AddConst(int v, const std::string& desc);
	int AddConst(double v, const std::string& desc);
	int AddConst(bool v, const std::string& desc);
	int AddConst(Vec3 v, const std::string& desc);
	int AddConst(const std::shared_ptr<partgraph::TopoShape>& shp, const std::string& desc);
	int AddOp(const std::string& op,
	          const std::vector<int>& inputs,
	          const std::vector<int>& var_inputs = {},
	          const std::string& desc = "");

	void UpdateConst(int step_id, Val v);
	void SetStepVtNodeId(int step_id, uint32_t vt_node_id);
	void Truncate(size_t keep);

	const OpStep* Get(int step_id) const;
	size_t Size() const { return m_steps.size(); }
	const std::vector<OpStep>& Steps() const { return m_steps; }

	void Clear() { m_steps.clear(); }

	void StoreToByteArray(uint8_t** buf, uint32_t& len) const;
	bool LoadFromByteArray(const uint8_t* buf, uint32_t len);

private:
	std::vector<OpStep> m_steps;
	int Append(OpStep step);
};

// ---------------------------------------------------------------
//  CompGraph -- top-level facade
//
//  Wraps OpHistory + IRGraph + Optimizer + Evaluator.
//  OpHistory records the user's design intent.
//  Lower() converts OpHistory -> IRGraph (the optimisable IR).
//  Optimize() rewrites the IR.
//  Eval() executes.
// ---------------------------------------------------------------

class CompGraph
{
public:
	CompGraph();

	// --- record operations (design-intent level) ---
	int AddConst(int v, const std::string& desc);
	int AddConst(double v, const std::string& desc);
	int AddConst(bool v, const std::string& desc);
	int AddConst(Vec3 v, const std::string& desc);
	int AddConst(const std::shared_ptr<partgraph::TopoShape>& shp, const std::string& desc);
	int AddOp(const std::string& op,
	          const std::vector<int>& inputs,
	          const std::vector<int>& var_inputs = {},
	          const std::string& desc = "");

	// --- mutation ---
	void UpdateConst(int ext_id, Val v);
	void Truncate(size_t keep);

	// --- lowering + optimisation ---
	void Lower();
	void Lower(int root_step);
	void Optimize();

	// --- shape restore (version tree integration) ---
	//
	// The external design system manages the VersionTree. After it
	// commits a shape, it calls SetNodeVersion() to record the mapping.
	// SetRestoreFn() tells the Evaluator how to recover an evicted shape.
	//
	void SetRestoreFn(RestoreFn fn);
	void SetCommitFn(CommitFn fn);
	void SetNodeVersion(int ext_id, uint32_t vt_node_id);

	// --- evaluation ---
	void SetParallel(bool enabled) { m_parallel = enabled; }
	bool IsParallel() const { return m_parallel; }
	Val Eval(int ext_id);

	// --- query ---
	size_t   NodeCount() const;
	uint32_t CalcOpId(int ext_id, int sub_op_id) const;
	NRef     Ref(int ext_id) const;

	// --- reconnection support ---
	size_t GetHistorySize() const { return m_history.Size(); }
	const std::string& GetStepOpName(int step_id) const;
	std::vector<int> GetStepInputs(int step_id) const;
	void ClaimStep(int step_id) { m_claimed_steps.insert(step_id); }
	bool IsStepClaimed(int step_id) const { return m_claimed_steps.count(step_id) > 0; }
	void ClearClaims() { m_claimed_steps.clear(); }
	bool HasPreloadedHistory() const { return m_preloaded; }

	// --- access internals ---
	OpHistory&           GetHistory()        { return m_history; }
	const OpHistory&     GetHistory()  const { return m_history; }
	IRGraph&       GetIR()       { return m_ir; }
	const IRGraph& GetIR() const { return m_ir; }
	OpRegistry&    GetRegistry() { return m_reg; }
	Optimizer&     GetOptimizer(){ return m_opt; }
	auto           GetTopoNaming() const { return m_tn; }
	void           SetTopoNaming(const std::shared_ptr<TopoNaming>& tn) { m_tn = tn; }

	void StoreToByteArray(uint8_t** buf, uint32_t& len) const;
	bool LoadFromByteArray(const uint8_t* buf, uint32_t len);

	std::string Dump() const { return m_ir.Dump(); }

private:
	OpRegistry m_reg;
	OpHistory  m_history;
	IRGraph    m_ir;
	Optimizer  m_opt;
	Evaluator  m_eval;

	std::shared_ptr<TopoNaming>          m_tn;

	struct NodeMeta
	{
		NRef        ref;
		std::string desc;
	};
	std::vector<NodeMeta> m_nodes;
	std::unordered_map<uint32_t, int> m_ref2ext;

	mutable std::map<std::pair<int,int>, uint32_t> m_op_id_map;
	std::unordered_set<int> m_claimed_steps;

	bool m_lowered = false;
	bool m_preloaded = false;
	bool m_parallel = false;
	size_t m_lowered_count = 0;

	int Register(NRef ref, const std::string& desc);
	void RebuildIR();
	void AppendNewSteps();
};

} // namespace breptopo
