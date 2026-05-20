#include "cadapp_c/ops/resolve_ops.h"
#include "cadapp_c/ir/TopoRefIR.h"
#include "cadapp_c/resolve/TopoRefResolver.h"

#include "brepgraph_c/CompGraph.h"

#include "brepkit_c/TopoShape.h"

#include <TopoDS_Shape.hxx>
#include <TopExp.hxx>
#include <TopAbs.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

namespace cadapp
{

namespace
{

using brepgraph::EvalCtx;
using brepgraph::ShapeVal;
using brepgraph::Val;

ShapeVal ResolveOne(EvalCtx& ctx, TopAbs_ShapeEnum kind)
{
	auto sv = ctx.GetShape(0);
	if (!sv.shape) return {};

	auto rv = ctx.GetTopoRef(1);
	if (!rv.handle) return {};
	const auto* ref = static_cast<const TopoRefIR*>(rv.handle.get());

	double tolerance = ctx.Num(2);
	if (tolerance <= 0.0) tolerance = 1e-3;

	std::vector<TopoRefIR> one{ *ref };
	auto resolved = TopoRefResolver::Resolve(
		sv.shape->GetShape(), one, ctx.tn.get(), tolerance);

	if (resolved.empty() || resolved[0].topo_index <= 0) {
		return {};
	}

	TopTools_IndexedMapOfShape m;
	TopExp::MapShapes(sv.shape->GetShape(), kind, m);
	if (resolved[0].topo_index > m.Extent()) {
		return {};
	}

	ShapeVal out;
	out.shape = std::make_shared<brepkit::TopoShape>(
		m.FindKey(resolved[0].topo_index));
	out.tag = resolved[0].uid;   // 0 means TopoNaming had no binding
	return out;
}

} // anonymous namespace

void RegisterResolveOps(brepgraph::OpRegistry& reg)
{
	// no_vt_cache: the resolved sub-shape depends on the upstream
	// shape's transient identity (topo_index). Caching the result in
	// the VersionTree would survive across sessions where the upstream
	// shape may have rebuilt with a different MapShapes order, so the
	// stale sub-shape would be wrong. Within a session the LruCache
	// still memoizes the result.
	reg.Define("resolve_edge_ref", {"shape", "ref", "tolerance"}, {},
		[](EvalCtx& ctx) -> Val {
			return ResolveOne(ctx, TopAbs_EDGE);
		},
		{false, false, false, false, true});

	reg.Define("resolve_face_ref", {"shape", "ref", "tolerance"}, {},
		[](EvalCtx& ctx) -> Val {
			return ResolveOne(ctx, TopAbs_FACE);
		},
		{false, false, false, false, true});
}

} // namespace cadapp
