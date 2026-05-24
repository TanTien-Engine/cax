#include "cadapp_c/ops/resolve_ops.h"
#include "cadapp_c/ir/TopoRefIR.h"
#include "cadapp_c/resolve/TopoRefResolver.h"

#include "brepgraph_c/computation/CalcGraph.h"

#include "brepkit_c/TopoShape.h"

#include <TopoDS_Shape.hxx>
#include <TopExp.hxx>
#include <TopAbs.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include <cstdio>

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
		// Diagnostic: failed resolution is silent by default and
		// debugging "Fillet did nothing" is painful without it.
		// Print the ref centroid + the closest match distance the
		// resolver actually computed so the caller can see whether
		// it was just over tolerance or completely off.
		double dist = resolved.empty()
			? -1.0
			: resolved[0].match_dist;
		std::fprintf(stderr,
			"[resolve_%s] MISS ref_pt=(%.4f,%.4f,%.4f) "
			"best_dist=%.4f tol=%.4f\n",
			(kind == TopAbs_FACE) ? "face" : "edge",
			ref->point[0], ref->point[1], ref->point[2],
			dist, tolerance);
		return {};
	}

	TopTools_IndexedMapOfShape m;
	TopExp::MapShapes(sv.shape->GetShape(), kind, m);
	if (resolved[0].topo_index > m.Extent()) {
		std::fprintf(stderr,
			"[resolve_%s] INDEX_OOR ref_pt=(%.4f,%.4f,%.4f) "
			"idx=%d map_extent=%d\n",
			(kind == TopAbs_FACE) ? "face" : "edge",
			ref->point[0], ref->point[1], ref->point[2],
			resolved[0].topo_index, m.Extent());
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
