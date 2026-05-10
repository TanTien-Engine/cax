#include "comp_ops.h"
#include "CompGraph.h"
#include "TopoNaming.h"
#include "HistGraph.h"
#include "NodeShape.h"

#include "partgraph_c/PrimMaker.h"
#include "partgraph_c/TopoAlgo.h"
#include "partgraph_c/TopoShape.h"

#include <graph/Node.h>

namespace breptopo
{

static ShapeVal MakeShapeVal(const std::shared_ptr<partgraph::TopoShape>& shp)
{
	ShapeVal sv;
	sv.shape = shp;
	return sv;
}

void RegisterBuiltinOps(OpRegistry& reg)
{
	reg.Define("box", {"length", "width", "height"}, {},
		[](EvalCtx& ctx) -> Val {
			double dx = ctx.Num(0);
			double dy = ctx.Num(1);
			double dz = ctx.Num(2);
			auto shape = partgraph::PrimMaker::Box(dx, dy, dz, ctx.op_id, ctx.tn);
			return MakeShapeVal(shape);
		});

	reg.Define("translate", {"shape", "offset"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			auto off = ctx.GetVec3(1);
			if (!sv.shape) return {};
			auto shp = partgraph::TopoAlgo::Translate(sv.shape, off[0], off[1], off[2], ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		});

	reg.Define("offset", {"shape", "offset", "is_solid"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			double off = ctx.Num(1);
			bool is_solid = ctx.Bool(2);
			auto shp = partgraph::TopoAlgo::OffsetShape(sv.shape, static_cast<float>(off), is_solid, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		},
		{true, false, false, false});  // is_dressup

	reg.Define("cut", {"shape1", "shape2"}, {},
		[](EvalCtx& ctx) -> Val {
			auto s1 = ctx.GetShape(0);
			auto s2 = ctx.GetShape(1);
			if (!s1.shape || !s2.shape) return {};
			auto shp = partgraph::TopoAlgo::Cut(s1.shape, s2.shape, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		},
		{false, false, true, false});  // is_boolean

	reg.Define("fuse", {"shape1", "shape2"}, {},
		[](EvalCtx& ctx) -> Val {
			auto s1 = ctx.GetShape(0);
			auto s2 = ctx.GetShape(1);
			if (!s1.shape || !s2.shape) return {};
			auto shp = partgraph::TopoAlgo::Fuse(s1.shape, s2.shape, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		},
		{false, false, true, false});  // is_boolean

	reg.Define("fillet", {"shape", "radius"}, {"edges"},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			double radius = ctx.Num(1);
			auto edge_vals = ctx.VarShapes();
			std::vector<std::shared_ptr<partgraph::TopoShape>> edges;
			for (auto& ev : edge_vals) {
				if (ev.shape) edges.push_back(ev.shape);
			}
			auto shp = partgraph::TopoAlgo::Fillet(sv.shape, radius, edges, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		},
		{true, false, false, false});  // is_dressup

	reg.Define("chamfer", {"shape", "dist"}, {"edges"},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			double dist = ctx.Num(1);
			auto edge_vals = ctx.VarShapes();
			std::vector<std::shared_ptr<partgraph::TopoShape>> edges;
			for (auto& ev : edge_vals) {
				if (ev.shape) edges.push_back(ev.shape);
			}
			auto shp = partgraph::TopoAlgo::Chamfer(sv.shape, dist, edges, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		},
		{true, false, false, false});  // is_dressup

	reg.Define("selector_edge", {"shape", "uid"}, {},
		[](EvalCtx& ctx) -> Val {
			uint32_t uid = static_cast<uint32_t>(ctx.Num(1));
			if (!ctx.tn) return {};
			std::vector<std::shared_ptr<graph::Node>> nodes;
			if (!ctx.tn->GetEdgeGraph()->QueryNodes(uid, nodes)) return {};
			if (nodes.empty()) return {};
			auto shp = nodes[0]->GetComponent<NodeShape>().GetShape();
			return MakeShapeVal(shp);
		});

	reg.Define("selector_face", {"shape", "uid"}, {},
		[](EvalCtx& ctx) -> Val {
			uint32_t uid = static_cast<uint32_t>(ctx.Num(1));
			if (!ctx.tn) return {};
			std::vector<std::shared_ptr<graph::Node>> nodes;
			if (!ctx.tn->GetFaceGraph()->QueryNodes(uid, nodes)) return {};
			if (nodes.empty()) return {};
			auto shp = nodes[0]->GetComponent<NodeShape>().GetShape();
			return MakeShapeVal(shp);
		});

	reg.Define("merge", {}, {"shapes"},
		[](EvalCtx& ctx) -> Val {
			auto shapes = ctx.VarShapes();
			if (shapes.empty()) return {};
			if (shapes.size() == 1) return shapes[0];
			return shapes[0];
		});
}

} // namespace breptopo
