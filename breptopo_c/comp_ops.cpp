#include "comp_ops.h"
#include "CompGraph.h"
#include "TopoNaming.h"
#include "HistGraph.h"
#include "NodeShape.h"

#include "partgraph_c/PrimMaker.h"
#include "partgraph_c/TopoAlgo.h"
#include "partgraph_c/TopoAlgo_Ext.h"
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
			if (!nodes[0]->HasComponent<NodeShape>()) return {};
			auto shp = nodes[0]->GetComponent<NodeShape>().GetShape();
			return MakeShapeVal(shp);
		},
		{false, false, false, false, true});  // no_vt_cache

	reg.Define("selector_face", {"shape", "uid"}, {},
		[](EvalCtx& ctx) -> Val {
			uint32_t uid = static_cast<uint32_t>(ctx.Num(1));
			if (!ctx.tn) return {};
			std::vector<std::shared_ptr<graph::Node>> nodes;
			if (!ctx.tn->GetFaceGraph()->QueryNodes(uid, nodes)) return {};
			if (nodes.empty()) return {};
			auto shp = nodes[0]->GetComponent<NodeShape>().GetShape();
			return MakeShapeVal(shp);
		},
		{false, false, false, false, true});  // no_vt_cache

	reg.Define("merge", {}, {"shapes"},
		[](EvalCtx& ctx) -> Val {
			auto shapes = ctx.VarShapes();
			if (shapes.empty()) return {};
			if (shapes.size() == 1) return shapes[0];
			return shapes[0];
		});

	reg.Define("cylinder", {"radius", "height"}, {},
		[](EvalCtx& ctx) -> Val {
			double r = ctx.Num(0);
			double h = ctx.Num(1);
			auto shape = partgraph::PrimMaker::Cylinder(r, h, ctx.op_id, ctx.tn);
			return MakeShapeVal(shape);
		});

	reg.Define("cone", {"radius1", "radius2", "height"}, {},
		[](EvalCtx& ctx) -> Val {
			double r1 = ctx.Num(0);
			double r2 = ctx.Num(1);
			double h  = ctx.Num(2);
			auto shape = partgraph::PrimMaker::Cone(r1, r2, h, ctx.op_id, ctx.tn);
			return MakeShapeVal(shape);
		});

	reg.Define("sphere", {"radius"}, {},
		[](EvalCtx& ctx) -> Val {
			double r = ctx.Num(0);
			auto shape = partgraph::PrimMaker::Sphere(r, ctx.op_id, ctx.tn);
			return MakeShapeVal(shape);
		});

	reg.Define("torus", {"major_radius", "minor_radius"}, {},
		[](EvalCtx& ctx) -> Val {
			double r1 = ctx.Num(0);
			double r2 = ctx.Num(1);
			auto shape = partgraph::PrimMaker::Torus(r1, r2, ctx.op_id, ctx.tn);
			return MakeShapeVal(shape);
		});

	reg.Define("prism", {"face", "direction"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			auto dir = ctx.GetVec3(1);
			auto shp = partgraph::TopoAlgo::Prism(
				sv.shape, dir[0], dir[1], dir[2], ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		});

	reg.Define("extrude_ex",
		{"face", "direction", "dist1", "dist2", "end1", "end2", "ref"}, {},
		[](EvalCtx& ctx) -> Val {
			auto face = ctx.GetShape(0);
			if (!face.shape) return {};
			auto dir = ctx.GetVec3(1);
			double d1 = ctx.Num(2);
			double d2 = ctx.Num(3);
			auto e1 = static_cast<partgraph::ExtrudeEndType>(ctx.Int(4));
			auto e2 = static_cast<partgraph::ExtrudeEndType>(ctx.Int(5));
			auto ref = ctx.GetShape(6);
			auto shp = partgraph::TopoAlgo_Ext::ExtrudeEx(
				face.shape, dir[0], dir[1], dir[2],
				d1, d2, e1, e2,
				ref.shape, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		});

	reg.Define("mirror", {"shape", "origin", "normal"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			auto o = ctx.GetVec3(1);
			auto n = ctx.GetVec3(2);
			auto shp = partgraph::TopoAlgo::Mirror(sv.shape,
				sm::vec3((float)o[0], (float)o[1], (float)o[2]),
				sm::vec3((float)n[0], (float)n[1], (float)n[2]),
				ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		});

	reg.Define("shell", {"shape", "thickness"}, {"faces"},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			float thickness = (float)ctx.Num(1);
			auto face_vals = ctx.VarShapes();
			std::vector<std::shared_ptr<partgraph::TopoShape>> faces;
			for (auto& fv : face_vals) {
				if (fv.shape) faces.push_back(fv.shape);
			}
			auto shp = partgraph::TopoAlgo::ThickSolid(
				sv.shape, faces, thickness, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		},
		{true, false, false, false});
}

} // namespace breptopo
