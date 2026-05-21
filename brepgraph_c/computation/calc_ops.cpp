#include "brepgraph_c/computation/calc_ops.h"
#include "brepgraph_c/computation/CalcGraph.h"
#include "brepgraph_c/history/TopoNaming.h"
#include "brepgraph_c/history/HistGraph.h"
#include "brepgraph_c/common/NodeShape.h"

#include "brepkit_c/PrimMaker.h"
#include "brepkit_c/TopoAlgo.h"
#include "brepkit_c/TopoAlgo_Ext.h"
#include "brepkit_c/TopoShape.h"

#include <graph/Node.h>

namespace brepgraph
{

static ShapeVal MakeShapeVal(const std::shared_ptr<brepkit::TopoShape>& shp)
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
			auto shape = brepkit::PrimMaker::Box(dx, dy, dz, ctx.op_id, ctx.tn);
			return MakeShapeVal(shape);
		});

	reg.Define("translate", {"shape", "offset"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			auto off = ctx.GetVec3(1);
			if (!sv.shape) return {};
			auto shp = brepkit::TopoAlgo::Translate(sv.shape, off[0], off[1], off[2], ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		});

	reg.Define("rotate", {"shape", "axis_origin", "axis_dir", "angle"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			auto o = ctx.GetVec3(1);
			auto d = ctx.GetVec3(2);
			double a = ctx.Num(3);
			auto shp = brepkit::TopoAlgo::Rotate(sv.shape,
				sm::vec3((float)o[0], (float)o[1], (float)o[2]),
				sm::vec3((float)d[0], (float)d[1], (float)d[2]),
				a, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		});

	reg.Define("offset", {"shape", "offset", "is_solid"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			double off = ctx.Num(1);
			bool is_solid = ctx.Bool(2);
			auto shp = brepkit::TopoAlgo::OffsetShape(sv.shape, static_cast<float>(off), is_solid, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		},
		{true, false, false, false});  // is_dressup

	reg.Define("cut", {"shape1", "shape2"}, {},
		[](EvalCtx& ctx) -> Val {
			auto s1 = ctx.GetShape(0);
			auto s2 = ctx.GetShape(1);
			if (!s1.shape || !s2.shape) return {};
			auto shp = brepkit::TopoAlgo::Cut(s1.shape, s2.shape, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		},
		{false, false, true, false});  // is_boolean

	reg.Define("fuse", {"shape1", "shape2"}, {},
		[](EvalCtx& ctx) -> Val {
			auto s1 = ctx.GetShape(0);
			auto s2 = ctx.GetShape(1);
			if (!s1.shape || !s2.shape) return {};
			auto shp = brepkit::TopoAlgo::Fuse(s1.shape, s2.shape, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		},
		{false, false, true, false});  // is_boolean

	reg.Define("fillet", {"shape", "radius"}, {"edges"},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			double radius = ctx.Num(1);
			auto edge_vals = ctx.VarShapes();
			std::vector<std::shared_ptr<brepkit::TopoShape>> edges;
			for (auto& ev : edge_vals) {
				if (ev.shape) edges.push_back(ev.shape);
			}
			auto shp = brepkit::TopoAlgo::Fillet(sv.shape, radius, edges, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		},
		{true, false, false, false});  // is_dressup

	reg.Define("chamfer", {"shape", "dist"}, {"edges"},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			double dist = ctx.Num(1);
			auto edge_vals = ctx.VarShapes();
			std::vector<std::shared_ptr<brepkit::TopoShape>> edges;
			for (auto& ev : edge_vals) {
				if (ev.shape) edges.push_back(ev.shape);
			}
			auto shp = brepkit::TopoAlgo::Chamfer(sv.shape, dist, edges, ctx.op_id, ctx.tn);
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
			auto shape = brepkit::PrimMaker::Cylinder(r, h, ctx.op_id, ctx.tn);
			return MakeShapeVal(shape);
		});

	reg.Define("cone", {"radius1", "radius2", "height"}, {},
		[](EvalCtx& ctx) -> Val {
			double r1 = ctx.Num(0);
			double r2 = ctx.Num(1);
			double h  = ctx.Num(2);
			auto shape = brepkit::PrimMaker::Cone(r1, r2, h, ctx.op_id, ctx.tn);
			return MakeShapeVal(shape);
		});

	reg.Define("sphere", {"radius"}, {},
		[](EvalCtx& ctx) -> Val {
			double r = ctx.Num(0);
			auto shape = brepkit::PrimMaker::Sphere(r, ctx.op_id, ctx.tn);
			return MakeShapeVal(shape);
		});

	reg.Define("torus", {"major_radius", "minor_radius"}, {},
		[](EvalCtx& ctx) -> Val {
			double r1 = ctx.Num(0);
			double r2 = ctx.Num(1);
			auto shape = brepkit::PrimMaker::Torus(r1, r2, ctx.op_id, ctx.tn);
			return MakeShapeVal(shape);
		});

	reg.Define("prism", {"face", "direction"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			auto dir = ctx.GetVec3(1);
			auto shp = brepkit::TopoAlgo::Prism(
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
			auto e1 = static_cast<brepkit::ExtrudeEndType>(ctx.Int(4));
			auto e2 = static_cast<brepkit::ExtrudeEndType>(ctx.Int(5));
			auto ref = ctx.GetShape(6);
			auto shp = brepkit::TopoAlgo_Ext::ExtrudeEx(
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
			auto shp = brepkit::TopoAlgo::Mirror(sv.shape,
				sm::vec3((float)o[0], (float)o[1], (float)o[2]),
				sm::vec3((float)n[0], (float)n[1], (float)n[2]),
				ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		});

	reg.Define("linear_pattern",
		{"shape", "dir1", "count1", "spacing1", "dir2", "count2", "spacing2"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			auto d1 = ctx.GetVec3(1);
			int  c1 = ctx.Int(2);
			double s1 = ctx.Num(3);
			auto d2 = ctx.GetVec3(4);
			int  c2 = ctx.Int(5);
			double s2 = ctx.Num(6);
			auto shp = brepkit::TopoAlgo_Ext::LinearPattern(sv.shape,
				sm::vec3((float)d1[0], (float)d1[1], (float)d1[2]), c1, s1,
				sm::vec3((float)d2[0], (float)d2[1], (float)d2[2]), c2, s2,
				ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		},
		{false, true, false, false, false});  // is_pattern

	reg.Define("circular_pattern",
		{"shape", "axis_origin", "axis_dir", "count", "total_angle"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			auto o = ctx.GetVec3(1);
			auto d = ctx.GetVec3(2);
			int  c = ctx.Int(3);
			double a = ctx.Num(4);
			auto shp = brepkit::TopoAlgo_Ext::CircularPattern(sv.shape,
				sm::vec3((float)o[0], (float)o[1], (float)o[2]),
				sm::vec3((float)d[0], (float)d[1], (float)d[2]),
				c, a, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		},
		{false, true, false, false, false});  // is_pattern

	reg.Define("shell", {"shape", "thickness"}, {"faces"},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			float thickness = (float)ctx.Num(1);
			auto face_vals = ctx.VarShapes();
			std::vector<std::shared_ptr<brepkit::TopoShape>> faces;
			for (auto& fv : face_vals) {
				if (fv.shape) faces.push_back(fv.shape);
			}
			auto shp = brepkit::TopoAlgo::ThickSolid(
				sv.shape, faces, thickness, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		},
		{true, false, false, false});

	// "sketch_face" is registered by cadapp::RegisterSketchOps -- it
	// is the only op that needs to know a concrete IR type (cadapp::
	// SketchIR sitting inside the type-erased SketchVal), so it lives
	// in cadapp/ops/ rather than here.
}

} // namespace brepgraph
