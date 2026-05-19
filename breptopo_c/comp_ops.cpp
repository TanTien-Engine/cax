#include "comp_ops.h"
#include "CompGraph.h"
#include "TopoNaming.h"
#include "HistGraph.h"
#include "NodeShape.h"

#include "partgraph_c/PrimMaker.h"
#include "partgraph_c/TopoAlgo.h"
#include "partgraph_c/TopoAlgo_Ext.h"
#include "partgraph_c/TopoShape.h"

#include "cadcvt_c/ir/SketchIR.h"
#include "cadcvt_c/store/SketchBridge.h"

#include "sketchlib/Scene.h"

#include <geoshape/Shape2D.h>
#include <geoshape/Point2D.h>
#include <geoshape/Line2D.h>
#include <geoshape/Circle.h>
#include <geoshape/Arc.h>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Circ.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopTools_ListOfShape.hxx>
#include <ShapeFix_Face.hxx>

#include <graph/Node.h>

namespace breptopo
{

static ShapeVal MakeShapeVal(const std::shared_ptr<partgraph::TopoShape>& shp)
{
	ShapeVal sv;
	sv.shape = shp;
	return sv;
}

// ---------------------------------------------------------------
//  sketch_face helpers (used only by the sketch_face op below)
// ---------------------------------------------------------------
namespace
{

// Map a 2D sketch point (x, y) into 3D via (origin, x_dir, normal).
gp_Pnt LocalToWorld(double x, double y,
                    const double origin[3],
                    const double x_dir[3],
                    const double normal[3])
{
	double yd[3] =
	{
		normal[1] * x_dir[2] - normal[2] * x_dir[1],
		normal[2] * x_dir[0] - normal[0] * x_dir[2],
		normal[0] * x_dir[1] - normal[1] * x_dir[0],
	};
	return gp_Pnt(
		origin[0] + x_dir[0] * x + yd[0] * y,
		origin[1] + x_dir[1] * x + yd[1] * y,
		origin[2] + x_dir[2] * x + yd[2] * y);
}

// Build one TopoDS_Edge from a single 2D geometry, lifted onto the
// sketch plane. Using exact Geom_Curve based edges (line / circle /
// arc) instead of polygon sampling is what lets MakeWire match
// endpoints reliably; sampled polylines accumulate fp error and the
// resulting wire looks open to OCCT, which makes Prism degenerate to
// a sweep of the wire (uncapped shell).
TopoDS_Edge BuildEdgeFromShape(const gs::Shape2D& shape,
                                const double origin[3],
                                const double x_dir[3],
                                const double normal[3])
{
	switch (shape.GetType())
	{
	case gs::ShapeType2D::Line:
	{
		const auto& s  = static_cast<const gs::Line2D&>(shape);
		gp_Pnt p1 = LocalToWorld(s.GetStart().x, s.GetStart().y, origin, x_dir, normal);
		gp_Pnt p2 = LocalToWorld(s.GetEnd().x,   s.GetEnd().y,   origin, x_dir, normal);
		if (p1.Distance(p2) < 1e-9) return TopoDS_Edge();
		BRepBuilderAPI_MakeEdge mk(p1, p2);
		return mk.IsDone() ? mk.Edge() : TopoDS_Edge();
	}
	case gs::ShapeType2D::Circle:
	{
		const auto& s = static_cast<const gs::Circle&>(shape);
		gp_Pnt  c = LocalToWorld(s.GetCenter().x, s.GetCenter().y, origin, x_dir, normal);
		gp_Dir  nd(normal[0], normal[1], normal[2]);
		gp_Dir  xd(x_dir [0], x_dir [1], x_dir [2]);
		gp_Ax2  ax(c, nd, xd);
		gp_Circ circ(ax, s.GetRadius());
		BRepBuilderAPI_MakeEdge mk(circ);
		return mk.IsDone() ? mk.Edge() : TopoDS_Edge();
	}
	case gs::ShapeType2D::Arc:
	{
		const auto& s = static_cast<const gs::Arc&>(shape);
		float a0 = 0, a1 = 0;
		s.GetAngles(a0, a1);
		gp_Pnt  c = LocalToWorld(s.GetCenter().x, s.GetCenter().y, origin, x_dir, normal);
		gp_Dir  nd(normal[0], normal[1], normal[2]);
		gp_Dir  xd(x_dir [0], x_dir [1], x_dir [2]);
		gp_Ax2  ax(c, nd, xd);
		gp_Circ circ(ax, s.GetRadius());
		BRepBuilderAPI_MakeEdge mk(circ, (double)a0, (double)a1);
		return mk.IsDone() ? mk.Edge() : TopoDS_Edge();
	}
	case gs::ShapeType2D::Point:
		return TopoDS_Edge();
	default:
		return TopoDS_Edge();
	}
}

TopoDS_Wire BuildWireFromSolved(const cadcvt::SketchBridge::GeoShapes& solved,
                                 const double origin[3],
                                 const double x_dir[3],
                                 const double normal[3])
{
	TopTools_ListOfShape edges;
	for (const auto& kv : solved)
	{
		TopoDS_Edge e = BuildEdgeFromShape(*kv.second, origin, x_dir, normal);
		if (!e.IsNull()) edges.Append(e);
	}
	if (edges.IsEmpty()) return TopoDS_Wire();

	BRepBuilderAPI_MakeWire mk;
	mk.Add(edges);
	return mk.IsDone() ? mk.Wire() : TopoDS_Wire();
}

std::shared_ptr<partgraph::TopoShape> WireToFace(const TopoDS_Wire& wire,
                                                  const double origin[3],
                                                  const double normal[3])
{
	if (wire.IsNull()) return {};

	gp_Pnt o(origin[0], origin[1], origin[2]);
	gp_Dir n(normal[0], normal[1], normal[2]);
	gp_Pln plane(o, n);

	// Do NOT call ShapeFix_Face::FixAddNaturalBound: for a plane the
	// natural bound is an infinite box, the fixer would treat our
	// wire as an inner hole, and Prism then sweeps it into an
	// annular slab with the inner top cap missing.
	BRepBuilderAPI_MakeFace mkFace(plane, wire);
	if (!mkFace.IsDone()) return {};

	// FixOrientation flips CW wires so the face normal lines up with
	// the plane normal -- without this Prism's caps may end up with
	// inward normals (renderer hides them, looks like a missing cap).
	TopoDS_Face   face = mkFace.Face();
	ShapeFix_Face fixer(face);
	fixer.FixOrientation();
	fixer.Perform();
	face = fixer.Face();

	return std::make_shared<partgraph::TopoShape>(face);
}

} // anonymous namespace

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

	// sketch_face: pulls the SketchIR back out of the type-erased
	// SketchVal, runs sketchlib's constraint solver, and lifts the
	// solved 2D wire onto the plane carried by the Vec3 inputs.
	// Plane params are separate inputs so changing the plane re-runs
	// the wire/face step without re-solving constraints.
	reg.Define("sketch_face", {"sketch", "origin", "x_dir", "normal"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetSketch(0);
			if (!sv.handle) return {};
			auto* sk = static_cast<const cadcvt::SketchIR*>(sv.handle.get());

			Vec3 origin_v = ctx.GetVec3(1);
			Vec3 x_dir_v  = ctx.GetVec3(2);
			Vec3 normal_v = ctx.GetVec3(3);

			sketchlib::Scene                 scene;
			cadcvt::SketchBridge::GeoShapes  solved;
			if (!cadcvt::SketchBridge::ImportToScene(*sk, scene, solved)) return {};
			scene.Solve(solved);

			TopoDS_Wire wire = BuildWireFromSolved(
				solved, origin_v.data(), x_dir_v.data(), normal_v.data());
			if (wire.IsNull()) return {};

			auto face = WireToFace(wire, origin_v.data(), normal_v.data());
			if (!face) return {};

			return MakeShapeVal(face);
		});
}

} // namespace breptopo
