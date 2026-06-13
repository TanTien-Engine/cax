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

#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <Geom_Curve.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <TopAbs.hxx>
#include <TopExp_Explorer.hxx>
#include <gp_Pnt.hxx>

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

	reg.Define("common", {"shape1", "shape2"}, {},
		[](EvalCtx& ctx) -> Val {
			auto s1 = ctx.GetShape(0);
			auto s2 = ctx.GetShape(1);
			if (!s1.shape || !s2.shape) return {};
			auto shp = brepkit::TopoAlgo::Common(s1.shape, s2.shape, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		},
		{false, false, true, false});  // is_boolean

	// trim: split shape1 by shape2's faces and keep only the fragments
	// on the (keep_pt, keep_dir) side; the tool body itself is consumed
	// by the caller (ZW3D FtSolidSoloTrm). mutual!=0 additionally trims
	// the TOOL by the base and keeps its witnessed-side remnant as a
	// separate body (ZW3D fld8). Tool missing -> pass-through (a missed
	// trim degrades the metric, an emptied chain kills it).
	reg.Define("trim", {"shape", "tool", "keep_pt", "keep_dir", "mutual"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			auto tv = ctx.GetShape(1);
			if (!tv.shape) return MakeShapeVal(sv.shape);
			auto p = ctx.GetVec3(2);
			auto d = ctx.GetVec3(3);
			const bool mu = ctx.Num(4) > 0.5;
			auto shp = brepkit::TopoAlgo::TrimByTool(sv.shape, tv.shape,
				sm::vec3((float)p[0], (float)p[1], (float)p[2]),
				sm::vec3((float)d[0], (float)d[1], (float)d[2]),
				mu, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp ? shp : sv.shape);
		},
		{false, false, true, false});  // is_boolean

	// sew: join base + tool sheet bodies into one shell at `tol`,
	// solidifying closed results (ZW3D CdShapeSew / sheet FtBoolSoloAdd).
	// Tool missing -> pass-through, same degradation policy as trim.
	reg.Define("sew", {"shape", "tool", "tol"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			auto tv = ctx.GetShape(1);
			if (!tv.shape) return MakeShapeVal(sv.shape);
			const double tol = ctx.Num(2);
			auto shp = brepkit::TopoAlgo::SewJoin(sv.shape, tv.shape, tol,
				ctx.op_id, ctx.tn);
			return MakeShapeVal(shp ? shp : sv.shape);
		},
		{false, false, true, false});  // is_boolean

	// Pre-split body edges at a set of point hints. Inserted into the
	// graph before the Fillet / Chamfer dressup when the FreeCAD reader's
	// face-pick handler detected base brep vertices that don't exist in
	// the cax body (BOP merged adjacent edges). Splitting those edges
	// back into segments lets ChFi3d handle fillets that would otherwise
	// fail on the merged curve. See TopoAlgo::SplitBodyAtPoints.
	reg.Define("split_body_at_points", {"shape"}, {"hints"},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			auto hint_vals = ctx.VarShapes();
			std::vector<sm::vec3> pts;
			pts.reserve(hint_vals.size());
			for (auto& hv : hint_vals) {
				if (!hv.shape) continue;
				const TopoDS_Shape& s = hv.shape->GetShape();
				if (s.ShapeType() != TopAbs_VERTEX) continue;
				gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(s));
				pts.push_back(sm::vec3(
					(float)p.X(), (float)p.Y(), (float)p.Z()));
			}
			if (pts.empty()) return MakeShapeVal(sv.shape);
			auto shp = brepkit::TopoAlgo::SplitBodyAtPoints(
				sv.shape, pts, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		});

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
			// Edge refs were declared but none resolved (e.g. FreeCAD
			// Fillet referencing faces our resolver can't match).
			// Skip rather than fall through to TopoAlgo's "empty edges
			// = fillet all" path, which tries to round every edge of
			// the body and crashes on non-trivial shapes.
			if (!edge_vals.empty() && edges.empty()) {
				return MakeShapeVal(sv.shape);
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
			if (!edge_vals.empty() && edges.empty()) {
				return MakeShapeVal(sv.shape);
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

	// merge: bundle every input shape into one COMPOUND. Consumers that
	// need "all the tools as one input" (sew, trim) explore it by face;
	// the old stub returned shapes[0] only, silently dropping the rest
	// (02-ear 缝合2: 3 tools wired, 1 arrived, the ear box never closed).
	reg.Define("merge", {}, {"shapes"},
		[](EvalCtx& ctx) -> Val {
			auto shapes = ctx.VarShapes();
			if (shapes.empty()) return {};
			if (shapes.size() == 1) return shapes[0];
			BRep_Builder bb;
			TopoDS_Compound comp;
			bb.MakeCompound(comp);
			int added = 0;
			for (auto& sv : shapes)
			{
				if (sv.shape && !sv.shape->GetShape().IsNull())
				{
					bb.Add(comp, sv.shape->GetShape());
					++added;
				}
			}
			if (added == 0) return {};
			return MakeShapeVal(std::make_shared<brepkit::TopoShape>(comp));
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

	reg.Define("ellipsoid", {"radius1", "radius2", "radius3"}, {},
		[](EvalCtx& ctx) -> Val {
			double r1 = ctx.Num(0);
			double r2 = ctx.Num(1);
			double r3 = ctx.Num(2);
			auto shape = brepkit::PrimMaker::Ellipsoid(r1, r2, r3, ctx.op_id, ctx.tn);
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

	// Drafted prism: `angle` radians, > 0 shrinks the section along the
	// extrusion (ZW3D Draft semantics; see TopoAlgo::DPrism).
	reg.Define("dprism", {"face", "direction", "angle"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			auto dir = ctx.GetVec3(1);
			double angle = ctx.Num(2);
			auto shp = brepkit::TopoAlgo::DPrism(
				sv.shape, dir[0], dir[1], dir[2], angle, ctx.op_id, ctx.tn);
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

	reg.Define("revolve",
		{"face", "axis_origin", "axis_dir", "angle", "is_full"}, {},
		[](EvalCtx& ctx) -> Val {
			auto face = ctx.GetShape(0);
			if (!face.shape) return {};
			auto o = ctx.GetVec3(1);
			auto d = ctx.GetVec3(2);
			double a = ctx.Num(3);
			bool   is_full = ctx.Bool(4);
			auto shp = brepkit::TopoAlgo_Ext::Revolve(
				face.shape,
				sm::vec3((float)o[0], (float)o[1], (float)o[2]),
				sm::vec3((float)d[0], (float)d[1], (float)d[2]),
				a, is_full, ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		});

	reg.Define("sweep", {"profile", "path", "is_solid", "frenet"}, {},
		[](EvalCtx& ctx) -> Val {
			auto profile = ctx.GetShape(0);
			auto path    = ctx.GetShape(1);
			if (!profile.shape || !path.shape) return {};
			bool is_solid = ctx.Bool(2);
			bool frenet   = ctx.Bool(3);
			auto shp = brepkit::TopoAlgo_Ext::Sweep(
				profile.shape, path.shape, is_solid, ctx.op_id, ctx.tn, frenet);
			return MakeShapeVal(shp);
		});

	// helix_wire: a parametric helical spine (FreeCAD Part::Helix).
	// Built in the local frame (axis +Z, starts at (radius,0,0)); the
	// caller feeds it to `sweep` as the path. cone_angle ~0 (or the
	// degenerate >= pi/2) yields a cylindrical helix.
	reg.Define("helix_wire",
		{"pitch", "height", "radius", "cone_angle", "left_handed"}, {},
		[](EvalCtx& ctx) -> Val {
			double pitch      = ctx.Num(0);
			double height     = ctx.Num(1);
			double radius     = ctx.Num(2);
			double cone_angle = ctx.Num(3);
			bool   left_handed = ctx.Bool(4);
			auto shp = brepkit::TopoAlgo_Ext::HelixWire(
				pitch, height, radius, cone_angle, left_handed,
				ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		});

	// loft: BRepOffsetAPI_ThruSections across >= 2 section wires.
	// Profiles are variadic so the same op handles two-section and
	// multi-section lofts without bumping op arity. The is_solid flag
	// rides as a positional input because the FreeCAD AdditiveLoft
	// path needs a solid, while a future shell-only caller can pass
	// false and skip the cap.
	//
	// Post-step: ShapeUpgrade_UnifySameDomain (via TopoAlgo::UnifySameDomain).
	// FreeCAD's PartDesign::AdditiveLoft defaults Refine=true and pipes
	// the ThruSections result through the same upgrader. Without that
	// pass our loft body keeps every per-section seam edge plus any
	// degenerate edges OCCT introduces along the side surface; a later
	// Fillet that lands on (or near) one of those degenerate / split
	// edges builds a self-intersecting blend face and shows up as a
	// big bad patch (see Page_020_Exercise2D-12: the trailing Fillet
	// on Mirrored.Edge12 left a phantom blend until we refined the
	// loft body here).
	reg.Define("loft", {"is_solid"}, {"profiles"},
		[](EvalCtx& ctx) -> Val {
			bool is_solid = ctx.Bool(0);
			auto profile_vals = ctx.VarShapes();
			std::vector<std::shared_ptr<brepkit::TopoShape>> wires;
			wires.reserve(profile_vals.size());
			for (auto& pv : profile_vals) {
				if (pv.shape) wires.push_back(pv.shape);
			}
			if (wires.size() < 2) return {};
			auto shp = brepkit::TopoAlgo::ThruSections(
				wires, is_solid, ctx.op_id, ctx.tn);
			if (!shp) return {};
			// Refine: same op_id so the downstream resolver still sees
			// one naming layer per "loft" feature instead of two.
			auto refined = brepkit::TopoAlgo::UnifySameDomain(
				shp, ctx.op_id, ctx.tn);
			return MakeShapeVal(refined ? refined : shp);
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

	// refine: ShapeUpgrade_UnifySameDomain on the input shape.
	// Returns the input unchanged when the upgrader produces an
	// empty result (e.g. shape has no merge-able adjacent faces).
	// Used by the Mirror feature replay to collapse the per-
	// Original fuse seams before a downstream Fillet picks edges
	// -- see Page_020: pre-refine Mirror left 68 faces / 159 edges
	// where FreeCAD had 42 / 113, and the Fillet's edge-ref
	// resolver matched against one of those phantom seam edges,
	// producing a self-intersecting blend with negative volume
	// and a hundreds-of-metres bbox.
	reg.Define("refine", {"shape"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			auto refined = brepkit::TopoAlgo::UnifySameDomain(
				sv.shape, ctx.op_id, ctx.tn);
			return MakeShapeVal(refined ? refined : sv.shape);
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

	// Feature pattern: replicate a seed `tool` on a linear grid and
	// combine ALL instances onto `base` with ONE boolean (op_kind:
	// 0 = fuse / boss, 1 = cut / hole). Folds the tool-pattern + the
	// boolean into a single op so it survives history-rebuild as one
	// node instead of expanding into N per-instance booleans.
	reg.Define("feature_pattern",
		{"base", "tool", "op_kind", "dir1", "count1", "spacing1", "dir2", "count2", "spacing2"}, {},
		[](EvalCtx& ctx) -> Val {
			auto bv = ctx.GetShape(0);
			auto tv = ctx.GetShape(1);
			if (!tv.shape) return {};
			if (!bv.shape) return MakeShapeVal(tv.shape);
			int  op_kind = ctx.Int(2);
			auto d1 = ctx.GetVec3(3);
			int  c1 = ctx.Int(4);
			double s1 = ctx.Num(5);
			auto d2 = ctx.GetVec3(6);
			int  c2 = ctx.Int(7);
			double s2 = ctx.Num(8);
			auto shp = brepkit::TopoAlgo_Ext::FeaturePattern(bv.shape, tv.shape, op_kind,
				sm::vec3((float)d1[0], (float)d1[1], (float)d1[2]), c1, s1,
				sm::vec3((float)d2[0], (float)d2[1], (float)d2[2]), c2, s2,
				ctx.op_id, ctx.tn);
			return MakeShapeVal(shp);
		},
		{});  // self-contained folded op (pattern union + one boolean);
		      // intentionally NOT flagged is_pattern -- it has two shape
		      // inputs and an internal boolean, so optimizer passes that
		      // assume a single-input geometry pattern must not touch it.

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

	// Find the nearest edge in a shape to a given pick point, returned as
	// a single-edge wire.  Used by the ZW3D sweep reader when the path is a
	// body edge (the plugin only exports the pick point, not the curve).
	// Uses the midpoint of the parametric range: for a boss rim circle
	// (radius R from pick point) this gives distance R, correctly beating
	// any long straight edge whose midpoint is further away.  Empirically,
	// this produces the closest result to ZW3D truth on R2900.
	reg.Define("edge_pick_wire", {"shape", "pick_x", "pick_y", "pick_z"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetShape(0);
			if (!sv.shape) return {};
			double px = ctx.Num(1), py = ctx.Num(2), pz = ctx.Num(3);
			gp_Pnt pick(px, py, pz);

			const TopoDS_Shape& base = sv.shape->GetShape();
			double      best_d = 1e30;
			TopoDS_Edge best;

			for (TopExp_Explorer ex(base, TopAbs_EDGE); ex.More(); ex.Next()) {
				const TopoDS_Edge& e = TopoDS::Edge(ex.Current());
				double t0, t1;
				Handle(Geom_Curve) c = BRep_Tool::Curve(e, t0, t1);
				if (c.IsNull()) continue;
				double d = c->Value((t0 + t1) * 0.5).Distance(pick);
				if (d < best_d) { best_d = d; best = e; }
			}

			if (best.IsNull()) return {};
			BRepBuilderAPI_MakeWire mw(best);
			if (!mw.IsDone()) return {};
			return MakeShapeVal(
				std::make_shared<brepkit::TopoShape>(mw.Wire()));
		});

	// "sketch_face" is registered by cadapp::RegisterSketchOps -- it
	// is the only op that needs to know a concrete IR type (cadapp::
	// SketchIR sitting inside the type-erased SketchVal), so it lives
	// in cadapp/ops/ rather than here.
}

} // namespace brepgraph
