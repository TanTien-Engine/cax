#include "cadapp_c/ops/sketch_ops.h"
#include "cadapp_c/ir/SketchIR.h"
#include "cadapp_c/store/SketchBridge.h"

#include "brepgraph_c/computation/CalcGraph.h"

#include "brepkit_c/TopoShape.h"

#include "sketchlib/Scene.h"

#include <geoshape/Shape2D.h>
#include <geoshape/Point2D.h>
#include <geoshape/Line2D.h>
#include <geoshape/Circle.h>
#include <geoshape/Arc.h>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepGProp.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <BRep_Builder.hxx>
#include <ElSLib.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Circ.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <ShapeFix_Face.hxx>

#include <memory>

namespace cadapp
{

namespace
{

using brepgraph::EvalCtx;
using brepgraph::ShapeVal;
using brepgraph::Val;
using brepgraph::Vec3;

ShapeVal MakeShapeVal(const std::shared_ptr<brepkit::TopoShape>& shp)
{
	ShapeVal sv;
	sv.shape = shp;
	return sv;
}

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

// Gap tolerance for stitching edges into wires. Project space is
// metres (FreeCAD mm * unit_scale 0.001), and sketchlib's solver is
// float -- so endpoints can drift by ~1e-6 m even when the user's
// constraints say they coincide. A pure BRepBuilderAPI_MakeWire is
// stricter than that (Precision::Confusion() ~ 1e-7) and rejects the
// chain. 1e-5 m (10 microns) is loose enough to absorb solver drift
// without merging unrelated endpoints in a tightly-packed sketch.
constexpr double kWireStitchTol = 1.0e-5;

// Chain solved edges into one or more closed wires. Multiple wires
// indicate a sketch with holes or disconnected loops; the caller is
// responsible for assembling them into a single face.
//
// Returns an empty sequence when no edges could be built or no wires
// could be stitched at all.
Handle(TopTools_HSequenceOfShape)
BuildWiresFromSolved(const cadapp::SketchBridge::GeoShapes& solved,
                     const double origin[3],
                     const double x_dir[3],
                     const double normal[3])
{
	Handle(TopTools_HSequenceOfShape) edges_seq =
		new TopTools_HSequenceOfShape();
	for (const auto& kv : solved)
	{
		TopoDS_Edge e = BuildEdgeFromShape(*kv.second, origin, x_dir, normal);
		if (!e.IsNull()) edges_seq->Append(e);
	}
	if (edges_seq->IsEmpty()) return Handle(TopTools_HSequenceOfShape)();

	Handle(TopTools_HSequenceOfShape) wires_seq;
	// shared=Standard_False: chain by endpoint distance, not by
	// shared TVertex identity. Our edges are built fresh from
	// gp_Pnt's, so no shared vertices exist yet.
	ShapeAnalysis_FreeBounds::ConnectEdgesToWires(
		edges_seq, kWireStitchTol, Standard_False, wires_seq);
	return wires_seq;
}

// Per-wire scratch used by WiresToFace's containment classifier:
// a wire-only face for point-in classification, that face's area
// (cheaper than recomputing each pass), an interior sample point
// (centroid of the bounded region), and the depth in the
// containment forest (0 = outer region, 1 = hole, 2 = island inside
// a hole, ...).
struct WireRec
{
	TopoDS_Wire wire;
	TopoDS_Face face;       // wire-only, no holes - used for classification + area
	double      area = 0.0;
	gp_Pnt      sample;
	int         depth = 0;
	// Point-in-face classifier for `face`, built ONCE here and reused across
	// the O(n^2) containment tests below. Rebuilding a BRepTopAdaptor_FClass2d
	// per pairwise test was ~98% of the sketch_face op cost on dense profiles
	// (92 wires -> ~8.5k rebuilds, ~700ms); the ctor is the expensive part,
	// Perform() is cheap. unique_ptr keeps WireRec move-only (no accidental copy).
	std::unique_ptr<BRepTopAdaptor_FClass2d> cls;
};

// Build a WireRec from a single TopoDS_Wire. Returns false when the
// wire degenerates (open, zero area, MakeFace failed) and the caller
// should drop it.
bool BuildWireRec(const TopoDS_Wire& w, const gp_Pln& plane, WireRec& out)
{
	BRepBuilderAPI_MakeFace mk(plane, w);
	if (!mk.IsDone()) return false;
	out.face = mk.Face();
	GProp_GProps props;
	BRepGProp::SurfaceProperties(out.face, props);
	out.area = std::abs(props.Mass());
	if (out.area == 0.0) return false;
	out.sample = props.CentreOfMass();
	out.wire   = w;
	out.cls    = std::make_unique<BRepTopAdaptor_FClass2d>(
		out.face, Precision::Confusion());
	return true;
}

// Classify a 3D world point against a planar face's PREBUILT classifier by
// projecting onto the plane's UV. The classifier must come from a face
// supported by `plane` (its UV system); since we build all faces here from
// the same gp_Pln this holds.
bool PointInsideFace(BRepTopAdaptor_FClass2d& cls, const gp_Pln& plane,
                      const gp_Pnt& p)
{
	double u, v;
	ElSLib::Parameters(plane, p, u, v);
	return cls.Perform(gp_Pnt2d(u, v)) == TopAbs_IN;
}

// Stitch wires into faces honoring containment. A sketch with two
// disjoint annuli (e.g. FreeCAD pocket with two through-holes drawn
// in one sketch) produces four wires here - two outer R, two inner
// r - and the previous "largest = outer, rest = holes" heuristic
// collapsed them into one bogus face. The forest below classifies
// each wire by depth and emits one face per even-depth wire with
// its immediate odd-depth children as holes; disjoint regions land
// as separate faces inside a compound, and nested cases (island
// inside hole inside outer) are handled by the same rule.
std::shared_ptr<brepkit::TopoShape> WiresToFace(
	const Handle(TopTools_HSequenceOfShape)& wires,
	const double                              origin[3],
	const double                              normal[3])
{
	if (wires.IsNull() || wires->IsEmpty()) return {};

	gp_Pnt o(origin[0], origin[1], origin[2]);
	gp_Dir n(normal[0], normal[1], normal[2]);
	gp_Pln plane(o, n);

	std::vector<WireRec> recs;
	recs.reserve(wires->Length());
	for (int i = 1; i <= wires->Length(); ++i)
	{
		WireRec r;
		if (BuildWireRec(TopoDS::Wire(wires->Value(i)), plane, r))
			recs.push_back(std::move(r));
	}
	if (recs.empty()) return {};

	// Depth = number of OTHER wires that strictly contain me. Skip
	// pairs where the candidate container's area is no larger than
	// mine -- closed planar regions can only contain strictly
	// smaller ones.
	for (size_t i = 0; i < recs.size(); ++i)
	{
		for (size_t j = 0; j < recs.size(); ++j)
		{
			if (i == j) continue;
			if (recs[j].area <= recs[i].area) continue;
			if (PointInsideFace(*recs[j].cls, plane, recs[i].sample))
				++recs[i].depth;
		}
	}

	// Emit one face per even-depth wire (outer boundary), pulling in
	// every wire at depth+1 that lies strictly inside it as a hole.
	// Do NOT call ShapeFix_Face::FixAddNaturalBound: for a plane the
	// natural bound is an infinite box, the fixer would treat the
	// outer wire as an inner hole, and Prism then sweeps it into an
	// annular slab with the inner top cap missing.
	BRep_Builder    bb;
	TopoDS_Compound comp;
	bb.MakeCompound(comp);
	int built = 0;
	for (size_t i = 0; i < recs.size(); ++i)
	{
		if (recs[i].depth % 2 != 0) continue;

		BRepBuilderAPI_MakeFace mk(plane, recs[i].wire);
		if (!mk.IsDone()) continue;

		for (size_t j = 0; j < recs.size(); ++j)
		{
			if (i == j) continue;
			if (recs[j].depth != recs[i].depth + 1) continue;
			if (recs[j].area >= recs[i].area) continue;
			if (PointInsideFace(*recs[i].cls, plane, recs[j].sample))
				mk.Add(recs[j].wire);
		}

		// FixOrientation flips CW wires so the face normal lines up
		// with the plane normal -- without this Prism's caps may
		// end up with inward normals (renderer hides them, looks
		// like a missing cap). It also re-orients inner wires
		// opposite to the outer, which is what OCCT expects for a
		// face with holes.
		TopoDS_Face   face = mk.Face();
		ShapeFix_Face fixer(face);
		fixer.FixOrientation();
		fixer.Perform();
		bb.Add(comp, fixer.Face());
		++built;
	}

	if (built == 0) return {};

	// Single-region sketches (the common case) keep returning a
	// bare Face -- avoids wrapping it in a Compound just so the
	// downstream Prism gets a face it could otherwise have received
	// directly, and preserves the existing topology-naming behavior
	// for single-region pads/pockets.
	if (built == 1)
	{
		TopExp_Explorer ex(comp, TopAbs_FACE);
		if (ex.More())
			return std::make_shared<brepkit::TopoShape>(ex.Current());
	}
	return std::make_shared<brepkit::TopoShape>(comp);
}

} // anonymous namespace

std::shared_ptr<brepkit::TopoShape> BuildSketchFace(
	const SketchIR& sk,
	const double*   origin,
	const double*   x_dir,
	const double*   normal)
{
	sketchlib::Scene                scene;
	cadapp::SketchBridge::GeoShapes solved;
	if (!cadapp::SketchBridge::ImportToScene(sk, scene, solved)) return nullptr;
	if (solved.empty()) return nullptr;
	scene.Solve(solved);

	auto wires = BuildWiresFromSolved(solved, origin, x_dir, normal);
	if (wires.IsNull() || wires->IsEmpty()) return nullptr;

	return WiresToFace(wires, origin, normal);
}

void RegisterSketchOps(brepgraph::OpRegistry& reg)
{
	// sketch_face: pulls the SketchIR back out of the type-erased
	// SketchVal, runs sketchlib's constraint solver, and lifts the
	// solved 2D wire onto the plane carried by the Vec3 inputs.
	// Plane params are separate inputs so changing the plane re-runs
	// the wire/face step without re-solving constraints.
	reg.Define("sketch_face", {"sketch", "origin", "x_dir", "normal"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetSketch(0);
			if (!sv.handle) return {};
			auto* sk = static_cast<const cadapp::SketchIR*>(sv.handle.get());

			Vec3 origin_v = ctx.GetVec3(1);
			Vec3 x_dir_v  = ctx.GetVec3(2);
			Vec3 normal_v = ctx.GetVec3(3);

			auto face = BuildSketchFace(
				*sk, origin_v.data(), x_dir_v.data(), normal_v.data());
			if (!face) return {};

			return MakeShapeVal(face);
		});

	// sketch_wire: like sketch_face but returns the stitched wire
	// directly without face filling. Used as the spine/path input to
	// Sweep. When the solved sketch chains into a single wire we
	// return that wire; multiple wires fall back to a compound and
	// the caller (TopoAlgo_Ext::Sweep) picks the first usable one.
	reg.Define("sketch_wire", {"sketch", "origin", "x_dir", "normal"}, {},
		[](EvalCtx& ctx) -> Val {
			auto sv = ctx.GetSketch(0);
			if (!sv.handle) return {};
			auto* sk = static_cast<const cadapp::SketchIR*>(sv.handle.get());

			Vec3 origin_v = ctx.GetVec3(1);
			Vec3 x_dir_v  = ctx.GetVec3(2);
			Vec3 normal_v = ctx.GetVec3(3);

			sketchlib::Scene                 scene;
			cadapp::SketchBridge::GeoShapes  solved;
			if (!cadapp::SketchBridge::ImportToScene(*sk, scene, solved)) return {};
			if (solved.empty()) return {};
			scene.Solve(solved);

			auto wires = BuildWiresFromSolved(
				solved, origin_v.data(), x_dir_v.data(), normal_v.data());
			if (wires.IsNull() || wires->IsEmpty()) return {};

			if (wires->Length() == 1) {
				return MakeShapeVal(std::make_shared<brepkit::TopoShape>(
					TopoDS::Wire(wires->Value(1))));
			}

			BRep_Builder    bb;
			TopoDS_Compound comp;
			bb.MakeCompound(comp);
			for (int i = 1; i <= wires->Length(); ++i) {
				bb.Add(comp, TopoDS::Wire(wires->Value(i)));
			}
			return MakeShapeVal(std::make_shared<brepkit::TopoShape>(comp));
		});
}

} // namespace cadapp
