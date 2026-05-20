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
#include <GProp_GProps.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Circ.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <ShapeFix_Face.hxx>

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

// Face area on the sketch plane, used to pick the outer wire when a
// sketch has holes. Builds a throwaway face just to measure -- cheap
// for a single wire and avoids re-implementing polygon shoelace in
// 3D world coords.
double WireFaceArea(const TopoDS_Wire& wire, const gp_Pln& plane)
{
	if (wire.IsNull()) return 0.0;
	BRepBuilderAPI_MakeFace mk(plane, wire);
	if (!mk.IsDone()) return 0.0;
	GProp_GProps props;
	BRepGProp::SurfaceProperties(mk.Face(), props);
	return std::abs(props.Mass());
}

std::shared_ptr<brepkit::TopoShape> WiresToFace(
	const Handle(TopTools_HSequenceOfShape)& wires,
	const double                              origin[3],
	const double                              normal[3])
{
	if (wires.IsNull() || wires->IsEmpty()) return {};

	gp_Pnt o(origin[0], origin[1], origin[2]);
	gp_Dir n(normal[0], normal[1], normal[2]);
	gp_Pln plane(o, n);

	// Find the largest-area wire and treat it as the outer boundary.
	// The rest are inner wires (holes). When there's only one wire,
	// outer_idx == 1 and the inner loop is a no-op.
	int    outer_idx  = 1;
	double outer_area = -1.0;
	for (int i = 1; i <= wires->Length(); ++i)
	{
		const TopoDS_Wire& w = TopoDS::Wire(wires->Value(i));
		double a = WireFaceArea(w, plane);
		if (a > outer_area) {
			outer_area = a;
			outer_idx  = i;
		}
	}

	const TopoDS_Wire& outer = TopoDS::Wire(wires->Value(outer_idx));

	// Do NOT call ShapeFix_Face::FixAddNaturalBound: for a plane the
	// natural bound is an infinite box, the fixer would treat our
	// wire as an inner hole, and Prism then sweeps it into an
	// annular slab with the inner top cap missing.
	BRepBuilderAPI_MakeFace mkFace(plane, outer);
	if (!mkFace.IsDone()) return {};

	for (int i = 1; i <= wires->Length(); ++i)
	{
		if (i == outer_idx) continue;
		const TopoDS_Wire& hole = TopoDS::Wire(wires->Value(i));
		mkFace.Add(hole);
	}

	// FixOrientation flips CW wires so the face normal lines up with
	// the plane normal -- without this Prism's caps may end up with
	// inward normals (renderer hides them, looks like a missing cap).
	// It also re-orients inner wires opposite to the outer, which is
	// what OCCT expects for a face with holes.
	TopoDS_Face   face = mkFace.Face();
	ShapeFix_Face fixer(face);
	fixer.FixOrientation();
	fixer.Perform();
	face = fixer.Face();

	return std::make_shared<brepkit::TopoShape>(face);
}

} // anonymous namespace

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

			sketchlib::Scene                 scene;
			cadapp::SketchBridge::GeoShapes  solved;
			if (!cadapp::SketchBridge::ImportToScene(*sk, scene, solved)) return {};
			if (solved.empty()) return {};
			scene.Solve(solved);

			auto wires = BuildWiresFromSolved(
				solved, origin_v.data(), x_dir_v.data(), normal_v.data());
			if (wires.IsNull() || wires->IsEmpty()) return {};

			auto face = WiresToFace(
				wires, origin_v.data(), normal_v.data());
			if (!face) return {};

			return MakeShapeVal(face);
		});
}

} // namespace cadapp
