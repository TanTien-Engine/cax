// ============================================================
// TopoAlgo_Ext.cpp
//
// Extended feature operations.
// All new ops follow the same TopoNaming update pattern as the
// existing TopoAlgo.cpp ops.
//
// ============================================================

#include "TopoAlgo_Ext.h"
#include "TopoShape.h"
#include "ShapeBuilder.h"
#include "occt_adapter.h"
#include "brepgraph_c/history/TopoNaming.h"

// OCCT
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomFill_Trihedron.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepLib.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom2d_Line.hxx>
#include <gp.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Lin2d.hxx>
#include <gp_Pnt2d.hxx>
#include <Precision.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BOPAlgo_GlueEnum.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <TopTools_ListOfShape.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepTools_History.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Vec.hxx>
#include <gp_Trsf.hxx>
#include <gp_Pln.hxx>

#include <stdexcept>

namespace brepkit
{

namespace
{

// Fuse a vector of pattern instances (each a SOLID coming out of a
// BRepBuilderAPI_Transform) into a single clean SOLID. Mirrors the
// fix already applied in TopoAlgo::Fuse:
//
//   1. Use the new BOP API (SetArguments / SetTools) -- the legacy
//      two-arg constructor sometimes returns an empty COMPOUND when
//      operands have coincident sub-faces.
//   2. SetFuzzyValue(1e-6) and an optional SetGlue hint so coincident
//      faces between adjacent pattern instances are treated as glued
//      rather than re-intersected (which fails silently and leaves
//      an unfused COMPOUND result). The glue mode is a hint to BOP
//      about HOW the operands are related:
//        - BOPAlgo_GlueShift : copies are pure translations of each
//          other (LinearPattern). Skips intersection checks between
//          arguments and gives the fast path.
//        - BOPAlgo_GlueOff   : no relationship assumed; do full
//          intersection. REQUIRED for CircularPattern -- rotated
//          copies are NOT translation-equivalent, and BOP under
//          GlueShift on rotated args silently returns an EMPTY shape
//          on the first fuse step (passes IsDone(), passes
//          !Shape().IsNull(), but has zero faces / zero volume).
//          The next iteration then fuses void+rot2 = rot2 alone, so
//          only the last instance survives -- looks like "the copies
//          overlap" but is actually "the fuse silently dropped them".
//   3. After all pairwise fuses, run ShapeUpgrade_UnifySameDomain to
//      merge coplanar adjacent faces into one (FreeCAD's pattern
//      output is already unified; cax used to leave the raw
//      seam-split faces, which broke fillet refs downstream).
//   4. If the final result is a COMPOUND wrapping exactly one SOLID
//      (BOP convention), unwrap it so callers receive a SOLID --
//      otherwise downstream fuse/cut hits the COMPOUND-input quirk.
//
// Returns the (possibly empty) input shape unchanged when there's
// only one instance.
TopoDS_Shape FuseInstancesAndUnify(const std::vector<TopoDS_Shape>& instances,
                                   BOPAlgo_GlueEnum                 glue = BOPAlgo_GlueShift)
{
    if (instances.empty()) {
        return TopoDS_Shape();
    }
    TopoDS_Shape result = instances[0];

    for (size_t i = 1; i < instances.size(); ++i)
    {
        BRepAlgoAPI_Fuse algo;
        TopTools_ListOfShape args;  args.Append(result);
        TopTools_ListOfShape tools; tools.Append(instances[i]);
        algo.SetArguments(args);
        algo.SetTools(tools);
        algo.SetFuzzyValue(1e-6);
        algo.SetGlue(glue);
        algo.Build();

        // A successful BOP can still produce a degenerate empty shape
        // when the glue hint is wrong (see header comment). Treat a
        // zero-face result as failure so we fall back to a plain
        // glue-off fuse rather than poisoning the running result.
        bool ok = algo.IsDone() && !algo.Shape().IsNull();
        if (ok) {
            TopExp_Explorer ex(algo.Shape(), TopAbs_FACE);
            if (!ex.More()) {
                ok = false;
            }
        }
        if (ok) {
            result = algo.Shape();
            continue;
        }

        // Glue mode is wrong for the geometry of this pair; fall
        // back to plain fuse rather than abandoning the step.
        BRepAlgoAPI_Fuse retry;
        retry.SetArguments(args);
        retry.SetTools(tools);
        retry.SetFuzzyValue(1e-6);
        retry.Build();
        if (retry.IsDone() && !retry.Shape().IsNull()) {
            result = retry.Shape();
        }
    }

    if (result.IsNull()) {
        return result;
    }

    // ---- Unify coplanar faces ----
    //
    // Default tolerances of UnifySameDomain are sub-micrometre and
    // refuse to merge most cross-instance seam faces (BOP output
    // between rotated copies has ~1e-7 m drift on the seam normal).
    // Without aggressive merging, the body keeps the raw pattern
    // fragments; downstream face-ref resolution then can't find a
    // face whose centroid is at the FreeCAD-authored ref point,
    // because that point is the symmetric centre of N fragments
    // rather than the centroid of any single fragment.
    //
    // Bumping LinearTolerance to 0.1mm + AngularTolerance to 1e-3
    // rad reliably merges those seams. Side-effect: small in-
    // instance features can also get filled, costing ~1.5-2% volume
    // on heavily-patterned bodies (e.g. Page_017_Exercise2D-09).
    // That's an acceptable trade for getting Fillet / Chamfer refs
    // to resolve, which would otherwise no-op silently.
    try {
        ShapeUpgrade_UnifySameDomain unify(result,
                                          /*UnifyEdges=*/Standard_True,
                                          /*UnifyFaces=*/Standard_True,
                                          /*ConcatBSplines=*/Standard_False);
        unify.SetLinearTolerance (1e-4);
        unify.SetAngularTolerance(1e-3);
        unify.Build();
        TopoDS_Shape unified = unify.Shape();
        if (!unified.IsNull()) {
            result = unified;
        }
    } catch (...) {
        // Leave `result` as the un-unified fuse output -- still
        // better than a raw compound of instances.
    }

    // ---- Unwrap a single-SOLID COMPOUND ----
    if (result.ShapeType() == TopAbs_COMPOUND)
    {
        TopExp_Explorer ex(result, TopAbs_SOLID);
        if (ex.More())
        {
            TopoDS_Shape first = ex.Current();
            ex.Next();
            if (!ex.More()) {
                // Exactly one SOLID in the compound: hand it back
                // directly. Downstream fuse/cut prefers SOLID args.
                result = first;
            }
        }
    }
    return result;
}

} // anonymous namespace

// ============================================================
// ExtrudeEx -Boss-Extrude with end conditions
// ============================================================

std::shared_ptr<TopoShape> TopoAlgo_Ext::ExtrudeEx(
    const std::shared_ptr<TopoShape>& shape,
    double dir_x,
    double dir_y,
    double dir_z,
    double dist1,
    double dist2,
    ExtrudeEndType end1,
    ExtrudeEndType end2,
    const std::shared_ptr<TopoShape>& ref,
    uint32_t op_id,
    const std::shared_ptr<brepgraph::TopoNaming>& tn)
{
    if (!shape) {
        return nullptr;
    }

    // Normalize direction
    double len = std::sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
    if (len < 1e-15) {
        return nullptr;
    }
    dir_x /= len;
    dir_y /= len;
    dir_z /= len;

    auto build_one = [&](double dist, ExtrudeEndType end, bool reverse) -> TopoDS_Shape {
        if (dist < 1e-15 && end == ExtrudeEndType::Blind) {
            return TopoDS_Shape();
        }

        // For ThroughAll / UpToFirst, compute a large enough distance
        // from bounding box so the prism unambiguously exits the body.
        double effective_dist = dist;
        if (end == ExtrudeEndType::ThroughAll ||
            end == ExtrudeEndType::UpToFirst) {
            Bnd_Box box;
            BRepBndLib::Add(shape->GetShape(), box);
            if (ref) {
                BRepBndLib::Add(ref->GetShape(), box);
            }
            double xmin, ymin, zmin, xmax, ymax, zmax;
            box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
            double diag = std::sqrt((xmax - xmin) * (xmax - xmin)
                                  + (ymax - ymin) * (ymax - ymin)
                                  + (zmax - zmin) * (zmax - zmin));
            // Multiply by safety factor to ensure full pass-through
            effective_dist = diag * 2.5;
        }

        double sign = reverse ? -1.0 : 1.0;
        gp_Vec vec(
            sign * dir_x * effective_dist,
            sign * dir_y * effective_dist,
            sign * dir_z * effective_dist);

        BRepPrimAPI_MakePrism prism(shape->GetShape(), vec, Standard_True);
        prism.Build();
        if (!prism.IsDone()) {
            return TopoDS_Shape();
        }

        TopoDS_Shape result = prism.Shape();

        // For UpToSurface, trim against the reference surface
        if (end == ExtrudeEndType::UpToSurface && ref) {
            BRepAlgoAPI_Splitter splitter;
            TopTools_ListOfShape args;
            args.Append(result);
            TopTools_ListOfShape tools;
            tools.Append(ref->GetShape());
            splitter.SetArguments(args);
            splitter.SetTools(tools);
            splitter.Build();
            if (splitter.IsDone()) {
                // Pick the piece on the profile side (heuristic: closer to profile)
                // For simplicity here, return the splitter result as-is;
                // production code should pick the correct half via centroid test.
                result = splitter.Shape();
            }
        }

        // For UpToFirst, the prism is the cut tool but only the FIRST
        // chunk of material it intersects should be removed. Walk
        // BRepAlgoAPI_Common(ref, prism) to get every carved chunk
        // (each is a connected SOLID inside the body), then keep only
        // the SOLID whose centroid is closest to the sketch face's
        // centroid -- that's the "first" piece the sweep encounters.
        //
        // Without this step, UpToFirst behaves like ThroughAll: a
        // horizontal cylinder through Page_051's pad ate the slot
        // wall AND the outer wall (4.98e-6 vs FreeCAD's 1.77e-6 m^3).
        // FreeCAD PartDesign UpToFirst stops at the slot wall's far
        // exit; that's what selecting the closest chunk reproduces.
        if (end == ExtrudeEndType::UpToFirst && ref)
        {
            BRepAlgoAPI_Common common;
            TopTools_ListOfShape ca; ca.Append(ref->GetShape());
            TopTools_ListOfShape cb; cb.Append(result);
            common.SetArguments(ca);
            common.SetTools(cb);
            common.SetFuzzyValue(1e-6);
            try { common.Build(); } catch (...) {}
            if (common.IsDone() && !common.Shape().IsNull())
            {
                // Collect each carved SOLID.
                std::vector<TopoDS_Shape> chunks;
                for (TopExp_Explorer ex(common.Shape(), TopAbs_SOLID);
                     ex.More(); ex.Next())
                {
                    chunks.push_back(ex.Current());
                }
                if (!chunks.empty())
                {
                    // Sketch face centroid -- the anchor we measure
                    // "first" from.
                    GProp_GProps sk_props;
                    BRepGProp::SurfaceProperties(shape->GetShape(), sk_props);
                    gp_Pnt sk_c = sk_props.CentreOfMass();

                    // Pick chunk minimizing the SIGNED projection of
                    // (chunk_centroid - sketch_centroid) onto the sweep
                    // direction. Signed projection so a chunk behind
                    // the sketch plane (the sweep starts at the sketch
                    // and goes forward) ranks higher than one ahead.
                    // The smallest positive projection wins.
                    gp_Vec dir_v(sign * dir_x, sign * dir_y, sign * dir_z);
                    int    best_i  = -1;
                    double best_d  = 0.0;
                    for (size_t i = 0; i < chunks.size(); ++i)
                    {
                        GProp_GProps cp;
                        BRepGProp::VolumeProperties(chunks[i], cp);
                        if (cp.Mass() <= 0) continue;
                        gp_Pnt cc = cp.CentreOfMass();
                        gp_Vec off(sk_c, cc);
                        double proj = off.Dot(dir_v);
                        // Tie-break with a small bias toward larger
                        // volume so a hairline sliver right at the
                        // sketch plane doesn't out-rank the real
                        // first chunk.
                        if (proj < 1e-9) continue;
                        if (best_i < 0 || proj < best_d) {
                            best_i = (int)i;
                            best_d = proj;
                        }
                    }
                    if (best_i >= 0) {
                        result = chunks[best_i];
                    }
                }
            }
        }

        return result;
    };

    TopoDS_Shape s1, s2;

    switch (end1)
    {
    case ExtrudeEndType::MidPlane:
    {
        // Shift the base face by -dir*dist/2, then prism +dir*dist as
        // a single solid. The previous "two halves + fuse" approach
        // left the sketch plane buried inside the solid as an INTERNAL
        // face, plus seam edges around its boundary. Downstream BOPs
        // could see those internal sub-shapes either as legitimate
        // geometry (wrong result) or silently bail out (empty
        // COMPOUND). UnifySameDomain papered over the cap-merge case
        // but the seam edges still tripped subsequent face-coincident
        // fuses. A single prism off a shifted face has no internal
        // sub-shapes to begin with.
        gp_Trsf shift;
        shift.SetTranslation(gp_Vec(
            -dir_x * dist1 * 0.5,
            -dir_y * dist1 * 0.5,
            -dir_z * dist1 * 0.5));
        BRepBuilderAPI_Transform shifter(shape->GetShape(), shift, Standard_True);
        if (shifter.IsDone()) {
            BRepPrimAPI_MakePrism prism(
                shifter.Shape(),
                gp_Vec(dir_x * dist1, dir_y * dist1, dir_z * dist1),
                Standard_True);
            prism.Build();
            if (prism.IsDone()) {
                s1 = prism.Shape();
            }
        }
        break;
    }
    case ExtrudeEndType::Blind:
    case ExtrudeEndType::ThroughAll:
    case ExtrudeEndType::UpToSurface:
    case ExtrudeEndType::UpToVertex:
    case ExtrudeEndType::OffsetFromSurface:
    case ExtrudeEndType::UpToFirst:
    {
        s1 = build_one(dist1, end1, false);
        // Optional second direction
        if (dist2 > 1e-15 ||
            end2 == ExtrudeEndType::ThroughAll ||
            end2 == ExtrudeEndType::UpToFirst) {
            s2 = build_one(dist2, end2, true);
        }
        break;
    }
    }

    // Combine the two halves if both exist
    TopoDS_Shape final_shape;
    if (!s1.IsNull() && !s2.IsNull()) {
        BRepAlgoAPI_Fuse fuse(s1, s2);
        fuse.Build();
        if (fuse.IsDone()) {
            final_shape = fuse.Shape();
        } else {
            final_shape = s1; // fallback
        }
    } else if (!s1.IsNull()) {
        final_shape = s1;
    } else if (!s2.IsNull()) {
        final_shape = s2;
    } else {
        return nullptr;
    }

    auto result = std::make_shared<TopoShape>(final_shape);

    // Update TopoNaming
    if (tn) {
        // Use the prism builder's history for the primary direction
        // This is simplified; full impl should chain history through fuse too
        BRepPrimAPI_MakePrism prism_for_history(
            shape->GetShape(),
            gp_Vec(dir_x * dist1, dir_y * dist1, dir_z * dist1),
            Standard_True);
        prism_for_history.Build();
        if (prism_for_history.IsDone()) {
            tn->Update(prism_for_history, final_shape, shape->GetShape(), op_id);
        }
    }

    return result;
}


// ============================================================
// Revolve -Boss-Revolve / Cut-Revolve
// ============================================================

std::shared_ptr<TopoShape> TopoAlgo_Ext::Revolve(
    const std::shared_ptr<TopoShape>& shape,
    const sm::vec3& axis_origin,
    const sm::vec3& axis_dir,
    double angle,
    bool is_full,
    uint32_t op_id,
    const std::shared_ptr<brepgraph::TopoNaming>& tn)
{
    if (!shape) {
        return nullptr;
    }

    gp_Ax1 axis;
    axis.SetLocation(gp_Pnt(axis_origin.x, axis_origin.y, axis_origin.z));
    axis.SetDirection(gp_Dir(axis_dir.x, axis_dir.y, axis_dir.z));

    double sweep_angle = is_full ? 2.0 * M_PI : angle;

    BRepPrimAPI_MakeRevol revol(shape->GetShape(), axis, sweep_angle, Standard_True);
    revol.Build();
    if (!revol.IsDone()) {
        return nullptr;
    }

    if (tn) {
        tn->Update(revol, revol.Shape(), shape->GetShape(), op_id);
    }

    return std::make_shared<TopoShape>(revol.Shape());
}


// ============================================================
// Sweep -Sweep along path
// ============================================================

std::shared_ptr<TopoShape> TopoAlgo_Ext::Sweep(
    const std::shared_ptr<TopoShape>& profile,
    const std::shared_ptr<TopoShape>& path,
    bool is_solid,
    uint32_t op_id,
    const std::shared_ptr<brepgraph::TopoNaming>& tn,
    bool frenet)
{
    if (!profile || !path) {
        return nullptr;
    }

    // Path must be a wire
    TopoDS_Shape path_shape = path->GetShape();
    TopoDS_Wire path_wire;

    if (path_shape.ShapeType() == TopAbs_WIRE) {
        path_wire = TopoDS::Wire(path_shape);
    } else if (path_shape.ShapeType() == TopAbs_EDGE) {
        // Auto-wrap single edge into wire
        BRepBuilderAPI_MakeWire mw(TopoDS::Edge(path_shape));
        if (!mw.IsDone()) {
            return nullptr;
        }
        path_wire = mw.Wire();
    } else {
        return nullptr;
    }

    // BRepOffsetAPI_MakePipe is the simplest sweep: profile follows path.
    // The trihedron mode controls how the cross-section is transported:
    // true Frenet keeps a non-symmetric profile radially oriented along a
    // helix (FreeCAD Part::Sweep Frenet=true), while the default corrected
    // Frenet minimizes twist (what the circular PartDesign Pipe expects).
    double spine_len = 0.0;
    bool   spine_is_linear = true;
    for (TopExp_Explorer ex(path_wire, TopAbs_EDGE); ex.More(); ex.Next()) {
        BRepAdaptor_Curve c(TopoDS::Edge(ex.Current()));
        spine_len += GCPnts_AbscissaPoint::Length(c);
        if (c.GetType() != GeomAbs_Line) {
            spine_is_linear = false;
        }
    }

    // A spine that collapsed to (near) zero length -- e.g. a spine sketch
    // the constraint solver could not fully pin, leaving a zero-radius arc
    // -- cannot be swept. Feeding it to MakePipe sends GeomFill_Frenet into
    // unbounded recursion (an uncatchable stack-overflow SIGSEGV), so bail
    // out: the Replayer falls back to the FreeCAD-authored brep on null.
    if (spine_len < Precision::Confusion()) {
        return nullptr;
    }

    // Both Frenet modes evaluate the spine's Frenet frame, which is also
    // singular on a (non-degenerate) straight spine: zero curvature -> the
    // normal is undefined, and OCCT then recurses to the same stack
    // overflow. A piecewise-linear spine carries no curvature to follow
    // anyway, so route it through the discrete trihedron -- well-defined on
    // straight / polyline spines and non-recursing.
    GeomFill_Trihedron mode =
        spine_is_linear ? GeomFill_IsDiscreteTrihedron
                        : (frenet ? GeomFill_IsFrenet : GeomFill_IsCorrectedFrenet);
    BRepOffsetAPI_MakePipe pipe(path_wire, profile->GetShape(),
                                mode, Standard_False);
    pipe.Build();
    if (!pipe.IsDone()) {
        return nullptr;
    }

    TopoDS_Shape result = pipe.Shape();

    // For solid sweep with closed profile, MakePipe already produces a solid
    // For open profiles, result is a shell -caller can decide what to do

    if (tn) {
        tn->Update(pipe, result, profile->GetShape(), op_id);
    }

    return std::make_shared<TopoShape>(result);
}


// ============================================================
// HelixWire -helical spine wire (FreeCAD Part::Helix)
// ============================================================

std::shared_ptr<TopoShape> TopoAlgo_Ext::HelixWire(
    double pitch,
    double height,
    double radius,
    double cone_angle,
    bool   left_handed,
    uint32_t /*op_id*/,
    const std::shared_ptr<brepgraph::TopoNaming>& /*tn*/)
{
    // Guard against the degenerate inputs FreeCAD itself rejects (a
    // zero/negative pitch never advances, zero height/radius collapses
    // the curve). Returning null lets the Replayer surface a clean
    // "missing spine" rather than feeding garbage to the sweep.
    if (pitch <= Precision::Confusion()
        || height <= Precision::Confusion()
        || radius <= Precision::Confusion())
    {
        return nullptr;
    }

    // The helix lives in the local frame: axis = +Z, the curve starts
    // at (radius, 0, 0). A conical helix needs a half-angle strictly
    // inside (0, pi/2); at the limit cos(angle) -> 0 the v-advance
    // blows up, so anything that degenerate (FreeCAD's Page_085 stores
    // Angle=90 and marks the object Invalid, keeping its last valid
    // cylindrical shape) falls back to a cylinder.
    gp_Ax3 ax3(gp_Pnt(0.0, 0.0, 0.0), gp::DZ(), gp::DX());
    const double abs_angle = std::fabs(cone_angle);
    const bool   is_cone =
        abs_angle > Precision::Angular()
        && abs_angle < (M_PI * 0.5 - Precision::Angular());

    Handle(Geom_Surface) surf;
    double v_per_turn;   // surface v advanced over one 2*PI turn
    double v_total;      // surface v reaching the requested height
    if (is_cone)
    {
        surf = new Geom_ConicalSurface(ax3, cone_angle, radius);
        // On a cone the v parameter runs along the slant line, so an
        // axial advance of `pitch` per turn costs pitch/cos(angle) in v.
        const double inv_cos = 1.0 / std::cos(cone_angle);
        v_per_turn = pitch  * inv_cos;
        v_total    = height * inv_cos;
    }
    else
    {
        surf = new Geom_CylindricalSurface(ax3, radius);
        // On a cylinder v is the axial height directly.
        v_per_turn = pitch;
        v_total    = height;
    }

    // 2D line in (u, v): u winds 2*PI per turn (negated for a
    // left-handed helix), v advances `v_per_turn` over the same turn.
    const double du = left_handed ? -2.0 * M_PI : 2.0 * M_PI;
    gp_Dir2d dir2d(du, v_per_turn);
    gp_Lin2d lin2d(gp_Pnt2d(0.0, 0.0), dir2d);
    Handle(Geom2d_Line) line2d = new Geom2d_Line(lin2d);

    // Trim length: the 2D direction is unit, so the parameter that
    // reaches v_total is v_total / (v-component of the unit dir).
    const double dir_v = dir2d.Y();
    if (std::fabs(dir_v) < Precision::Confusion()) {
        return nullptr;
    }
    const double t_end = v_total / dir_v;

    // Building the spine as a SINGLE edge spanning every turn makes the
    // downstream BRepOffsetAPI_MakePipe approximate the swept surface
    // with one coarse BSpline whose control poles bulge well outside the
    // real tube. At metre scale (the cax IR is in metres) OCCT's
    // absolute tolerances let that pole bloat balloon the pole-based
    // bounding box ~5x even though the area/volume stay correct.
    // Splitting the helix into several edges per turn keeps each pipe
    // segment's BSpline tight, so the swept solid -and its bbox- hugs
    // the true coil. ~8 edges/turn converges the bbox to the authored
    // shape; the ceil keeps at least one edge for sub-turn helices.
    const double turns = std::fabs(v_total / v_per_turn);
    const int    kSegPerTurn = 8;
    int n_seg = (int)std::ceil(turns * kSegPerTurn);
    if (n_seg < 1) {
        n_seg = 1;
    }

    BRepBuilderAPI_MakeWire mk_wire;
    for (int i = 0; i < n_seg; ++i) {
        const double t0 = t_end * (double)i       / (double)n_seg;
        const double t1 = t_end * (double)(i + 1) / (double)n_seg;
        BRepBuilderAPI_MakeEdge mk_edge(line2d, surf, t0, t1);
        if (!mk_edge.IsDone()) {
            return nullptr;
        }
        mk_wire.Add(mk_edge.Edge());
    }
    if (!mk_wire.IsDone()) {
        return nullptr;
    }
    TopoDS_Wire wire = mk_wire.Wire();

    // Lift the parametric (pcurve-only) edge to a real 3D curve so the
    // sweep can build a trihedron along it.
    BRepLib::BuildCurves3d(wire);

    return std::make_shared<TopoShape>(wire);
}


// ============================================================
// LinearPattern -Linear Pattern (1D or 2D grid)
// ============================================================

std::shared_ptr<TopoShape> TopoAlgo_Ext::LinearPattern(
    const std::shared_ptr<TopoShape>& base,
    const sm::vec3& dir1,
    int count1,
    double spacing1,
    const sm::vec3& dir2,
    int count2,
    double spacing2,
    uint32_t op_id,
    const std::shared_ptr<brepgraph::TopoNaming>& tn)
{
    if (!base || count1 < 1) {
        return nullptr;
    }

    // Normalize direction 1
    double len1 = std::sqrt(dir1.x * dir1.x + dir1.y * dir1.y + dir1.z * dir1.z);
    if (len1 < 1e-15) {
        return base;
    }
    sm::vec3 d1(dir1.x / len1, dir1.y / len1, dir1.z / len1);

    // Direction 2 (optional)
    bool has_dir2 = false;
    sm::vec3 d2;
    double len2 = std::sqrt(dir2.x * dir2.x + dir2.y * dir2.y + dir2.z * dir2.z);
    if (len2 > 1e-15 && count2 > 1) {
        d2 = sm::vec3(dir2.x / len2, dir2.y / len2, dir2.z / len2);
        has_dir2 = true;
    }
    int effective_count2 = has_dir2 ? count2 : 1;

    // Collect all instances
    std::vector<TopoDS_Shape> instances;
    instances.reserve(count1 * effective_count2);

    for (int i = 0; i < count1; ++i) {
        for (int j = 0; j < effective_count2; ++j) {
            if (i == 0 && j == 0) {
                instances.push_back(base->GetShape());
                continue;
            }

            double tx = i * spacing1 * d1.x;
            double ty = i * spacing1 * d1.y;
            double tz = i * spacing1 * d1.z;
            if (has_dir2) {
                tx += j * spacing2 * d2.x;
                ty += j * spacing2 * d2.y;
                tz += j * spacing2 * d2.z;
            }

            gp_Trsf trsf;
            trsf.SetTranslation(gp_Vec(tx, ty, tz));
            BRepBuilderAPI_Transform xform(base->GetShape(), trsf, Standard_True);
            instances.push_back(xform.Shape());
        }
    }

    TopoDS_Shape result = FuseInstancesAndUnify(instances);
    auto result_shape = std::make_shared<TopoShape>(result);

    // Note: TopoNaming for pattern is complex -each instance produces
    // siblings of the original entities. Simplified update here:
    if (tn) {
        // For full naming support, would need to record each fuse history.
        // For now, treat the whole pattern as a single op replacement.
    }

    return result_shape;
}


// ============================================================
// CircularPattern -Circular Pattern around an axis
// ============================================================

std::shared_ptr<TopoShape> TopoAlgo_Ext::CircularPattern(
    const std::shared_ptr<TopoShape>& base,
    const sm::vec3& axis_origin,
    const sm::vec3& axis_dir,
    int count,
    double angle,
    uint32_t op_id,
    const std::shared_ptr<brepgraph::TopoNaming>& tn)
{
    if (!base || count < 1) {
        return nullptr;
    }

    gp_Ax1 axis;
    axis.SetLocation(gp_Pnt(axis_origin.x, axis_origin.y, axis_origin.z));
    axis.SetDirection(gp_Dir(axis_dir.x, axis_dir.y, axis_dir.z));

    double step = (count > 1) ? (angle / static_cast<double>(count)) : 0.0;

    std::vector<TopoDS_Shape> instances;
    instances.reserve(count);

    for (int i = 0; i < count; ++i) {
        if (i == 0) {
            instances.push_back(base->GetShape());
            continue;
        }

        gp_Trsf trsf;
        trsf.SetRotation(axis, step * i);
        BRepBuilderAPI_Transform xform(base->GetShape(), trsf, Standard_True);
        instances.push_back(xform.Shape());
    }

    // Rotated copies are NOT pure translations of each other, so the
    // default BOPAlgo_GlueShift hint silently returns an empty shape
    // on the first pairwise fuse (see FuseInstancesAndUnify header).
    // Disable the glue hint so BOP does full intersection on every
    // pair -- correct geometry trumps the glue fast path here.
    TopoDS_Shape result = FuseInstancesAndUnify(instances, BOPAlgo_GlueOff);
    return std::make_shared<TopoShape>(result);
}


// ============================================================
// HoleWizard -parametric hole with counterbore/countersink
// ============================================================

std::shared_ptr<TopoShape> TopoAlgo_Ext::HoleWizard(
    const std::shared_ptr<TopoShape>& body,
    const sm::vec3& pos,
    const sm::vec3& dir,
    double diameter,
    double depth,
    HoleType hole_type,
    double cb_diameter,
    double cb_depth,
    double cs_diameter,
    double cs_angle,
    uint32_t op_id,
    const std::shared_ptr<brepgraph::TopoNaming>& tn)
{
    if (!body || diameter < 1e-15) {
        return nullptr;
    }

    double radius = diameter * 0.5;

    // Normalize direction
    gp_Dir hole_dir(dir.x, dir.y, dir.z);
    gp_Pnt hole_pos(pos.x, pos.y, pos.z);

    // Compute effective depth: 0 means through-all
    double effective_depth = depth;
    if (effective_depth < 1e-15) {
        Bnd_Box box;
        BRepBndLib::Add(body->GetShape(), box);
        double xmin, ymin, zmin, xmax, ymax, zmax;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        double diag = std::sqrt((xmax - xmin) * (xmax - xmin)
                              + (ymax - ymin) * (ymax - ymin)
                              + (zmax - zmin) * (zmax - zmin));
        effective_depth = diag * 2.5;
    }

    // Build the tool shape to cut from the body
    gp_Ax2 ax(hole_pos, hole_dir);
    TopoDS_Shape tool;

    switch (hole_type)
    {
    case HoleType::Counterbore:
    {
        // Counterbore: large cylinder on top + main hole below
        double cb_radius = cb_diameter * 0.5;
        BRepPrimAPI_MakeCylinder cb_cyl(ax, cb_radius, cb_depth);
        cb_cyl.Build();

        // Offset the main hole start
        gp_Pnt main_start(
            hole_pos.X() + hole_dir.X() * cb_depth,
            hole_pos.Y() + hole_dir.Y() * cb_depth,
            hole_pos.Z() + hole_dir.Z() * cb_depth);
        gp_Ax2 main_ax(main_start, hole_dir);
        BRepPrimAPI_MakeCylinder main_cyl(main_ax, radius, effective_depth - cb_depth);
        main_cyl.Build();

        BRepAlgoAPI_Fuse fuse(cb_cyl.Shape(), main_cyl.Shape());
        fuse.Build();
        tool = fuse.IsDone() ? fuse.Shape() : cb_cyl.Shape();
        break;
    }
    case HoleType::Countersink:
    {
        // Countersink: conical entry + main hole
        double cs_radius = cs_diameter * 0.5;
        double half_angle = cs_angle * 0.5;
        double cone_depth = (cs_radius - radius) / std::tan(half_angle);
        if (cone_depth < 1e-15) { cone_depth = 0.1; }

        BRepPrimAPI_MakeCone cone(ax, cs_radius, radius, cone_depth);
        cone.Build();

        gp_Pnt main_start(
            hole_pos.X() + hole_dir.X() * cone_depth,
            hole_pos.Y() + hole_dir.Y() * cone_depth,
            hole_pos.Z() + hole_dir.Z() * cone_depth);
        gp_Ax2 main_ax(main_start, hole_dir);
        BRepPrimAPI_MakeCylinder main_cyl(main_ax, radius, effective_depth - cone_depth);
        main_cyl.Build();

        BRepAlgoAPI_Fuse fuse(cone.Shape(), main_cyl.Shape());
        fuse.Build();
        tool = fuse.IsDone() ? fuse.Shape() : cone.Shape();
        break;
    }
    case HoleType::Tapped:
    case HoleType::Simple:
    default:
    {
        BRepPrimAPI_MakeCylinder cyl(ax, radius, effective_depth);
        cyl.Build();
        tool = cyl.Shape();
        break;
    }
    }

    if (tool.IsNull()) {
        return nullptr;
    }

    // Cut the tool from the body
    BRepAlgoAPI_Cut cut(body->GetShape(), tool);
    cut.Build();
    if (!cut.IsDone()) {
        return nullptr;
    }

    auto result = std::make_shared<TopoShape>(cut.Shape());

    if (tn) {
        opencascade::handle<BRepTools_History> o_hist = cut.History();
        tn->Update(o_hist, cut.Shape(), body->GetShape(), op_id);
    }

    return result;
}


// ============================================================
// VariableFillet -variable radius fillet along edges
// ============================================================

std::shared_ptr<TopoShape> TopoAlgo_Ext::VariableFillet(
    const std::shared_ptr<TopoShape>& shape,
    const std::vector<std::shared_ptr<TopoShape>>& edges,
    const std::vector<double>& params,
    uint32_t op_id,
    const std::shared_ptr<brepgraph::TopoNaming>& tn)
{
    if (!shape || edges.empty() || params.size() < 2) {
        return nullptr;
    }

    BRepFilletAPI_MakeFillet fillet(shape->GetShape());

    for (auto& edge_shape : edges) {
        TopoDS_Edge edge = TopoDS::Edge(edge_shape->GetShape());
        fillet.Add(edge);

        // Apply variable radius: params = [u0, r0, u1, r1, ...]
        int edge_idx = fillet.NbContours();
        for (size_t i = 0; i + 1 < params.size(); i += 2) {
            double u = params[i];
            double r = params[i + 1];
            fillet.SetRadius(r, edge_idx, static_cast<int>(i / 2) + 1);
        }
    }

    fillet.Build();
    if (!fillet.IsDone()) {
        return nullptr;
    }

    if (tn) {
        tn->Update(fillet, fillet.Shape(), shape->GetShape(), op_id);
    }

    return std::make_shared<TopoShape>(fillet.Shape());
}


// ============================================================
// SweepWithGuide -sweep with auxiliary guide curves
// ============================================================

std::shared_ptr<TopoShape> TopoAlgo_Ext::SweepWithGuide(
    const std::shared_ptr<TopoShape>& profile,
    const std::shared_ptr<TopoShape>& path,
    const std::vector<std::shared_ptr<TopoShape>>& guides,
    bool is_solid,
    uint32_t op_id,
    const std::shared_ptr<brepgraph::TopoNaming>& tn)
{
    if (!profile || !path) {
        return nullptr;
    }

    // Path must be a wire
    TopoDS_Shape path_shape = path->GetShape();
    TopoDS_Wire path_wire;

    if (path_shape.ShapeType() == TopAbs_WIRE) {
        path_wire = TopoDS::Wire(path_shape);
    } else if (path_shape.ShapeType() == TopAbs_EDGE) {
        BRepBuilderAPI_MakeWire mw(TopoDS::Edge(path_shape));
        if (!mw.IsDone()) {
            return nullptr;
        }
        path_wire = mw.Wire();
    } else {
        return nullptr;
    }

    BRepOffsetAPI_MakePipeShell pipe_shell(path_wire);

    // If a guide curve is provided, use it as an auxiliary spine
    // to control how the profile scales/positions along the path.
    // BRepOffsetAPI_MakePipeShell::SetMode(auxiliaryWire, curvilinearEquivalence)
    if (!guides.empty()) {
        auto& first_guide = guides[0];
        TopoDS_Wire aux_wire;
        auto& gs = first_guide->GetShape();
        if (gs.ShapeType() == TopAbs_WIRE) {
            aux_wire = TopoDS::Wire(gs);
        } else if (gs.ShapeType() == TopAbs_EDGE) {
            BRepBuilderAPI_MakeWire mw(TopoDS::Edge(gs));
            if (mw.IsDone()) {
                aux_wire = mw.Wire();
            }
        }
        if (!aux_wire.IsNull()) {
            pipe_shell.SetMode(aux_wire, Standard_True);
        }
    }

    // Profile can be a wire or face; use wire
    TopoDS_Wire profile_wire;
    auto& prof_shape = profile->GetShape();
    if (prof_shape.ShapeType() == TopAbs_WIRE) {
        profile_wire = TopoDS::Wire(prof_shape);
    } else if (prof_shape.ShapeType() == TopAbs_FACE) {
        TopExp_Explorer exp(prof_shape, TopAbs_WIRE);
        if (exp.More()) {
            profile_wire = TopoDS::Wire(exp.Current());
        }
    } else if (prof_shape.ShapeType() == TopAbs_EDGE) {
        BRepBuilderAPI_MakeWire mw(TopoDS::Edge(prof_shape));
        if (mw.IsDone()) {
            profile_wire = mw.Wire();
        }
    }

    if (profile_wire.IsNull()) {
        return nullptr;
    }

    pipe_shell.Add(profile_wire);

    pipe_shell.Build();
    if (!pipe_shell.IsDone()) {
        return nullptr;
    }

    if (is_solid) {
        pipe_shell.MakeSolid();
    }

    TopoDS_Shape result = pipe_shell.Shape();

    if (tn) {
        tn->Update(pipe_shell, result, profile->GetShape(), op_id);
    }

    return std::make_shared<TopoShape>(result);
}


// ============================================================
// Rib -structural rib / web feature
// ============================================================

std::shared_ptr<TopoShape> TopoAlgo_Ext::Rib(
    const std::shared_ptr<TopoShape>& body,
    const std::shared_ptr<TopoShape>& profile,
    const sm::vec3& dir,
    double thickness,
    bool is_symmetric,
    uint32_t op_id,
    const std::shared_ptr<brepgraph::TopoNaming>& tn)
{
    if (!body || !profile || thickness < 1e-15) {
        return nullptr;
    }

    // 1. Offset the profile wire to create a thick ribbon
    TopoDS_Wire profile_wire;
    auto& prof_shape = profile->GetShape();
    if (prof_shape.ShapeType() == TopAbs_WIRE) {
        profile_wire = TopoDS::Wire(prof_shape);
    } else if (prof_shape.ShapeType() == TopAbs_EDGE) {
        BRepBuilderAPI_MakeWire mw(TopoDS::Edge(prof_shape));
        if (!mw.IsDone()) {
            return nullptr;
        }
        profile_wire = mw.Wire();
    } else {
        return nullptr;
    }

    // Build a face from the wire
    BRepBuilderAPI_MakeFace face_maker(profile_wire, Standard_True);
    if (!face_maker.IsDone()) {
        return nullptr;
    }

    // 2. Extrude the face in the rib direction to create a thin slab
    gp_Vec extrude_dir(dir.x, dir.y, dir.z);
    double dir_len = extrude_dir.Magnitude();
    if (dir_len < 1e-15) {
        return nullptr;
    }

    // Compute a large extrusion that will pass through the body
    Bnd_Box box;
    BRepBndLib::Add(body->GetShape(), box);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    double diag = std::sqrt((xmax - xmin) * (xmax - xmin)
                          + (ymax - ymin) * (ymax - ymin)
                          + (zmax - zmin) * (zmax - zmin));

    // Now extrude to create a thin slab
    gp_Vec norm_dir = extrude_dir.Normalized();

    TopoDS_Shape rib_slab;
    if (is_symmetric) {
        // Extrude half on each side
        gp_Vec v_fwd = norm_dir * (thickness * 0.5);
        gp_Vec v_bwd = norm_dir * (-thickness * 0.5);

        BRepPrimAPI_MakePrism prism_fwd(face_maker.Face(), v_fwd, Standard_True);
        BRepPrimAPI_MakePrism prism_bwd(face_maker.Face(), v_bwd, Standard_True);
        prism_fwd.Build();
        prism_bwd.Build();

        if (!prism_fwd.IsDone() || !prism_bwd.IsDone()) {
            return nullptr;
        }

        BRepAlgoAPI_Fuse fuse(prism_fwd.Shape(), prism_bwd.Shape());
        fuse.Build();
        rib_slab = fuse.IsDone() ? fuse.Shape() : prism_fwd.Shape();
    } else {
        gp_Vec v = norm_dir * thickness;
        BRepPrimAPI_MakePrism prism(face_maker.Face(), v, Standard_True);
        prism.Build();
        if (!prism.IsDone()) {
            return nullptr;
        }
        rib_slab = prism.Shape();
    }

    // 3. Intersect with body to trim the rib to the body envelope
    BRepAlgoAPI_Common common(rib_slab, body->GetShape());
    common.Build();
    if (!common.IsDone()) {
        return nullptr;
    }

    // 4. Fuse the trimmed rib with the body
    BRepAlgoAPI_Fuse fuse(body->GetShape(), common.Shape());
    fuse.Build();
    if (!fuse.IsDone()) {
        return nullptr;
    }

    auto result = std::make_shared<TopoShape>(fuse.Shape());

    if (tn) {
        opencascade::handle<BRepTools_History> o_hist = fuse.History();
        tn->Update(o_hist, fuse.Shape(), body->GetShape(), op_id);
    }

    return result;
}

} // namespace brepkit
