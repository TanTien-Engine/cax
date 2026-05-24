#include "cadapp_c/resolve/TopoRefResolver.h"
#include "cadapp_c/resolve/TopoGeomUtils.h"

#include "brepgraph_c/history/TopoNaming.h"
#include "brepgraph_c/history/HistGraph.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Tool.hxx>
#include <Extrema_ExtPC.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace cadapp
{

namespace
{

double Norm3(const double v[3])
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void Normalize(double v[3])
{
    double n = Norm3(v);
    if (n > 1e-15)
    {
        v[0] /= n;
        v[1] /= n;
        v[2] /= n;
    }
}

// Combined position + normal distance. Mirrors the weight used by
// the temp reference (0.1 on the normal term).
double MatchScore(const double  a_pt[3],
                  const double  a_n[3],
                  const gp_Pnt& b_pt,
                  const gp_Dir& b_n,
                  double        normal_weight = 0.1)
{
    double dx  = a_pt[0] - b_pt.X();
    double dy  = a_pt[1] - b_pt.Y();
    double dz  = a_pt[2] - b_pt.Z();
    double pos = std::sqrt(dx * dx + dy * dy + dz * dz);

    // a_n may not be unit length; b_n is already a gp_Dir.
    double an[3] = { a_n[0], a_n[1], a_n[2] };
    if (Norm3(an) > 1e-12) {
        Normalize(an);
    }

    // Allow inverted normals (signs flipped by orientation).
    double dot       = an[0] * b_n.X() + an[1] * b_n.Y() + an[2] * b_n.Z();
    double norm_diff = 1.0 - std::fabs(dot);  // 0 colinear, 1 perpendicular
    return pos + normal_weight * norm_diff;
}

// Closest distance from `p` to the curve underlying `edge`, clamped
// to the edge's parameter range. Returns +inf for degenerate edges
// or when Extrema fails. Used as the primary edge-match score so a
// ref midpoint that physically LIES ON a longer / merged cax edge
// scores tightly even when its parameter-midpoint sits mm away.
double PointToEdgeDistance(const gp_Pnt& p, const TopoDS_Edge& edge)
{
    if (BRep_Tool::Degenerated(edge)) {
        return std::numeric_limits<double>::infinity();
    }
    BRepAdaptor_Curve curve(edge);
    double f = curve.FirstParameter();
    double l = curve.LastParameter();
    Extrema_ExtPC extrema(p, curve, f, l);
    if (!extrema.IsDone()) {
        return std::numeric_limits<double>::infinity();
    }
    double best = std::numeric_limits<double>::infinity();
    int n = extrema.NbExt();
    for (int i = 1; i <= n; ++i) {
        double d2 = extrema.SquareDistance(i);
        if (d2 < best) best = d2;
    }
    // Also test the two endpoints -- Extrema can miss when the
    // minimum lies AT the parameter boundary (extremum is on the
    // open side of f/l so the solver flags it as out-of-range and
    // skips). That matters for endpoint-snapped picks like fillet
    // chains where the ref midpoint lands right at a chain elbow.
    double d_f = p.SquareDistance(curve.Value(f));
    double d_l = p.SquareDistance(curve.Value(l));
    if (d_f < best) best = d_f;
    if (d_l < best) best = d_l;
    return (best == std::numeric_limits<double>::infinity())
        ? best
        : std::sqrt(best);
}

uint32_t LookupUid(brepgraph::TopoNaming* naming,
                   TopoRefIR::Kind       kind,
                   const TopoDS_Shape&   sub)
{
    if (!naming) {
        return 0;
    }

    std::shared_ptr<brepgraph::HistGraph> hg;
    switch (kind)
    {
    case TopoRefIR::Kind::Vertex:
        hg = naming->GetVertexGraph();
        break;
    case TopoRefIR::Kind::Edge:
        hg = naming->GetEdgeGraph();
        break;
    case TopoRefIR::Kind::Face:
        hg = naming->GetFaceGraph();
        break;
    default:
        return 0;
    }
    if (!hg) {
        return 0;
    }

    uint32_t uid = hg->GetUID(sub);
    return (uid == 0xFFFFFFFF) ? 0 : uid;
}

} // anonymous namespace


std::vector<ResolvedRef> TopoRefResolver::Resolve(const TopoDS_Shape&            shape,
                                                  const std::vector<TopoRefIR>&  refs,
                                                  brepgraph::TopoNaming*          naming,
                                                  double                         tolerance)
{
    std::vector<ResolvedRef> out;
    out.reserve(refs.size());

    if (shape.IsNull())
    {
        out.assign(refs.size(), ResolvedRef{});
        return out;
    }

    TopTools_IndexedMapOfShape edgeMap;
    TopTools_IndexedMapOfShape faceMap;
    TopTools_IndexedMapOfShape vertMap;
    TopExp::MapShapes(shape, TopAbs_EDGE,   edgeMap);
    TopExp::MapShapes(shape, TopAbs_FACE,   faceMap);
    TopExp::MapShapes(shape, TopAbs_VERTEX, vertMap);

    for (const auto& ref : refs)
    {
        ResolvedRef r;
        r.kind       = ref.kind;
        r.match_dist = std::numeric_limits<double>::max();

        switch (ref.kind)
        {
        case TopoRefIR::Kind::Edge:
        {
            gp_Pnt ref_pt(ref.point[0], ref.point[1], ref.point[2]);
            for (int i = 1; i <= edgeMap.Extent(); ++i)
            {
                const TopoDS_Edge& e = TopoDS::Edge(edgeMap.FindKey(i));
                gp_Pnt mid;
                gp_Dir tan;
                if (!EdgeMidpoint(e, mid, tan)) {
                    continue;
                }

                // Primary score: point-to-CURVE distance, not point-
                // to-midpoint. When cax merges what FreeCAD kept as
                // two adjacent edges into one longer edge, the
                // merged edge's parameter midpoint shifts by half
                // the neighbour's length (we saw a 9 mm shift on a
                // 20 mm edge in Page_026's Mirror+refine result),
                // far beyond any reasonable picking tolerance. But
                // FreeCAD's ref midpoint still LIES on the merged
                // curve, so a curve-distance metric stays sub-mm.
                double s = PointToEdgeDistance(ref_pt, e);

                // Tangent disambiguator: when two physically distinct
                // edges happen to share a midpoint (e.g., crossed
                // wires), prefer the one whose tangent agrees with
                // the ref. Weight kept tiny so it only breaks ties
                // among same-distance hits.
                if (ref.normal[0] != 0.0 || ref.normal[1] != 0.0 ||
                    ref.normal[2] != 0.0)
                {
                    double dot = ref.normal[0] * tan.X()
                               + ref.normal[1] * tan.Y()
                               + ref.normal[2] * tan.Z();
                    double n2  = ref.normal[0] * ref.normal[0]
                               + ref.normal[1] * ref.normal[1]
                               + ref.normal[2] * ref.normal[2];
                    if (n2 > 1e-15) dot /= std::sqrt(n2);
                    s += 0.01 * (1.0 - std::fabs(dot));
                }

                // Tie-breaker: prefer edges with similar arc length.
                // Weight stays well below picking tolerance so a
                // length mismatch alone cannot reject a sub-mm match
                // (the common case when FreeCAD and cax disagree on
                // how many co-linear edges to merge after a BOP).
                if (ref.measure > 0.0)
                {
                    double len       = EdgeArcLength(e);
                    double meas_diff = std::fabs(len - ref.measure)
                                     / std::max(len, ref.measure);
                    s += 0.0005 * meas_diff;
                }

                if (s < r.match_dist)
                {
                    r.match_dist = s;
                    r.topo_index = i;
                }
            }
            if (r.topo_index > 0 && r.match_dist <= tolerance)
            {
                const TopoDS_Shape& sub = edgeMap.FindKey(r.topo_index);
                r.uid = LookupUid(naming, ref.kind, sub);
            }
            else
            {
                // Beyond tolerance: treat as unmatched.
                r.topo_index = 0;
            }
            break;
        }

        case TopoRefIR::Kind::Face:
        {
            // Pass 1: rank ALL candidates by the primary score
            // (centroid + normal + area). For dressup face picks
            // the rank-1 candidate is usually the right face, but
            // on symmetric / co-planar bodies several cax faces
            // can share centroid+normal+area exactly -- a Pad with
            // 4 corner pads, a mirrored body, etc. In those cases
            // pass 2 (sample validation) is what picks the right
            // one.
            struct Cand { int idx; double score; };
            std::vector<Cand> cands;
            cands.reserve(faceMap.Extent());
            for (int i = 1; i <= faceMap.Extent(); ++i)
            {
                const TopoDS_Face& f = TopoDS::Face(faceMap.FindKey(i));
                gp_Pnt c;
                gp_Dir n;
                if (!FaceCenter(f, c, n)) {
                    continue;
                }

                double s = MatchScore(ref.point, ref.normal, c, n);

                // Tie-breaker only. Same rationale as the edge case
                // above: FreeCAD's authored shape can have N
                // coplanar faces merged into one (area Nx larger
                // than cax's per-instance face), but the centroid +
                // normal still identify the right physical place.
                // Keep the area term tiny so it ranks among ties
                // without ever rejecting a position-perfect match.
                if (ref.measure > 0.0)
                {
                    double a         = FaceArea(f);
                    double meas_diff = std::fabs(a - ref.measure)
                                     / std::max(a, ref.measure);
                    s += 0.0005 * meas_diff;
                }

                cands.push_back({i, s});
            }

            std::sort(cands.begin(), cands.end(),
                      [](const Cand& a, const Cand& b) {
                          return a.score < b.score;
                      });

            // Pass 2: sample validation. If the ref carries extra
            // sample points (face picks from the FreeCAD reader's
            // face-pick handler attach 3-4 boundary midpoints),
            // walk the candidates in score order and pick the
            // first one whose face contains ALL samples within
            // sample_tol. The samples are physical points on the
            // authored face's boundary -- a cax face that doesn't
            // extend to them isn't the right one even if its
            // centroid lines up.
            //
            // Without samples the legacy behavior is preserved:
            // pick the rank-1 candidate.
            int    best_idx   = 0;
            double best_score = std::numeric_limits<double>::max();

            if (ref.extra_sample_count > 0 && !cands.empty())
            {
                // Cap how many we BRepExtrema-probe; samples are
                // only a tie-breaker among near-tied primary scores.
                const size_t kMaxValidate = 8;
                // Sample-on-surface tolerance: comparable to BOP
                // fuzziness on FreeCAD-saved BReps. Looser than the
                // primary `tolerance` (which gates pos+normal+area
                // score, a different scale).
                const double sample_tol = 1e-3;

                for (size_t k = 0;
                     k < cands.size() && k < kMaxValidate;
                     ++k)
                {
                    const TopoDS_Face& f =
                        TopoDS::Face(faceMap.FindKey(cands[k].idx));
                    bool all_on = true;
                    for (int j = 0; j < ref.extra_sample_count; ++j)
                    {
                        gp_Pnt sp(ref.extra_samples[j][0],
                                   ref.extra_samples[j][1],
                                   ref.extra_samples[j][2]);
                        TopoDS_Vertex sv = BRepBuilderAPI_MakeVertex(sp);
                        BRepExtrema_DistShapeShape ext(sv, f);
                        if (!ext.IsDone() ||
                            ext.NbSolution() == 0 ||
                            ext.Value() > sample_tol)
                        {
                            all_on = false;
                            break;
                        }
                    }
                    if (all_on) {
                        best_idx   = cands[k].idx;
                        best_score = cands[k].score;
                        break;
                    }
                }
                // Fallback: no candidate passed sample validation
                // (e.g., cax body diverged enough that samples
                // don't land). Use the top-scored candidate so we
                // don't regress fixtures that previously resolved
                // without samples.
                if (best_idx == 0) {
                    best_idx   = cands[0].idx;
                    best_score = cands[0].score;
                }
            }
            else if (!cands.empty())
            {
                best_idx   = cands[0].idx;
                best_score = cands[0].score;
            }

            // Point-in-face fallback. The primary score
            // (centroid+normal+area) misses by tens of mm when cax's
            // BOP splits what FreeCAD kept as one face into N
            // sub-faces -- each sub-centroid sits far from the
            // FreeCAD whole-face centroid even though the FreeCAD
            // centroid still LIES on one of cax's sub-faces (seen on
            // Page_037's Pocket.Face1: cax has 16 faces vs FreeCAD's
            // 15, so the resolver missed by tens of mm and the Shell
            // op silently got empty closing faces, degenerating into
            // a 2-shell offset solid). So when the primary best is
            // over tolerance, sweep the candidates in score order
            // and accept the first one whose surface actually
            // contains the ref point. Same point-on-surface check
            // the extra-sample pass uses, just keyed off the ref's
            // own centroid.
            if (best_idx > 0 && best_score > tolerance && !cands.empty())
            {
                const double sample_tol  = 1e-3;
                const size_t kMaxFallback = 16;
                gp_Pnt        rp(ref.point[0], ref.point[1], ref.point[2]);
                TopoDS_Vertex rv = BRepBuilderAPI_MakeVertex(rp);
                for (size_t k = 0;
                     k < cands.size() && k < kMaxFallback;
                     ++k)
                {
                    const TopoDS_Face& f =
                        TopoDS::Face(faceMap.FindKey(cands[k].idx));
                    BRepExtrema_DistShapeShape ext(rv, f);
                    if (ext.IsDone() && ext.NbSolution() > 0 &&
                        ext.Value() <= sample_tol)
                    {
                        best_idx   = cands[k].idx;
                        best_score = ext.Value();  // record real dist
                        break;
                    }
                }
            }

            r.topo_index = best_idx;
            r.match_dist = best_score;
            if (best_idx > 0 && best_score <= tolerance)
            {
                const TopoDS_Shape& sub = faceMap.FindKey(best_idx);
                r.uid = LookupUid(naming, ref.kind, sub);
            }
            else
            {
                r.topo_index = 0;
            }
            break;
        }

        case TopoRefIR::Kind::Vertex:
        {
            for (int i = 1; i <= vertMap.Extent(); ++i)
            {
                const TopoDS_Vertex& v   = TopoDS::Vertex(vertMap.FindKey(i));
                gp_Pnt               p   = BRep_Tool::Pnt(v);
                double               dx  = p.X() - ref.point[0];
                double               dy  = p.Y() - ref.point[1];
                double               dz  = p.Z() - ref.point[2];
                double               pos = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (pos < r.match_dist)
                {
                    r.match_dist = pos;
                    r.topo_index = i;
                }
            }
            if (r.topo_index > 0 && r.match_dist <= tolerance)
            {
                const TopoDS_Shape& sub = vertMap.FindKey(r.topo_index);
                r.uid = LookupUid(naming, ref.kind, sub);
            }
            else
            {
                r.topo_index = 0;
            }
            break;
        }
        }

        out.push_back(r);
    }

    return out;
}

} // namespace cadapp
