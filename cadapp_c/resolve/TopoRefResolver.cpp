#include "cadapp_c/resolve/TopoRefResolver.h"
#include "cadapp_c/resolve/TopoGeomUtils.h"

#include "brepgraph_c/history/TopoNaming.h"
#include "brepgraph_c/history/HistGraph.h"

#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <BRep_Tool.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>

#include <cmath>
#include <limits>

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
            for (int i = 1; i <= edgeMap.Extent(); ++i)
            {
                const TopoDS_Edge& e = TopoDS::Edge(edgeMap.FindKey(i));
                gp_Pnt mid;
                gp_Dir tan;
                if (!EdgeMidpoint(e, mid, tan)) {
                    continue;
                }

                double s = MatchScore(ref.point, ref.normal, mid, tan);

                // Tie-breaker: prefer edges with similar arc length.
                // Weight stays well below any reasonable position
                // tolerance so a length mismatch alone cannot reject
                // a face whose midpoint + tangent already match (the
                // common case when FreeCAD and cax disagree on how
                // many co-linear edges to merge after a BOP).
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

                if (s < r.match_dist)
                {
                    r.match_dist = s;
                    r.topo_index = i;
                }
            }
            if (r.topo_index > 0 && r.match_dist <= tolerance)
            {
                const TopoDS_Shape& sub = faceMap.FindKey(r.topo_index);
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
