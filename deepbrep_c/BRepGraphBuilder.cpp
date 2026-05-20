#include "BRepGraphBuilder.h"
#include "FeatureLabels.h"

#include "brepkit_c/TopoShape.h"

// OCCT
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopAbs.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <set>
#include <utility>

namespace deepbrep
{

namespace
{

int surface_type_index(const TopoDS_Face& face)
{
    BRepAdaptor_Surface adaptor(face);
    switch (adaptor.GetType()) {
    case GeomAbs_Plane:        return static_cast<int>(SurfaceType::Plane);
    case GeomAbs_Cylinder:     return static_cast<int>(SurfaceType::Cylinder);
    case GeomAbs_Cone:         return static_cast<int>(SurfaceType::Cone);
    case GeomAbs_Sphere:       return static_cast<int>(SurfaceType::Sphere);
    case GeomAbs_Torus:        return static_cast<int>(SurfaceType::Torus);
    case GeomAbs_BSplineSurface:
    case GeomAbs_BezierSurface:
    case GeomAbs_SurfaceOfRevolution:
    case GeomAbs_SurfaceOfExtrusion:
    case GeomAbs_OffsetSurface:
    case GeomAbs_OtherSurface:
    default:                   return static_cast<int>(SurfaceType::BSpline);
    }
}

int curve_type_index(const TopoDS_Edge& edge)
{
    if (BRep_Tool::Degenerated(edge)) {
        return static_cast<int>(CurveType::Other);
    }
    BRepAdaptor_Curve adaptor(edge);
    switch (adaptor.GetType()) {
    case GeomAbs_Line:           return static_cast<int>(CurveType::Line);
    case GeomAbs_Circle:         return static_cast<int>(CurveType::Circle);
    case GeomAbs_Ellipse:        return static_cast<int>(CurveType::Ellipse);
    case GeomAbs_BSplineCurve:
    case GeomAbs_BezierCurve:    return static_cast<int>(CurveType::BSpline);
    default:                     return static_cast<int>(CurveType::Other);
    }
}

// Returns a unit normal of `face` at parameter (u, v). Falls back to (0,0,1)
// if the adaptor cannot evaluate.
gp_Dir face_normal_at(const TopoDS_Face& face, double u, double v)
{
    BRepAdaptor_Surface adaptor(face);
    gp_Pnt p;
    gp_Vec du, dv;
    adaptor.D1(u, v, p, du, dv);
    gp_Vec n = du.Crossed(dv);
    if (n.Magnitude() < 1e-12) {
        return gp_Dir(0, 0, 1);
    }
    gp_Dir d(n);
    if (face.Orientation() == TopAbs_REVERSED) {
        d.Reverse();
    }
    return d;
}

double face_area(const TopoDS_Face& face)
{
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    return props.Mass();
}

double edge_length(const TopoDS_Edge& edge)
{
    GProp_GProps props;
    BRepGProp::LinearProperties(edge, props);
    return props.Mass();
}

// Classify convexity of an edge shared by two faces. Returns one of:
//   Convex / Concave / Smooth.
//
// Approach: pick the edge midpoint, evaluate each face's surface normal at
// that point's (u,v) on the respective face. The dot product of the two
// outward normals plus the geometric orientation tells convex vs concave.
// This is a heuristic -- good enough for feature-recognition seed features.
int convexity_at_edge(const TopoDS_Edge& edge,
                      const TopoDS_Face& face_a,
                      const TopoDS_Face& face_b,
                      double& out_dihedral_normalized)
{
    out_dihedral_normalized = 0.5f;

    if (BRep_Tool::Degenerated(edge)) {
        return static_cast<int>(Convexity::Smooth);
    }

    Standard_Real first = 0.0, last = 0.0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
    if (curve.IsNull()) {
        return static_cast<int>(Convexity::Smooth);
    }
    const double t = 0.5 * (first + last);
    gp_Pnt edge_pt = curve->Value(t);

    // Project the point onto each face's parametric domain. For robustness we
    // use the pcurve when available; otherwise fall back to the face center.
    auto face_normal_at_pt = [](const TopoDS_Face& face,
                                const TopoDS_Edge& e,
                                const gp_Pnt& /*pt*/,
                                double t) -> gp_Dir {
        Standard_Real f = 0, l = 0;
        Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(e, face, f, l);
        if (!pcurve.IsNull()) {
            const double tt = std::clamp(t, f, l);
            gp_Pnt2d uv = pcurve->Value(tt);
            return face_normal_at(face, uv.X(), uv.Y());
        }
        BRepAdaptor_Surface s(face);
        return face_normal_at(face,
                              0.5 * (s.FirstUParameter() + s.LastUParameter()),
                              0.5 * (s.FirstVParameter() + s.LastVParameter()));
    };

    gp_Dir na = face_normal_at_pt(face_a, edge, edge_pt, t);
    gp_Dir nb = face_normal_at_pt(face_b, edge, edge_pt, t);

    const double dot = na.Dot(nb);
    const double clamped = std::clamp(dot, -1.0, 1.0);
    // Dihedral between outward normals: 0 = aligned, pi = opposite.
    const double angle = std::acos(clamped);
    out_dihedral_normalized = static_cast<float>(angle / 3.14159265358979323846);

    // Cross of normals points along the edge tangent for one of convex /
    // concave; sign vs the curve tangent decides.
    gp_Vec edge_tan(0, 0, 0);
    BRepAdaptor_Curve cadaptor(edge);
    cadaptor.D1(t, edge_pt, edge_tan);
    if (edge_tan.Magnitude() < 1e-12) {
        return static_cast<int>(Convexity::Smooth);
    }
    gp_Vec cross(gp_Vec(na.X(), na.Y(), na.Z()).Crossed(gp_Vec(nb.X(), nb.Y(), nb.Z())));
    if (cross.Magnitude() < 1e-6) {
        // Normals (anti-)parallel -- tangent plane continuous.
        return static_cast<int>(Convexity::Smooth);
    }
    const double sign = cross.Dot(edge_tan);
    return sign > 0 ? static_cast<int>(Convexity::Convex)
                    : static_cast<int>(Convexity::Concave);
}

void fill_node_features(const TopoDS_Face& face, float* row)
{
    std::memset(row, 0, kNodeFeatDim * sizeof(float));

    const int st = surface_type_index(face);
    row[st] = 1.0f;                                    // surface_type one-hot

    const double area = face_area(face);
    // Area normalized by a coarse scale. The exact normalization is not
    // critical -- the encoder learns to scale.
    row[6] = static_cast<float>(area / (1.0 + area));

    BRepAdaptor_Surface s(face);
    const double um = 0.5 * (s.FirstUParameter() + s.LastUParameter());
    const double vm = 0.5 * (s.FirstVParameter() + s.LastVParameter());
    gp_Dir n = face_normal_at(face, um, vm);
    row[7] = static_cast<float>(n.X());
    row[8] = static_cast<float>(n.Y());
    row[9] = static_cast<float>(n.Z());

    // Wire-edge count. Outer wire is conventionally the first.
    int outer_edges = 0;
    int inner_wires = 0;
    int wire_idx = 0;
    for (TopExp_Explorer wexp(face, TopAbs_WIRE); wexp.More(); wexp.Next(), ++wire_idx) {
        if (wire_idx == 0) {
            for (TopExp_Explorer eexp(wexp.Current(), TopAbs_EDGE); eexp.More(); eexp.Next()) {
                ++outer_edges;
            }
        } else {
            ++inner_wires;
        }
    }
    row[10] = static_cast<float>(outer_edges) / 10.0f;
    row[11] = static_cast<float>(inner_wires);

    // Curvature stats are left at zero -- caller can post-process if needed.
    row[12] = 0.0f;
    row[13] = 0.0f;
}

void fill_edge_features(const TopoDS_Edge& edge,
                        const TopoDS_Face& fa, const TopoDS_Face& fb,
                        float* row)
{
    std::memset(row, 0, kEdgeFeatDim * sizeof(float));

    const int ct = curve_type_index(edge);
    row[ct] = 1.0f;

    double dihedral = 0.5;
    const int cv = convexity_at_edge(edge, fa, fb, dihedral);
    row[5 + cv] = 1.0f;
    row[8] = static_cast<float>(dihedral);

    const double len = edge_length(edge);
    row[9] = static_cast<float>(len / (1.0 + len));
}

}  // anonymous namespace

// ============================================================
// Public API
// ============================================================

void BRepGraphBuilder::AppendShape(
    const std::shared_ptr<brepkit::TopoShape>& topo_shape,
    GraphData& g,
    std::vector<std::pair<int, int>>& from_to,
    std::vector<std::vector<float>>& edge_rows)
{
    auto& shape = topo_shape->GetShape();

    TopTools_IndexedMapOfShape all_faces;
    TopExp::MapShapes(shape, TopAbs_FACE, all_faces);
    const int face_count = all_faces.Extent();
    const int node_offset = g.num_nodes;

    // Resize / extend node feature matrix.
    Mat new_nodes(node_offset + face_count, kNodeFeatDim);
    for (int i = 0; i < node_offset; ++i) {
        std::memcpy(new_nodes.row_ptr(i),
                    g.node_features.row_ptr(i),
                    kNodeFeatDim * sizeof(float));
    }
    for (int i = 1; i <= face_count; ++i) {
        const TopoDS_Face& f = TopoDS::Face(all_faces.FindKey(i));
        fill_node_features(f, new_nodes.row_ptr(node_offset + i - 1));
    }
    g.node_features = std::move(new_nodes);
    g.num_nodes = node_offset + face_count;

    // Walk shared edges to produce face-face adjacency.
    TopTools_IndexedDataMapOfShapeListOfShape edge_face_map;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_face_map);

    std::set<std::pair<int, int>> seen_undirected;
    for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());
        if (!edge_face_map.Contains(edge)) {
            continue;
        }
        const TopTools_ListOfShape& faces = edge_face_map.FindFromKey(edge);

        std::vector<int> face_ids;
        face_ids.reserve(faces.Extent());
        for (TopTools_ListIteratorOfListOfShape it(faces); it.More(); it.Next()) {
            const int idx = all_faces.FindIndex(it.Value());
            if (idx > 0) {
                face_ids.push_back(idx - 1);
            }
        }

        for (size_t a = 0; a < face_ids.size(); ++a) {
            for (size_t b = a + 1; b < face_ids.size(); ++b) {
                const int u = face_ids[a];
                const int v = face_ids[b];
                auto key = std::pair<int, int>(std::min(u, v), std::max(u, v));
                if (!seen_undirected.insert(key).second) {
                    continue;
                }

                const TopoDS_Face& fa = TopoDS::Face(all_faces.FindKey(u + 1));
                const TopoDS_Face& fb = TopoDS::Face(all_faces.FindKey(v + 1));

                std::vector<float> row(kEdgeFeatDim, 0.0f);
                fill_edge_features(edge, fa, fb, row.data());

                from_to.emplace_back(node_offset + u, node_offset + v);
                edge_rows.push_back(row);
                from_to.emplace_back(node_offset + v, node_offset + u);
                edge_rows.push_back(row);
            }
        }
    }
}

GraphData BRepGraphBuilder::Build(
    const std::vector<std::shared_ptr<brepkit::TopoShape>>& shapes)
{
    GraphData g;
    std::vector<std::pair<int, int>> from_to;
    std::vector<std::vector<float>> edge_rows;

    for (auto& s : shapes) {
        if (!s) continue;
        AppendShape(s, g, from_to, edge_rows);
    }

    // Sort edges by source so CSR fills cleanly.
    const int ne = static_cast<int>(from_to.size());
    std::vector<int> perm(ne);
    for (int i = 0; i < ne; ++i) perm[i] = i;
    std::sort(perm.begin(), perm.end(), [&](int a, int b) {
        return from_to[a].first < from_to[b].first;
    });

    g.edge_features = Mat(ne, kEdgeFeatDim);
    std::vector<std::pair<int, int>> ft_sorted(ne);
    for (int i = 0; i < ne; ++i) {
        ft_sorted[i] = from_to[perm[i]];
        std::memcpy(g.edge_features.row_ptr(i),
                    edge_rows[perm[i]].data(),
                    kEdgeFeatDim * sizeof(float));
    }
    build_csr_from_directed_edges(g, ft_sorted);
    return g;
}

GraphData BRepGraphBuilder::Build(const std::shared_ptr<brepkit::TopoShape>& shape)
{
    if (!shape) return GraphData{};
    return Build(std::vector<std::shared_ptr<brepkit::TopoShape>>{shape});
}

}
