#include "cadcvt_c/emitter/Replayer.h"
#include "cadcvt_c/store/SketchBridge.h"
#include "cadcvt_c/resolve/TopoRefResolver.h"

#include "breptopo_c/CompGraph.h"
#include "breptopo_c/TopoNaming.h"
#include "brepdb_c/VersionTree.h"
#include "partgraph_c/TopoShape.h"
#include "partgraph_c/TopoAlgo.h"
#include "partgraph_c/TopoAlgo_Ext.h"
#include "partgraph_c/PrimMaker.h"

#include "partgraph_c/BRepBuilder.h"

#include "sketchlib/Scene.h"

#include <geoshape/Shape2D.h>
#include <geoshape/Point2D.h>
#include <geoshape/Line2D.h>
#include <geoshape/Circle.h>
#include <geoshape/Arc.h>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Circ.hxx>
#include <gp_Elips.hxx>
#include <gp_Trsf.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <ShapeFix_Face.hxx>

#include <cmath>
#include <map>
#include <sstream>
#include <type_traits>
#include <variant>

// ============================================================
// Replayer.cpp
//
// Coverage of the "other-CAD -> cax" direction:
//   Sketch                                       OK
//   PrimBox / PrimCylinder / PrimCone /
//   PrimSphere / PrimTorus                       OK
//   BossExtrude / CutExtrude                     OK
//   Fillet / Chamfer (with TopoRef resolution)   OK
//   Shell (with TopoRef resolution)              OK
//   Mirror                                       OK
//   BossRevolve / CutRevolve                     TODO
//   Loft / Sweep                                 TODO
//   LinearPattern / CircularPattern              TODO
//
// Wire reconstruction: solved 2D shapes are sampled to 32-segment
// polylines and joined head-to-tail into a single polygon. Good
// enough for a closed-loop sanity test; a future pass should use
// BRepBuilderAPI_MakeEdge(Geom_Curve) for exact curves.
// ============================================================

namespace cadcvt
{

namespace
{

// Map a 2D sketch point (x, y) into 3D via (origin, x_dir, normal).
gp_Pnt LocalToWorld(double       x,
                    double       y,
                    const double origin[3],
                    const double x_dir[3],
                    const double normal[3])
{
    // y_dir = normal x x_dir
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

// Build one TopoDS_Edge from a single 2D geometry, lifted onto
// the sketch plane. Returns a null edge if the type isn't an
// edge-producing curve (Point, unhandled types). Using exact
// Geom_Curve based edges (line / circle / arc / ellipse) instead
// of polygon sampling is what lets BRepBuilderAPI_MakeWire match
// endpoints reliably; sampled polylines accumulate fp error and
// the resulting "wire" looks open to OCCT, which makes Prism
// degenerate to a sweep of the wire (uncapped shell).
TopoDS_Edge BuildEdgeFromShape(const gs::Shape2D& shape,
                               const double       origin[3],
                               const double       x_dir[3],
                               const double       normal[3])
{
    switch (shape.GetType())
    {
    case gs::ShapeType2D::Line:
    {
        const auto& s  = static_cast<const gs::Line2D&>(shape);
        gp_Pnt      p1 = LocalToWorld(s.GetStart().x, s.GetStart().y,
                                      origin, x_dir, normal);
        gp_Pnt      p2 = LocalToWorld(s.GetEnd().x,   s.GetEnd().y,
                                      origin, x_dir, normal);
        if (p1.Distance(p2) < 1e-9) {
            return TopoDS_Edge();
        }
        BRepBuilderAPI_MakeEdge mk(p1, p2);
        return mk.IsDone() ? mk.Edge() : TopoDS_Edge();
    }
    case gs::ShapeType2D::Circle:
    {
        const auto& s = static_cast<const gs::Circle&>(shape);
        gp_Pnt      c = LocalToWorld(s.GetCenter().x, s.GetCenter().y,
                                     origin, x_dir, normal);
        gp_Dir      nd(normal[0], normal[1], normal[2]);
        gp_Dir      xd(x_dir [0], x_dir [1], x_dir [2]);
        gp_Ax2      ax(c, nd, xd);
        gp_Circ     circ(ax, s.GetRadius());
        BRepBuilderAPI_MakeEdge mk(circ);
        return mk.IsDone() ? mk.Edge() : TopoDS_Edge();
    }
    case gs::ShapeType2D::Arc:
    {
        const auto& s = static_cast<const gs::Arc&>(shape);
        float       a0 = 0;
        float       a1 = 0;
        s.GetAngles(a0, a1);
        gp_Pnt  c = LocalToWorld(s.GetCenter().x, s.GetCenter().y,
                                 origin, x_dir, normal);
        gp_Dir  nd(normal[0], normal[1], normal[2]);
        gp_Dir  xd(x_dir [0], x_dir [1], x_dir [2]);
        gp_Ax2  ax(c, nd, xd);
        gp_Circ circ(ax, s.GetRadius());
        BRepBuilderAPI_MakeEdge mk(circ, (double)a0, (double)a1);
        return mk.IsDone() ? mk.Edge() : TopoDS_Edge();
    }
    case gs::ShapeType2D::Point:
        // Points don't contribute to wires.
        return TopoDS_Edge();
    default:
        // Ellipse / Spline / others not yet supported; ignored.
        return TopoDS_Edge();
    }
}

// Build a wire from every solved geometry using exact edges.
// BRepBuilderAPI_MakeWire(ListOfShape) reorders edges by endpoint
// matching, so the input order from the solver doesn't matter.
TopoDS_Wire BuildWireFromSolved(const SketchBridge::GeoShapes& solved,
                                const double                   origin[3],
                                const double                   x_dir[3],
                                const double                   normal[3])
{
    TopTools_ListOfShape edges;
    for (const auto& kv : solved)
    {
        TopoDS_Edge e = BuildEdgeFromShape(*kv.second, origin, x_dir, normal);
        if (!e.IsNull()) {
            edges.Append(e);
        }
    }
    if (edges.IsEmpty()) {
        return TopoDS_Wire();
    }

    BRepBuilderAPI_MakeWire mk;
    mk.Add(edges);
    if (!mk.IsDone()) {
        return TopoDS_Wire();
    }
    return mk.Wire();
}

std::shared_ptr<partgraph::TopoShape> WireToFace(const TopoDS_Wire& wire,
                                                  const double      origin[3],
                                                  const double      normal[3])
{
    if (wire.IsNull()) {
        return {};
    }

    gp_Pnt o(origin[0], origin[1], origin[2]);
    gp_Dir n(normal[0], normal[1], normal[2]);
    gp_Pln plane(o, n);

    // MakeFace(plane, wire) defaults to Inside=true, which already
    // auto-reverses CW wires so the face ends up bounded.
    //
    // Important: do NOT call ShapeFix_Face::FixAddNaturalBound on
    // the result. For a plane the "natural bound" is an infinite
    // box; the fixer would treat our wire as an inner hole and
    // turn the face into (infinite slab - our disc), which Prism
    // then sweeps into an annular slab with the inner top cap
    // missing - exactly the "one side not capped" symptom we saw.
    BRepBuilderAPI_MakeFace mkFace(plane, wire);
    if (!mkFace.IsDone()) {
        return {};
    }

    // FixOrientation is fine on its own: it just flips the wire
    // when its signed area is negative relative to the plane
    // normal, so the resulting face's natural normal lines up with
    // the plane's normal. Without this the prism's side faces or
    // top cap may end up with inward normals -> backface culling
    // hides them in the renderer, looking like a missing cap.
    TopoDS_Face   face = mkFace.Face();
    ShapeFix_Face fixer(face);
    fixer.FixOrientation();
    fixer.Perform();
    face = fixer.Face();

    return std::make_shared<partgraph::TopoShape>(face);
}

// Locate the SketchIR whose feature_id matches sketch_feat_id.
const SketchIR* FindSketch(const DocumentIR& doc, uint32_t sketch_feat_id)
{
    if (sketch_feat_id == 0xFFFFFFFF) {
        return nullptr;
    }
    for (const auto& sk : doc.sketches)
    {
        if (sk.feature_id == sketch_feat_id) {
            return &sk;
        }
    }
    return nullptr;
}

} // anonymous namespace

// ============================================================
// Impl
// ============================================================

struct Replayer::Impl
{
    std::shared_ptr<breptopo::TopoNaming> naming;
    std::shared_ptr<brepdb::VersionTree>  vtree;
};

Replayer::Replayer()
    : m_impl(std::make_unique<Impl>())
{
}

Replayer::~Replayer() = default;

void Replayer::SetNaming(const std::shared_ptr<breptopo::TopoNaming>& tn)
{
    m_impl->naming = tn;
}

void Replayer::SetVersionTree(const std::shared_ptr<brepdb::VersionTree>& vt)
{
    m_impl->vtree = vt;
}

bool Replayer::Replay(DocumentIR& doc, const ReplayOptions& opt, ReplayResult& out)
{
    out = ReplayResult{};

    if (!m_impl->naming) {
        m_impl->naming = std::make_shared<breptopo::TopoNaming>();
    }
    if (!m_impl->vtree) {
        m_impl->vtree = std::make_shared<brepdb::VersionTree>();
    }

    out.naming = m_impl->naming;
    out.vtree  = m_impl->vtree;

    auto cg = std::make_shared<breptopo::CompGraph>();
    cg->SetTopoNaming(m_impl->naming);

    int last_node = -1;

    for (size_t i = 0; i < doc.features.size(); ++i)
    {
        auto& feat = doc.features[i];
        if (feat.suppressed)
        {
            out.op_ids.push_back(0);
            continue;
        }

        int  node    = -1;
        bool step_ok = true;

        std::visit([&](auto& p)
        {
            using T = std::decay_t<decltype(p)>;

            // ---- Sketch ----
            if constexpr (std::is_same_v<T, FeatPayloadSketch>)
            {
                (void)p;
            }

            // ---- Primitives ----
            else if constexpr (std::is_same_v<T, FeatPayloadPrimBox>)
            {
                int l = cg->AddConst(p.length, "length");
                int w = cg->AddConst(p.width,  "width");
                int h = cg->AddConst(p.height, "height");
                node = cg->AddOp("box", {l, w, h}, {}, feat.name);
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimCylinder>)
            {
                int r = cg->AddConst(p.radius, "radius");
                int h = cg->AddConst(p.height, "height");
                node = cg->AddOp("cylinder", {r, h}, {}, feat.name);
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimCone>)
            {
                int r1 = cg->AddConst(p.radius1, "radius1");
                int r2 = cg->AddConst(p.radius2, "radius2");
                int h  = cg->AddConst(p.height,  "height");
                node = cg->AddOp("cone", {r1, r2, h}, {}, feat.name);
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimSphere>)
            {
                int r = cg->AddConst(p.radius, "radius");
                node = cg->AddOp("sphere", {r}, {}, feat.name);
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimTorus>)
            {
                int r1 = cg->AddConst(p.major_radius, "major_radius");
                int r2 = cg->AddConst(p.minor_radius, "minor_radius");
                node = cg->AddOp("torus", {r1, r2}, {}, feat.name);
            }

            // ---- Extrude (Boss / Cut) ----
            else if constexpr (std::is_same_v<T, FeatPayloadExtrude>)
            {
                const SketchIR* sk = FindSketch(doc, p.sketch_id);
                if (!sk)
                {
                    step_ok     = false;
                    out.err_msg = "missing sketch for feature " + feat.name;
                    return;
                }

                sketchlib::Scene        scene;
                SketchBridge::GeoShapes solved;
                if (!SketchBridge::ImportToScene(*sk, scene, solved))
                {
                    step_ok     = false;
                    out.err_msg = "ImportToScene failed: " + feat.name;
                    return;
                }
                scene.Solve(solved);

                TopoDS_Wire wire = BuildWireFromSolved(
                    solved,
                    sk->plane_origin,
                    sk->plane_x_dir,
                    sk->plane_normal);
                if (wire.IsNull())
                {
                    step_ok     = false;
                    out.err_msg = "wire build failed: " + feat.name;
                    return;
                }
                auto face = WireToFace(wire, sk->plane_origin, sk->plane_normal);
                if (!face)
                {
                    step_ok     = false;
                    out.err_msg = "wire->face failed: " + feat.name;
                    return;
                }

                int face_n = cg->AddConst(face, "profile");

                double yd[3] =
                {
                    sk->plane_normal[1] * sk->plane_x_dir[2] - sk->plane_normal[2] * sk->plane_x_dir[1],
                    sk->plane_normal[2] * sk->plane_x_dir[0] - sk->plane_normal[0] * sk->plane_x_dir[2],
                    sk->plane_normal[0] * sk->plane_x_dir[1] - sk->plane_normal[1] * sk->plane_x_dir[0],
                };
                double world_dir[3] =
                {
                    p.direction[0] * sk->plane_x_dir[0] + p.direction[1] * yd[0] + p.direction[2] * sk->plane_normal[0],
                    p.direction[0] * sk->plane_x_dir[1] + p.direction[1] * yd[1] + p.direction[2] * sk->plane_normal[1],
                    p.direction[0] * sk->plane_x_dir[2] + p.direction[1] * yd[2] + p.direction[2] * sk->plane_normal[2],
                };

                double sign = p.flip_direction ? -1.0 : 1.0;

                int tool_n;
                if (p.end_type != ExtrudeEndType::Blind)
                {
                    breptopo::Vec3 dir = {sign * world_dir[0],
                                          sign * world_dir[1],
                                          sign * world_dir[2]};
                    int dir_n = cg->AddConst(dir, "direction");
                    int d1_n  = cg->AddConst(p.distance,  "dist1");
                    int d2_n  = cg->AddConst(p.distance2, "dist2");
                    int e1_n  = cg->AddConst((int)p.end_type,  "end1");
                    int e2_n  = cg->AddConst((int)p.end_type2, "end2");
                    int ref_n = last_node >= 0
                        ? last_node
                        : cg->AddConst(std::shared_ptr<partgraph::TopoShape>{}, "null_ref");
                    tool_n = cg->AddOp("extrude_ex",
                        {face_n, dir_n, d1_n, d2_n, e1_n, e2_n, ref_n},
                        {}, feat.name);
                }
                else
                {
                    double dx = world_dir[0] * p.distance * sign;
                    double dy = world_dir[1] * p.distance * sign;
                    double dz = world_dir[2] * p.distance * sign;
                    breptopo::Vec3 dir = {dx, dy, dz};
                    int dir_n = cg->AddConst(dir, "direction");
                    tool_n = cg->AddOp("prism", {face_n, dir_n}, {}, feat.name);
                }

                if (feat.type == FeatType::BossExtrude)
                {
                    if (last_node < 0)
                    {
                        node = tool_n;
                    }
                    else
                    {
                        node = cg->AddOp("fuse", {last_node, tool_n},
                                          {}, feat.name);
                    }
                }
                else
                {
                    if (last_node < 0)
                    {
                        step_ok     = false;
                        out.err_msg = "CutExtrude without base shape: " + feat.name;
                        return;
                    }
                    node = cg->AddOp("cut", {last_node, tool_n},
                                      {}, feat.name);
                }
            }

            // ---- Fillet / Chamfer ----
            else if constexpr (std::is_same_v<T, FeatPayloadFillet> ||
                               std::is_same_v<T, FeatPayloadChamfer>)
            {
                if (last_node < 0)
                {
                    step_ok = false;
                    return;
                }

                // Phase-1: materialize previous shape for geometric
                // resolution. Phase-2 replaces this with a graph-level
                // resolved_edge_ref / resolved_face_ref op.
                auto prev_val = cg->Eval(last_node);
                auto* sv = std::get_if<breptopo::ShapeVal>(&prev_val);
                if (!sv || !sv->shape)
                {
                    step_ok = false;
                    return;
                }

                auto resolved = TopoRefResolver::Resolve(
                    sv->shape->GetShape(),
                    p.edges,
                    m_impl->naming.get(),
                    opt.topo_tolerance);

                std::vector<int> edge_nodes;

                TopTools_IndexedMapOfShape em;
                TopExp::MapShapes(sv->shape->GetShape(), TopAbs_EDGE, em);

                for (size_t k = 0; k < resolved.size(); ++k)
                {
                    if (resolved[k].topo_index <= 0 ||
                        resolved[k].topo_index > em.Extent())
                    {
                        continue;
                    }
                    auto edge_shape = std::make_shared<partgraph::TopoShape>(
                        em.FindKey(resolved[k].topo_index));
                    edge_nodes.push_back(cg->AddConst(edge_shape, "edge"));

                    if (opt.write_back_resolved && k < p.edges.size())
                    {
                        p.edges[k].resolved_uid        = resolved[k].uid;
                        p.edges[k].resolved_topo_index = resolved[k].topo_index;
                    }
                }

                if (edge_nodes.empty())
                {
                    step_ok     = false;
                    out.err_msg = "no edges resolved for: " + feat.name;
                    return;
                }

                if constexpr (std::is_same_v<T, FeatPayloadFillet>)
                {
                    int r = cg->AddConst(p.radius, "radius");
                    node = cg->AddOp("fillet", {last_node, r},
                                      edge_nodes, feat.name);
                }
                else
                {
                    int d = cg->AddConst(p.distance1, "dist");
                    node = cg->AddOp("chamfer", {last_node, d},
                                      edge_nodes, feat.name);
                }
            }

            // ---- Shell ----
            else if constexpr (std::is_same_v<T, FeatPayloadShell>)
            {
                if (last_node < 0)
                {
                    step_ok = false;
                    return;
                }

                auto prev_val = cg->Eval(last_node);
                auto* sv = std::get_if<breptopo::ShapeVal>(&prev_val);
                if (!sv || !sv->shape)
                {
                    step_ok = false;
                    return;
                }

                auto resolved = TopoRefResolver::Resolve(
                    sv->shape->GetShape(),
                    p.faces_to_open,
                    m_impl->naming.get(),
                    opt.topo_tolerance);

                std::vector<int> face_nodes;
                TopTools_IndexedMapOfShape fm;
                TopExp::MapShapes(sv->shape->GetShape(), TopAbs_FACE, fm);

                for (size_t k = 0; k < resolved.size(); ++k)
                {
                    if (resolved[k].topo_index <= 0 ||
                        resolved[k].topo_index > fm.Extent())
                    {
                        continue;
                    }
                    auto face_shape = std::make_shared<partgraph::TopoShape>(
                        fm.FindKey(resolved[k].topo_index));
                    face_nodes.push_back(cg->AddConst(face_shape, "face"));

                    if (opt.write_back_resolved && k < p.faces_to_open.size())
                    {
                        p.faces_to_open[k].resolved_uid        = resolved[k].uid;
                        p.faces_to_open[k].resolved_topo_index = resolved[k].topo_index;
                    }
                }

                int t = cg->AddConst(p.thickness, "thickness");
                node = cg->AddOp("shell", {last_node, t},
                                  face_nodes, feat.name);
            }

            // ---- Mirror ----
            else if constexpr (std::is_same_v<T, FeatPayloadMirror>)
            {
                if (last_node < 0)
                {
                    step_ok = false;
                    return;
                }
                breptopo::Vec3 origin = {p.plane_origin[0],
                                         p.plane_origin[1],
                                         p.plane_origin[2]};
                breptopo::Vec3 normal = {p.plane_normal[0],
                                         p.plane_normal[1],
                                         p.plane_normal[2]};
                int o = cg->AddConst(origin, "origin");
                int n = cg->AddConst(normal, "normal");
                node = cg->AddOp("mirror", {last_node, o, n},
                                  {}, feat.name);
            }

            // ---- Not implemented yet ----
            else
            {
                std::ostringstream oss;
                oss << "skipped unimplemented feature: " << feat.name
                    << " (type=" << (int)feat.type << ")";
                if (!out.err_msg.empty()) {
                    out.err_msg += "; ";
                }
                out.err_msg += oss.str();
            }
        }, feat.data);

        if (node >= 0)
        {
            last_node = node;
            out.op_ids.push_back(cg->CalcOpId(node, 0));
        }
        else
        {
            out.op_ids.push_back(0);
        }

        if (!step_ok && last_node < 0)
        {
            out.ok    = false;
            out.shape = nullptr;
            return false;
        }
    }

    if (last_node >= 0)
    {
        auto val = cg->Eval(last_node);
        if (auto* sv = std::get_if<breptopo::ShapeVal>(&val)) {
            out.shape = sv->shape;
        }
    }

    out.comp_graph = cg;
    out.ok         = true;
    return true;
}

} // namespace cadcvt
