#include "cadcvt_c/emitter/Replayer.h"
#include "cadcvt_c/store/SketchBridge.h"
#include "cadcvt_c/resolve/TopoRefResolver.h"

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

    std::shared_ptr<partgraph::TopoShape> shape;

    for (size_t i = 0; i < doc.features.size(); ++i)
    {
        auto& feat = doc.features[i];
        if (feat.suppressed)
        {
            out.op_ids.push_back(0);
            continue;
        }

        uint32_t op_id   = m_impl->naming->NextOpId();
        bool     step_ok = true;

        // Dispatch by payload variant. Adding a new payload kind
        // here is a compile error until you handle it - that is
        // the whole point of using std::visit.
        std::visit([&](auto& p)
        {
            using T = std::decay_t<decltype(p)>;

            // ---- Sketch ----
            if constexpr (std::is_same_v<T, FeatPayloadSketch>)
            {
                // Sketches do not produce 3D directly; the next
                // sketch-based feature consumes them.
                (void)p;
            }

            // ---- Primitives ----
            else if constexpr (std::is_same_v<T, FeatPayloadPrimBox>)
            {
                auto s = partgraph::PrimMaker::Box(
                    p.length, p.width, p.height,
                    op_id, m_impl->naming, m_impl->vtree);
                if (!s)
                {
                    step_ok = false;
                    return;
                }
                shape = s;
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimCylinder>)
            {
                auto s = partgraph::PrimMaker::Cylinder(
                    p.radius, p.height,
                    op_id, m_impl->naming, m_impl->vtree);
                if (!s)
                {
                    step_ok = false;
                    return;
                }
                shape = s;
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimCone>)
            {
                auto s = partgraph::PrimMaker::Cone(
                    p.radius1, p.radius2, p.height,
                    op_id, m_impl->naming, m_impl->vtree);
                if (!s)
                {
                    step_ok = false;
                    return;
                }
                shape = s;
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimSphere>)
            {
                auto s = partgraph::PrimMaker::Sphere(
                    p.radius,
                    op_id, m_impl->naming, m_impl->vtree);
                if (!s)
                {
                    step_ok = false;
                    return;
                }
                shape = s;
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimTorus>)
            {
                auto s = partgraph::PrimMaker::Torus(
                    p.major_radius, p.minor_radius,
                    op_id, m_impl->naming, m_impl->vtree);
                if (!s)
                {
                    step_ok = false;
                    return;
                }
                shape = s;
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

                // direction is expressed in sketch-local axes
                // (x = plane_x_dir, y = plane_y_dir, z = plane_normal).
                // Readers conventionally write (0,0,1) to mean
                // "along the sketch normal"; we rotate that into
                // world space so the prism direction lines up with
                // the face we just built.
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

                std::shared_ptr<partgraph::TopoShape> tool;
                if (p.end_type != ExtrudeEndType::Blind)
                {
                    tool = partgraph::TopoAlgo_Ext::ExtrudeEx(
                        face,
                        sign * world_dir[0],
                        sign * world_dir[1],
                        sign * world_dir[2],
                        p.distance,
                        p.distance2,
                        static_cast<partgraph::ExtrudeEndType>(p.end_type),
                        static_cast<partgraph::ExtrudeEndType>(p.end_type2),
                        shape,
                        op_id, m_impl->naming);
                }
                else
                {
                    double dx = world_dir[0] * p.distance * sign;
                    double dy = world_dir[1] * p.distance * sign;
                    double dz = world_dir[2] * p.distance * sign;
                    tool = partgraph::TopoAlgo::Prism(
                        face,
                        dx, dy, dz,
                        op_id, m_impl->naming, m_impl->vtree);
                }
                if (!tool)
                {
                    step_ok = false;
                    return;
                }

                if (feat.type == FeatType::BossExtrude)
                {
                    if (!shape)
                    {
                        shape = tool;
                    }
                    else
                    {
                        shape = partgraph::TopoAlgo::Fuse(
                            shape, tool,
                            op_id, m_impl->naming, m_impl->vtree);
                    }
                }
                else
                {
                    // CutExtrude requires an existing base.
                    if (!shape)
                    {
                        step_ok     = false;
                        out.err_msg = "CutExtrude without base shape: " + feat.name;
                        return;
                    }
                    shape = partgraph::TopoAlgo::Cut(
                        shape, tool,
                        op_id, m_impl->naming, m_impl->vtree);
                }
            }

            // ---- Fillet / Chamfer ----
            else if constexpr (std::is_same_v<T, FeatPayloadFillet> ||
                               std::is_same_v<T, FeatPayloadChamfer>)
            {
                if (!shape)
                {
                    step_ok = false;
                    return;
                }

                auto resolved = TopoRefResolver::Resolve(
                    shape->GetShape(),
                    p.edges,
                    m_impl->naming.get(),
                    opt.topo_tolerance);

                std::vector<std::shared_ptr<partgraph::TopoShape>> edge_shapes;
                edge_shapes.reserve(resolved.size());

                TopTools_IndexedMapOfShape em;
                TopExp::MapShapes(shape->GetShape(), TopAbs_EDGE, em);

                for (size_t k = 0; k < resolved.size(); ++k)
                {
                    if (resolved[k].topo_index <= 0 ||
                        resolved[k].topo_index > em.Extent())
                    {
                        continue;
                    }
                    const TopoDS_Shape& e = em.FindKey(resolved[k].topo_index);
                    edge_shapes.push_back(std::make_shared<partgraph::TopoShape>(e));

                    if (opt.write_back_resolved && k < p.edges.size())
                    {
                        p.edges[k].resolved_uid        = resolved[k].uid;
                        p.edges[k].resolved_topo_index = resolved[k].topo_index;
                    }
                }

                if (edge_shapes.empty())
                {
                    step_ok     = false;
                    out.err_msg = "no edges resolved for: " + feat.name;
                    return;
                }

                if constexpr (std::is_same_v<T, FeatPayloadFillet>)
                {
                    shape = partgraph::TopoAlgo::Fillet(
                        shape, p.radius, edge_shapes,
                        op_id, m_impl->naming, m_impl->vtree);
                }
                else
                {
                    shape = partgraph::TopoAlgo::Chamfer(
                        shape, p.distance1, edge_shapes,
                        op_id, m_impl->naming, m_impl->vtree);
                }
            }

            // ---- Shell ----
            else if constexpr (std::is_same_v<T, FeatPayloadShell>)
            {
                if (!shape)
                {
                    step_ok = false;
                    return;
                }

                auto resolved = TopoRefResolver::Resolve(
                    shape->GetShape(),
                    p.faces_to_open,
                    m_impl->naming.get(),
                    opt.topo_tolerance);

                std::vector<std::shared_ptr<partgraph::TopoShape>> face_shapes;
                TopTools_IndexedMapOfShape fm;
                TopExp::MapShapes(shape->GetShape(), TopAbs_FACE, fm);

                for (size_t k = 0; k < resolved.size(); ++k)
                {
                    if (resolved[k].topo_index <= 0 ||
                        resolved[k].topo_index > fm.Extent())
                    {
                        continue;
                    }
                    const TopoDS_Shape& f = fm.FindKey(resolved[k].topo_index);
                    face_shapes.push_back(std::make_shared<partgraph::TopoShape>(f));

                    if (opt.write_back_resolved && k < p.faces_to_open.size())
                    {
                        p.faces_to_open[k].resolved_uid        = resolved[k].uid;
                        p.faces_to_open[k].resolved_topo_index = resolved[k].topo_index;
                    }
                }

                shape = partgraph::TopoAlgo::ThickSolid(
                    shape, face_shapes, (float)p.thickness,
                    op_id, m_impl->naming, m_impl->vtree);
            }

            // ---- Mirror ----
            else if constexpr (std::is_same_v<T, FeatPayloadMirror>)
            {
                if (!shape)
                {
                    step_ok = false;
                    return;
                }
                shape = partgraph::TopoAlgo::Mirror(
                    shape,
                    sm::vec3((float)p.plane_origin[0],
                             (float)p.plane_origin[1],
                             (float)p.plane_origin[2]),
                    sm::vec3((float)p.plane_normal[0],
                             (float)p.plane_normal[1],
                             (float)p.plane_normal[2]),
                    op_id, m_impl->naming, m_impl->vtree);
            }

            // ---- Not implemented yet ----
            else
            {
                // Revolve / Loft / Sweep / Draft / Offset /
                // Transform / Patterns / Boolean / HoleWizard /
                // Rib / Opaque all fall here.
                std::ostringstream oss;
                oss << "skipped unimplemented feature: " << feat.name
                    << " (type=" << (int)feat.type << ")";
                if (!out.err_msg.empty()) {
                    out.err_msg += "; ";
                }
                out.err_msg += oss.str();
            }
        }, feat.data);

        out.op_ids.push_back(step_ok ? op_id : 0);

        if (!step_ok && shape == nullptr)
        {
            // Failed on the very first feature; bail.
            out.ok    = false;
            out.shape = nullptr;
            return false;
        }
    }

    out.ok    = true;
    out.shape = shape;
    return true;
}

} // namespace cadcvt
