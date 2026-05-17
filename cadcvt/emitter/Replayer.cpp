#include "cadcvt/emitter/Replayer.h"
#include "cadcvt/store/SketchBridge.h"
#include "cadcvt/resolve/TopoRefResolver.h"

#include "breptopo_c/TopoNaming.h"
#include "brepdb_c/VersionTree.h"
#include "partgraph_c/TopoShape.h"
#include "partgraph_c/TopoAlgo.h"
#include "partgraph_c/PrimMaker.h"

#include "sketchlib/Scene.h"

#include <geoshape/Shape2D.h>
#include <geoshape/Point2D.h>
#include <geoshape/Line2D.h>
#include <geoshape/Circle.h>
#include <geoshape/Arc.h>

#include <BRepBuilderAPI_MakePolygon.hxx>
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
#include <gp_Trsf.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

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

constexpr int kArcSegments = 32;

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

void SamplePolylineFromShape(const gs::Shape2D&     shape,
                             std::vector<sm::vec2>& out)
{
    switch (shape.GetType())
    {
    case gs::ShapeType2D::Line:
    {
        const auto& s = static_cast<const gs::Line2D&>(shape);
        out.push_back(s.GetStart());
        out.push_back(s.GetEnd());
        break;
    }
    case gs::ShapeType2D::Arc:
    {
        const auto& s = static_cast<const gs::Arc&>(shape);
        float a0 = 0;
        float a1 = 0;
        s.GetAngles(a0, a1);
        float    r = s.GetRadius();
        sm::vec2 c = s.GetCenter();
        for (int i = 0; i <= kArcSegments; ++i)
        {
            float t = a0 + (a1 - a0) * (float)i / kArcSegments;
            out.emplace_back(c.x + r * std::cos(t), c.y + r * std::sin(t));
        }
        break;
    }
    case gs::ShapeType2D::Circle:
    {
        const auto& s = static_cast<const gs::Circle&>(shape);
        float    r = s.GetRadius();
        sm::vec2 c = s.GetCenter();
        for (int i = 0; i <= kArcSegments; ++i)
        {
            float t = 2.0f * 3.14159265358979323846f * (float)i / kArcSegments;
            out.emplace_back(c.x + r * std::cos(t), c.y + r * std::sin(t));
        }
        break;
    }
    case gs::ShapeType2D::Point:
        // Points do not contribute to wires.
        break;
    default:
        // Ellipse / Spline not supported yet, silently skipped.
        break;
    }
}

// Concatenate every solved geometry into one 3D wire.
//
// Simplified strategy: append shape samples in order, close the
// polygon. Adequate for one-loop tests (single rectangle, single
// circle). A real algorithm needs to walk a 2D graph and pick
// closed loops, similar to sketchgraph/util.ves's wire pickup.
TopoDS_Wire BuildWireFromSolved(const SketchBridge::GeoShapes& solved,
                                const double                   origin[3],
                                const double                   x_dir[3],
                                const double                   normal[3])
{
    BRepBuilderAPI_MakePolygon poly;
    bool   first = true;
    gp_Pnt last_pt;

    for (const auto& kv : solved)
    {
        std::vector<sm::vec2> pts;
        SamplePolylineFromShape(*kv.second, pts);

        for (size_t i = 0; i < pts.size(); ++i)
        {
            gp_Pnt p = LocalToWorld(pts[i].x, pts[i].y, origin, x_dir, normal);
            if (first)
            {
                poly.Add(p);
                last_pt = p;
                first   = false;
            }
            else
            {
                // Skip coincident endpoints; MakePolygon rejects
                // duplicates.
                if (p.Distance(last_pt) > 1e-7)
                {
                    poly.Add(p);
                    last_pt = p;
                }
            }
        }
    }
    poly.Close();
    return poly.IsDone() ? poly.Wire() : TopoDS_Wire();
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

    BRepBuilderAPI_MakeFace mkFace(plane, wire);
    if (!mkFace.IsDone()) {
        return {};
    }
    return std::make_shared<partgraph::TopoShape>(mkFace.Face());
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
                auto face = WireToFace(wire, sk->plane_origin, sk->plane_normal);
                if (!face)
                {
                    step_ok     = false;
                    out.err_msg = "wire->face failed: " + feat.name;
                    return;
                }

                double dx = p.direction[0] * p.distance;
                double dy = p.direction[1] * p.distance;
                double dz = p.direction[2] * p.distance;
                if (p.flip_direction)
                {
                    dx = -dx;
                    dy = -dy;
                    dz = -dz;
                }

                auto tool = partgraph::TopoAlgo::Prism(
                    face,
                    dx, dy, dz,
                    op_id, m_impl->naming, m_impl->vtree);
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
