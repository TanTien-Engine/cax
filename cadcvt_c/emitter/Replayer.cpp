#include "cadcvt_c/emitter/Replayer.h"
#include "cadcvt_c/resolve/TopoRefResolver.h"

#include "breptopo_c/CompGraph.h"
#include "breptopo_c/TopoNaming.h"
#include "brepdb_c/VersionTree.h"
#include "partgraph_c/TopoShape.h"

#include <TopoDS.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

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
// The Replay path now builds a CompGraph: each feature becomes a
// graph subtree of typed const + op nodes. Sketches enter the graph
// as a $sketch const + plane Vec3 consts feeding a "sketch_face" op
// (registered as a builtin in breptopo/comp_ops.cpp).
// ============================================================

namespace cadcvt
{

namespace
{

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

                // Sketch is a graph node: a deep copy of the SketchIR
                // is owned by the CompGraph so the graph outlives the
                // DocumentIR. Plane params are separate Vec3 consts so
                // moving the plane doesn't invalidate the solver cache.
                auto sk_copy = std::make_shared<SketchIR>(*sk);
                int sketch_n = cg->AddConst(
                    std::shared_ptr<void>(sk_copy),
                    "sketch:" + sk->name);
                breptopo::Vec3 sk_origin = {
                    sk->plane_origin[0], sk->plane_origin[1], sk->plane_origin[2]};
                breptopo::Vec3 sk_x_dir  = {
                    sk->plane_x_dir[0],  sk->plane_x_dir[1],  sk->plane_x_dir[2]};
                breptopo::Vec3 sk_normal = {
                    sk->plane_normal[0], sk->plane_normal[1], sk->plane_normal[2]};
                int sk_o_n = cg->AddConst(sk_origin, "plane_origin");
                int sk_x_n = cg->AddConst(sk_x_dir,  "plane_x_dir");
                int sk_n_n = cg->AddConst(sk_normal, "plane_normal");
                int face_n = cg->AddOp("sketch_face",
                    {sketch_n, sk_o_n, sk_x_n, sk_n_n},
                    {}, sk->name);

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
