#include "cadapp_c/emitter/Replayer.h"
#include "cadapp_c/ops/sketch_ops.h"
#include "cadapp_c/ops/resolve_ops.h"

#include "breptopo_c/CompGraph.h"
#include "breptopo_c/TopoNaming.h"
#include "brepdb_c/VersionTree.h"
#include "brepkit_c/TopoShape.h"

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
// (registered by cadapp::RegisterSketchOps in cadapp/ops/).
//
// TopoRefIRs (edges for Fillet/Chamfer, faces for Shell) become
// "$toporef" const nodes feeding a "resolve_edge_ref" / "resolve_face_ref"
// op (registered by cadapp::RegisterResolveOps). Geometric matching
// happens at Eval time, not graph-build time, so Replayer never
// has to materialise an intermediate shape.
// ============================================================

namespace cadapp
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

// Build a resolve_*_ref op node from (shape_node, ref) and return
// the op's graph node id. The ref is moved into a heap copy so the
// const node owns it and survives later doc mutations.
int AddResolveRefNode(breptopo::CompGraph& cg,
                      const char*          op_name,
                      int                  shape_node,
                      const TopoRefIR&     ref,
                      double               tolerance,
                      const std::string&   desc)
{
    auto ref_copy = std::make_shared<TopoRefIR>(ref);
    int ref_n  = cg.AddConst(breptopo::TopoRefVal{
        std::static_pointer_cast<void>(ref_copy)}, desc);
    int tol_n  = cg.AddConst(tolerance, "tolerance");
    return cg.AddOp(op_name, {shape_node, ref_n, tol_n}, {}, desc);
}

// Pair recording which TopoRefIR in the user's DocumentIR should
// receive the resolved uid once the corresponding op evaluates.
// Filled while building the graph; consumed in a post-pass after
// Replay() finishes constructing all features.
struct ResolveBack
{
    int        resolve_node = -1;
    TopoRefIR* ref          = nullptr;
};

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
    // Augment breptopo's builtin op set with cadapp's IR-aware ops.
    // breptopo cannot register these itself without depending on
    // cadapp's IR types, so the wiring happens here where both
    // sides are visible.
    RegisterSketchOps(cg->GetRegistry());
    RegisterResolveOps(cg->GetRegistry());
    cg->SetTopoNaming(m_impl->naming);

    int last_node = -1;

    // (resolve_node, &TopoRefIR) pairs collected while building the
    // graph; consumed post-loop to write the resolved uid back into
    // the user's DocumentIR. Empty when opt.write_back_resolved is
    // false, in which case the post-pass is a no-op.
    std::vector<ResolveBack> resolve_back;

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
                        : cg->AddConst(std::shared_ptr<brepkit::TopoShape>{}, "null_ref");
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

                // Build a resolve_edge_ref op per ref and feed the
                // resulting sub-shape nodes as variadic edge inputs
                // to fillet / chamfer. The match itself happens at
                // Eval time -- Replayer stays a pure graph builder.
                std::vector<int> edge_nodes;
                edge_nodes.reserve(p.edges.size());
                for (size_t k = 0; k < p.edges.size(); ++k)
                {
                    int rn = AddResolveRefNode(*cg, "resolve_edge_ref",
                                                last_node, p.edges[k],
                                                opt.topo_tolerance,
                                                feat.name + ":edge");
                    edge_nodes.push_back(rn);
                    if (opt.write_back_resolved)
                        resolve_back.push_back({rn, &p.edges[k]});
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

                std::vector<int> face_nodes;
                face_nodes.reserve(p.faces_to_open.size());
                for (size_t k = 0; k < p.faces_to_open.size(); ++k)
                {
                    int rn = AddResolveRefNode(*cg, "resolve_face_ref",
                                                last_node, p.faces_to_open[k],
                                                opt.topo_tolerance,
                                                feat.name + ":face");
                    face_nodes.push_back(rn);
                    if (opt.write_back_resolved)
                        resolve_back.push_back({rn, &p.faces_to_open[k]});
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

    // Write-back pass for opt.write_back_resolved. Each resolve_*_ref
    // op outputs a ShapeVal whose tag holds the TopoNaming uid; we
    // eval them here (cheap -- downstream fillet/shell evals above
    // have already populated the cache) and copy the uid back into
    // the user-provided TopoRefIR fields. topo_index is not written
    // back: it is a transient, this-eval-only index that would mislead
    // a future replay if persisted.
    if (opt.write_back_resolved)
    {
        for (const auto& rb : resolve_back)
        {
            if (!rb.ref || rb.resolve_node < 0) continue;
            auto v = cg->Eval(rb.resolve_node);
            if (auto* sv = std::get_if<breptopo::ShapeVal>(&v))
            {
                rb.ref->resolved_uid        = sv->tag;
                rb.ref->resolved_topo_index = 0;
            }
        }
    }

    out.comp_graph = cg;
    out.ok         = true;
    return true;
}

} // namespace cadapp
