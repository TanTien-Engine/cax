#include "cadapp_c/emitter/Replayer.h"
#include "cadapp_c/ops/sketch_ops.h"
#include "cadapp_c/ops/resolve_ops.h"

#include "brepgraph_c/computation/CalcGraph.h"
#include "brepgraph_c/history/TopoNaming.h"
#include "brepdb_c/VersionTree.h"
#include "brepkit_c/TopoShape.h"

#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepTools.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <cstdio>
#include <cstdlib>
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
//   LinearPattern / CircularPattern              OK
//   MultiTransform                               OK
//   BossRevolve / CutRevolve                     OK
//   Sweep (FreeCAD AdditivePipe / SubtractivePipe) OK
//   Loft  (FreeCAD AdditiveLoft / SubtractiveLoft) OK
//
// The Replay path now builds a CalcGraph: each feature becomes a
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
int AddResolveRefNode(brepgraph::CalcGraph& cg,
                      const char*          op_name,
                      int                  shape_node,
                      const TopoRefIR&     ref,
                      double               tolerance,
                      const std::string&   desc)
{
    auto ref_copy = std::make_shared<TopoRefIR>(ref);
    int ref_n  = cg.AddConst(brepgraph::TopoRefVal{
        std::static_pointer_cast<void>(ref_copy)}, desc);
    int tol_n  = cg.AddConst(tolerance, "tolerance");
    return cg.AddOp(op_name, {shape_node, ref_n, tol_n}, {}, desc);
}

// Build the sketch_face graph subtree for a SketchIR and return the
// face op's node id. A deep copy of the SketchIR is owned by the
// CalcGraph so the graph outlives the DocumentIR; plane params are
// kept as separate Vec3 consts so moving the plane re-runs the
// face step without re-solving constraints.
int AddSketchFaceNode(brepgraph::CalcGraph& cg, const SketchIR& sk)
{
    auto sk_copy = std::make_shared<SketchIR>(sk);
    int sketch_n = cg.AddConst(
        std::shared_ptr<void>(sk_copy),
        "sketch:" + sk.name);
    brepgraph::Vec3 sk_origin = {
        sk.plane_origin[0], sk.plane_origin[1], sk.plane_origin[2]};
    brepgraph::Vec3 sk_x_dir  = {
        sk.plane_x_dir[0],  sk.plane_x_dir[1],  sk.plane_x_dir[2]};
    brepgraph::Vec3 sk_normal = {
        sk.plane_normal[0], sk.plane_normal[1], sk.plane_normal[2]};
    int sk_o_n = cg.AddConst(sk_origin, "plane_origin");
    int sk_x_n = cg.AddConst(sk_x_dir,  "plane_x_dir");
    int sk_n_n = cg.AddConst(sk_normal, "plane_normal");
    return cg.AddOp("sketch_face",
        {sketch_n, sk_o_n, sk_x_n, sk_n_n},
        {}, sk.name);
}

// Build the sketch_wire graph subtree -- same plane-param wiring as
// AddSketchFaceNode but the op returns the stitched wire (no face
// filling). Sweep uses this for the spine.
int AddSketchWireNode(brepgraph::CalcGraph& cg, const SketchIR& sk)
{
    auto sk_copy = std::make_shared<SketchIR>(sk);
    int sketch_n = cg.AddConst(
        std::shared_ptr<void>(sk_copy),
        "sketch:" + sk.name);
    brepgraph::Vec3 sk_origin = {
        sk.plane_origin[0], sk.plane_origin[1], sk.plane_origin[2]};
    brepgraph::Vec3 sk_x_dir  = {
        sk.plane_x_dir[0],  sk.plane_x_dir[1],  sk.plane_x_dir[2]};
    brepgraph::Vec3 sk_normal = {
        sk.plane_normal[0], sk.plane_normal[1], sk.plane_normal[2]};
    int sk_o_n = cg.AddConst(sk_origin, "plane_origin");
    int sk_x_n = cg.AddConst(sk_x_dir,  "plane_x_dir");
    int sk_n_n = cg.AddConst(sk_normal, "plane_normal");
    return cg.AddOp("sketch_wire",
        {sketch_n, sk_o_n, sk_x_n, sk_n_n},
        {}, sk.name + ":wire");
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

// Look up an ext_params key; return def when absent.
double ExtParam(const FeatureIR& feat, const char* key, double def)
{
    auto it = feat.ext_params.find(key);
    return (it == feat.ext_params.end()) ? def : it->second;
}

// Per-feature record of the "tool" sub-graph it contributes to the
// running body, plus the body it operated on. Pattern features
// (PolarPattern / LinearPattern / Mirrored / MultiTransform) with
// an Originals link list pull the originals' (tool, base, op) out
// of this map so the pattern multiplies just the tool, then fuses /
// cuts the multiplied tool against base (FreeCAD's semantics).
//
// op_kind:
//   'f' fuse  (additive primitives, BossExtrude)
//   'c' cut   (subtractive primitives, CutExtrude)
//   '0' replace (no boolean against a base; first feature in a body)
struct FeatureToolInfo
{
    int  tool_node = -1;
    int  base_node = -1;
    char op_kind   = '0';
};

// Read FeatureIR's ext_params for "originals_id_<i>" entries and
// return the resolved tool/base records that the pattern op should
// operate on. Returns empty when Originals is empty or unresolved,
// which signals the caller to fall back to "apply pattern to
// last_node".
std::vector<FeatureToolInfo> ResolveOriginals(
    const FeatureIR&                                 feat,
    const std::map<uint32_t, FeatureToolInfo>&       feature_tools)
{
    std::vector<FeatureToolInfo> out;
    auto cnt_it = feat.ext_params.find("originals_count");
    if (cnt_it == feat.ext_params.end()) return out;
    int n = (int)cnt_it->second;
    for (int i = 0; i < n; ++i)
    {
        auto id_it = feat.ext_params.find("originals_id_" + std::to_string(i));
        if (id_it == feat.ext_params.end()) continue;
        uint32_t id = (uint32_t)id_it->second;
        auto t_it = feature_tools.find(id);
        if (t_it == feature_tools.end()) continue;
        out.push_back(t_it->second);
    }
    return out;
}

// Combine a patterned "tool" node with the original feature's base
// shape according to the original's op_kind. For 'f' (additive) we
// fuse; for 'c' (subtractive) we cut; for '0' (first feature in a
// body) the pattern result IS the new body.
int CombinePatternedTool(brepgraph::CalcGraph& cg,
                         const FeatureToolInfo& orig,
                         int                    pattern_node,
                         const std::string&     desc)
{
    if (orig.op_kind == 'f' && orig.base_node >= 0) {
        return cg.AddOp("fuse", {orig.base_node, pattern_node}, {}, desc);
    }
    if (orig.op_kind == 'c' && orig.base_node >= 0) {
        return cg.AddOp("cut", {orig.base_node, pattern_node}, {}, desc);
    }
    return pattern_node;
}

// Build a "mirror with original" subtree: mirror the input shape
// across the given plane and fuse with the original. FreeCAD's
// PartDesign::Mirrored produces orig + mirror; our `mirror` op only
// returns the mirror, so we add the fuse here.
int AddMirrorWithOriginal(brepgraph::CalcGraph& cg,
                          int                   shape_node,
                          const brepgraph::Vec3& origin,
                          const brepgraph::Vec3& normal,
                          const std::string&    desc)
{
    int o_n = cg.AddConst(origin, "origin");
    int n_n = cg.AddConst(normal, "normal");
    int m_n = cg.AddOp("mirror", {shape_node, o_n, n_n}, {}, desc + ":mirror");
    return cg.AddOp("fuse", {shape_node, m_n}, {}, desc + ":fuse_orig");
}

// Apply mirror(Fi.tool) to `body` for each original Fi, using Fi's
// op_kind. Used for FreeCAD Mirrored with Originals = [F1..Fn]:
// each Fi's tool effect already lives in `body` (it was applied when
// Fi was replayed), so we just add the mirrored copy on top.
//
// op_kind dispatch:
//   '0' (no base, first feature in body) -> treat as fuse
//   'f' (additive)                       -> fuse(body, mirror(tool))
//   'c' (subtractive)                    -> cut (body, mirror(tool))
int AddMirroredOriginals(brepgraph::CalcGraph&              cg,
                         int                                body,
                         const std::vector<FeatureToolInfo>& origs,
                         const brepgraph::Vec3&             origin,
                         const brepgraph::Vec3&             normal,
                         const std::string&                 desc)
{
    int o_n = cg.AddConst(origin, "origin");
    int n_n = cg.AddConst(normal, "normal");
    int node = body;
    for (size_t i = 0; i < origs.size(); ++i)
    {
        const auto& orig = origs[i];
        std::string tag  = desc + ":orig" + std::to_string(i);
        int m_n = cg.AddOp("mirror",
                            {orig.tool_node, o_n, n_n}, {}, tag + ":mirror");
        const char* op = (orig.op_kind == 'c') ? "cut" : "fuse";
        node = cg.AddOp(op, {node, m_n}, {}, tag + ":" + op);
    }
    return node;
}

// Post-process a freshly-built primitive node:
//   1. Apply the FreeCAD Placement stashed in ext_params (rotate
//      around the body origin, then translate). This positions
//      PartDesign primitives inside their parent body so the
//      subsequent boolean lands in the right spot.
//   2. Fuse with the running body shape for PartDesign::Additive*,
//      or cut from it for PartDesign::Subtractive*. Plain Part::*
//      primitives (no freecad_type tag) keep the legacy "replace
//      last_node" behavior since they are independent objects in
//      FreeCAD.
//
// Returns the final node id (possibly the same as `prim_node`) and
// whether the caller should treat the step as ok. step_ok becomes
// false only when a subtractive op has no running body to cut from.
//
// tool_info_out captures the placed primitive node (the "tool") and
// the base / op_kind it combined with. A later PolarPattern with
// Originals = [this feature] uses these to multiply the tool only,
// then re-apply cut / fuse against the recorded base.
int FinalizePrimitiveNode(brepgraph::CalcGraph& cg,
                          const FeatureIR&     feat,
                          int                  prim_node,
                          int                  last_node,
                          ReplayResult&        out,
                          bool&                step_ok,
                          FeatureToolInfo*     tool_info_out = nullptr)
{
    int cur = prim_node;

    bool   has_t   = feat.ext_params.count("placement_px") > 0;
    bool   has_r   = feat.ext_params.count("placement_angle") > 0;
    if (has_r)
    {
        brepgraph::Vec3 origin = {0.0, 0.0, 0.0};
        brepgraph::Vec3 axis   = {
            ExtParam(feat, "placement_ox", 0.0),
            ExtParam(feat, "placement_oy", 0.0),
            ExtParam(feat, "placement_oz", 1.0)};
        double angle = ExtParam(feat, "placement_angle", 0.0);
        int o_n = cg.AddConst(origin, "place_origin");
        int d_n = cg.AddConst(axis,   "place_axis");
        int a_n = cg.AddConst(angle,  "place_angle");
        cur = cg.AddOp("rotate", {cur, o_n, d_n, a_n}, {}, feat.name + ":place_rot");
    }
    if (has_t)
    {
        brepgraph::Vec3 off = {
            ExtParam(feat, "placement_px", 0.0),
            ExtParam(feat, "placement_py", 0.0),
            ExtParam(feat, "placement_pz", 0.0)};
        int o_n = cg.AddConst(off, "place_offset");
        cur = cg.AddOp("translate", {cur, o_n}, {}, feat.name + ":place_tr");
    }

    // `cur` here is the placed tool shape. Record it before fuse/cut
    // so a downstream pattern can re-use the tool.
    if (tool_info_out) {
        tool_info_out->tool_node = cur;
        tool_info_out->base_node = last_node;
        tool_info_out->op_kind   = '0';
    }

    auto ft_it = feat.ext_strings.find("freecad_type");
    if (ft_it == feat.ext_strings.end()) {
        return cur;
    }
    const std::string& ft = ft_it->second;
    bool is_sub = ft.rfind("PartDesign::Subtractive", 0) == 0;
    bool is_add = ft.rfind("PartDesign::Additive",    0) == 0;

    if (is_sub)
    {
        if (last_node < 0)
        {
            step_ok     = false;
            out.err_msg = "Subtractive primitive without base shape: " + feat.name;
            return cur;
        }
        if (tool_info_out) tool_info_out->op_kind = 'c';
        return cg.AddOp("cut", {last_node, cur}, {}, feat.name);
    }
    if (is_add)
    {
        if (last_node < 0) {
            return cur;
        }
        if (tool_info_out) tool_info_out->op_kind = 'f';
        return cg.AddOp("fuse", {last_node, cur}, {}, feat.name);
    }
    return cur;
}

} // anonymous namespace

// ============================================================
// Impl
// ============================================================

struct Replayer::Impl
{
    std::shared_ptr<brepgraph::TopoNaming> naming;
    std::shared_ptr<brepdb::VersionTree>  vtree;
};

Replayer::Replayer()
    : m_impl(std::make_unique<Impl>())
{
}

Replayer::~Replayer() = default;

void Replayer::SetNaming(const std::shared_ptr<brepgraph::TopoNaming>& tn)
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
        m_impl->naming = std::make_shared<brepgraph::TopoNaming>();
    }
    if (!m_impl->vtree) {
        m_impl->vtree = std::make_shared<brepdb::VersionTree>();
    }

    out.naming = m_impl->naming;
    out.vtree  = m_impl->vtree;

    auto cg = std::make_shared<brepgraph::CalcGraph>();
    // Augment brepgraph's builtin op set with cadapp's IR-aware ops.
    // brepgraph cannot register these itself without depending on
    // cadapp's IR types, so the wiring happens here where both
    // sides are visible.
    RegisterSketchOps(cg->GetRegistry());
    RegisterResolveOps(cg->GetRegistry());
    cg->SetTopoNaming(m_impl->naming);

    int last_node = -1;

    // sketch_id -> face op node. Populated when a FeatPayloadSketch
    // feature is visited; consumed by later Extrude / Revolve / etc.
    // features that reference the same sketch. Building the face
    // once means a sketch shared by several features (e.g. Pad +
    // Pocket on the same profile) lands as a single subtree.
    std::map<uint32_t, int> sketch_face_nodes;

    // (resolve_node, &TopoRefIR) pairs collected while building the
    // graph; consumed post-loop to write the resolved uid back into
    // the user's DocumentIR. Empty when opt.write_back_resolved is
    // false, in which case the post-pass is a no-op.
    std::vector<ResolveBack> resolve_back;

    // feature_id -> the (tool, base, op_kind) record for that feature.
    // Populated by primitive / extrude handlers; consumed by pattern
    // / mirror / MultiTransform handlers that carry an Originals list.
    std::map<uint32_t, FeatureToolInfo> feature_tools;

    for (size_t i = 0; i < doc.features.size(); ++i)
    {
        auto& feat = doc.features[i];
        if (feat.suppressed)
        {
            out.op_ids.push_back(0);
            continue;
        }

        //if (feat.name == "Fillet002")
        //    break;

        int  node    = -1;
        bool step_ok = true;

        std::visit([&](auto& p)
        {
            using T = std::decay_t<decltype(p)>;

            // ---- Sketch ----
            if constexpr (std::is_same_v<T, FeatPayloadSketch>)
            {
                // Build the sketch_face subtree up-front and stash it
                // in sketch_face_nodes; downstream features (Extrude,
                // Revolve, ...) read from there so a sketch shared by
                // several features lands as a single subtree in the
                // calc graph. The feature itself doesn't contribute a
                // 3D body, so node stays -1 and last_node is unchanged.
                const SketchIR* sk = FindSketch(doc, p.sketch_id);
                if (sk) {
                    sketch_face_nodes[p.sketch_id] = AddSketchFaceNode(*cg, *sk);
                }
            }

            // ---- Primitives ----
            //
            // After building the bare primitive op, FinalizePrimitiveNode
            // applies the FreeCAD Placement and turns
            // PartDesign::Additive*/Subtractive* into the
            // corresponding fuse/cut against the running body shape.
            // tool_info captures the placed tool and the fuse/cut
            // kind so a downstream pattern with Originals = [this]
            // can multiply just the tool.
            else if constexpr (std::is_same_v<T, FeatPayloadPrimBox>)
            {
                int l = cg->AddConst(p.length, "length");
                int w = cg->AddConst(p.width,  "width");
                int h = cg->AddConst(p.height, "height");
                int prim = cg->AddOp("box", {l, w, h}, {}, feat.name);
                FeatureToolInfo ti;
                node = FinalizePrimitiveNode(*cg, feat, prim, last_node, out, step_ok, &ti);
                feature_tools[feat.id] = ti;
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimCylinder>)
            {
                int r = cg->AddConst(p.radius, "radius");
                int h = cg->AddConst(p.height, "height");
                int prim = cg->AddOp("cylinder", {r, h}, {}, feat.name);
                FeatureToolInfo ti;
                node = FinalizePrimitiveNode(*cg, feat, prim, last_node, out, step_ok, &ti);
                feature_tools[feat.id] = ti;
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimCone>)
            {
                int r1 = cg->AddConst(p.radius1, "radius1");
                int r2 = cg->AddConst(p.radius2, "radius2");
                int h  = cg->AddConst(p.height,  "height");
                int prim = cg->AddOp("cone", {r1, r2, h}, {}, feat.name);
                FeatureToolInfo ti;
                node = FinalizePrimitiveNode(*cg, feat, prim, last_node, out, step_ok, &ti);
                feature_tools[feat.id] = ti;
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimSphere>)
            {
                int r = cg->AddConst(p.radius, "radius");
                int prim = cg->AddOp("sphere", {r}, {}, feat.name);
                FeatureToolInfo ti;
                node = FinalizePrimitiveNode(*cg, feat, prim, last_node, out, step_ok, &ti);
                feature_tools[feat.id] = ti;
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimTorus>)
            {
                int r1 = cg->AddConst(p.major_radius, "major_radius");
                int r2 = cg->AddConst(p.minor_radius, "minor_radius");
                int prim = cg->AddOp("torus", {r1, r2}, {}, feat.name);
                FeatureToolInfo ti;
                node = FinalizePrimitiveNode(*cg, feat, prim, last_node, out, step_ok, &ti);
                feature_tools[feat.id] = ti;
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

                // Reuse the face built by the upstream Sketch feature
                // when present; otherwise (e.g. a DocumentIR that
                // forgot to emit a Sketch FeatureIR around its sketch)
                // build it inline so the extrude still resolves.
                int face_n;
                auto it = sketch_face_nodes.find(p.sketch_id);
                if (it != sketch_face_nodes.end()) {
                    face_n = it->second;
                } else {
                    face_n = AddSketchFaceNode(*cg, *sk);
                    sketch_face_nodes[p.sketch_id] = face_n;
                }

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

                // FreeCAD TwoLengths (Pad Type=4) lands here with
                // end_type == Blind but distance2 > 0: the reader has
                // stashed the second-side length in distance2 and left
                // end_type2 at Blind. The simple `prism` op below only
                // consumes `distance`, so without this distance2 > 0
                // gate the second half of the extrusion disappears
                // silently (see Page_077 Pad002: the 42mm +Z half of
                // the central cylinder went missing because the only
                // visible 6mm half was buried inside the base body).
                bool two_sided = (p.distance2 > 1e-15) ||
                                 (p.end_type2 != ExtrudeEndType::Blind);

                int tool_n;
                if (p.end_type != ExtrudeEndType::Blind || two_sided)
                {
                    brepgraph::Vec3 dir = {sign * world_dir[0],
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
                    brepgraph::Vec3 dir = {dx, dy, dz};
                    int dir_n = cg->AddConst(dir, "direction");
                    tool_n = cg->AddOp("prism", {face_n, dir_n}, {}, feat.name);
                }

                // Track the extrude tool so a downstream pattern with
                // Originals = [this Pad/Pocket] can multiply just the
                // prism instead of the whole body.
                FeatureToolInfo ti;
                ti.tool_node = tool_n;
                ti.base_node = last_node;

                if (feat.type == FeatType::BossExtrude)
                {
                    if (last_node < 0)
                    {
                        node = tool_n;
                        ti.op_kind = '0';
                    }
                    else
                    {
                        node = cg->AddOp("fuse", {last_node, tool_n},
                                          {}, feat.name);
                        ti.op_kind = 'f';
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
                    ti.op_kind = 'c';
                }
                // Mirror FreeCAD's PartDesign::Pad / Pocket default
                // Refine=true: append UnifySameDomain so coplanar side
                // faces from a multi-segment sketch contour collapse
                // and downstream Fillet runs on a single smooth strip
                // rather than N narrow strips that fracture the blend.
                // tool_n stays unrefined so a downstream PolarPattern
                // / Mirror with Originals=[this feat] still multiplies
                // a clean prism, not a refined body.
                if (opt.refine_after_primitive && node >= 0) {
                    node = cg->AddOp("refine", {node}, {},
                                      feat.name + ":refine");
                }
                feature_tools[feat.id] = ti;
            }

            // ---- Revolve (Boss / Cut) ----
            //
            // Mirror of the Extrude branch: reuse / build the sketch's
            // face, add a `revolve` op around the axis carried by the
            // payload, and fuse / cut against the running body.
            //
            // Negative angle (or Reversed=true via flip_direction) flips
            // the sweep direction; angle near 2*PI is treated as a full
            // revolution via the `is_full` flag so OCCT closes the
            // resulting solid instead of leaving a hairline seam.
            else if constexpr (std::is_same_v<T, FeatPayloadRevolve>)
            {
                const SketchIR* sk = FindSketch(doc, p.sketch_id);
                if (!sk)
                {
                    step_ok     = false;
                    out.err_msg = "missing sketch for feature " + feat.name;
                    return;
                }

                int face_n;
                auto it = sketch_face_nodes.find(p.sketch_id);
                if (it != sketch_face_nodes.end()) {
                    face_n = it->second;
                } else {
                    face_n = AddSketchFaceNode(*cg, *sk);
                    sketch_face_nodes[p.sketch_id] = face_n;
                }

                double sign = p.flip_direction ? -1.0 : 1.0;
                brepgraph::Vec3 ax_o = {p.axis_origin[0],
                                       p.axis_origin[1],
                                       p.axis_origin[2]};
                brepgraph::Vec3 ax_d = {p.axis_dir[0],
                                       p.axis_dir[1],
                                       p.axis_dir[2]};
                int o_n = cg->AddConst(ax_o, "axis_origin");
                int d_n = cg->AddConst(ax_d, "axis_dir");
                int a_n = cg->AddConst(sign * p.angle, "angle");
                bool is_full = std::abs(p.angle - 2.0 * 3.14159265358979323846)
                                < 1e-6;
                int f_n = cg->AddConst(is_full, "is_full");
                int tool_n = cg->AddOp("revolve",
                                        {face_n, o_n, d_n, a_n, f_n},
                                        {}, feat.name);

                FeatureToolInfo ti;
                ti.tool_node = tool_n;
                ti.base_node = last_node;

                if (feat.type == FeatType::BossRevolve)
                {
                    if (last_node < 0)
                    {
                        node = tool_n;
                        ti.op_kind = '0';
                    }
                    else
                    {
                        node = cg->AddOp("fuse", {last_node, tool_n},
                                          {}, feat.name);
                        ti.op_kind = 'f';
                    }
                }
                else
                {
                    if (last_node < 0)
                    {
                        step_ok     = false;
                        out.err_msg = "CutRevolve without base shape: " + feat.name;
                        return;
                    }
                    node = cg->AddOp("cut", {last_node, tool_n},
                                      {}, feat.name);
                    ti.op_kind = 'c';
                }
                // See Extrude branch above for rationale.
                if (opt.refine_after_primitive && node >= 0) {
                    node = cg->AddOp("refine", {node}, {},
                                      feat.name + ":refine");
                }
                feature_tools[feat.id] = ti;
            }

            // ---- Sweep (FreeCAD PartDesign::AdditivePipe /
            //              SubtractivePipe) ----
            //
            // Build the profile face from the profile sketch (re-using
            // any already-built sketch_face node), build the spine
            // wire from the spine sketch (id is stashed in ext_params
            // by the reader because FeatPayloadSweep::path_ref is
            // shaped for 3D edge refs, not sketches), then either
            // fuse (Additive) or cut (Subtractive) the swept tool
            // against the running body.
            else if constexpr (std::is_same_v<T, FeatPayloadSweep>)
            {
                const SketchIR* profile_sk = FindSketch(doc, p.profile_sketch_id);
                if (!profile_sk)
                {
                    step_ok     = false;
                    out.err_msg = "missing profile sketch for sweep: " + feat.name;
                    return;
                }

                int face_n;
                auto fit = sketch_face_nodes.find(p.profile_sketch_id);
                if (fit != sketch_face_nodes.end()) {
                    face_n = fit->second;
                } else {
                    face_n = AddSketchFaceNode(*cg, *profile_sk);
                    sketch_face_nodes[p.profile_sketch_id] = face_n;
                }

                uint32_t spine_id = 0xFFFFFFFF;
                auto sit = feat.ext_params.find("spine_sketch_id");
                if (sit != feat.ext_params.end()) {
                    spine_id = (uint32_t)sit->second;
                }
                const SketchIR* spine_sk = FindSketch(doc, spine_id);
                if (!spine_sk)
                {
                    step_ok     = false;
                    out.err_msg = "missing spine sketch for sweep: " + feat.name;
                    return;
                }

                int wire_n   = AddSketchWireNode(*cg, *spine_sk);
                int solid_n  = cg->AddConst(true, "is_solid");
                int tool_n   = cg->AddOp("sweep",
                                          {face_n, wire_n, solid_n},
                                          {}, feat.name);

                bool subtractive = false;
                auto tit = feat.ext_strings.find("freecad_type");
                if (tit != feat.ext_strings.end()
                    && tit->second == "PartDesign::SubtractivePipe")
                {
                    subtractive = true;
                }

                FeatureToolInfo ti;
                ti.tool_node = tool_n;
                ti.base_node = last_node;

                if (!subtractive)
                {
                    if (last_node < 0)
                    {
                        node = tool_n;
                        ti.op_kind = '0';
                    }
                    else
                    {
                        node = cg->AddOp("fuse", {last_node, tool_n},
                                          {}, feat.name);
                        ti.op_kind = 'f';
                    }
                }
                else
                {
                    if (last_node < 0)
                    {
                        step_ok     = false;
                        out.err_msg = "SubtractivePipe without base shape: " + feat.name;
                        return;
                    }
                    node = cg->AddOp("cut", {last_node, tool_n},
                                      {}, feat.name);
                    ti.op_kind = 'c';
                }
                // See Extrude branch above for rationale.
                if (opt.refine_after_primitive && node >= 0) {
                    node = cg->AddOp("refine", {node}, {},
                                      feat.name + ":refine");
                }
                feature_tools[feat.id] = ti;
            }

            // ---- Loft (FreeCAD PartDesign::AdditiveLoft /
            //             SubtractiveLoft) ----
            //
            // Build one sketch_wire node per profile sketch (the reader
            // packs Profile + Sections into profile_sketch_ids in order),
            // feed them as variadic inputs to the `loft` op (is_solid=true
            // because FreeCAD AdditiveLoft yields a solid body), then
            // either fuse (Additive) or cut (Subtractive) the loft tool
            // against the running body.
            else if constexpr (std::is_same_v<T, FeatPayloadLoft>)
            {
                if (p.profile_sketch_ids.size() < 2)
                {
                    step_ok     = false;
                    out.err_msg = "loft needs >= 2 profiles: " + feat.name;
                    return;
                }

                std::vector<int> wire_nodes;
                wire_nodes.reserve(p.profile_sketch_ids.size());
                for (uint32_t sid : p.profile_sketch_ids)
                {
                    const SketchIR* sk = FindSketch(doc, sid);
                    if (!sk)
                    {
                        step_ok     = false;
                        out.err_msg = "missing profile sketch for loft: " + feat.name;
                        return;
                    }
                    wire_nodes.push_back(AddSketchWireNode(*cg, *sk));
                }

                int is_solid_n = cg->AddConst(true, "is_solid");
                int tool_n     = cg->AddOp("loft",
                                            {is_solid_n},
                                            wire_nodes, feat.name);

                bool subtractive = false;
                auto tit = feat.ext_strings.find("freecad_type");
                if (tit != feat.ext_strings.end()
                    && tit->second == "PartDesign::SubtractiveLoft")
                {
                    subtractive = true;
                }

                FeatureToolInfo ti;
                ti.tool_node = tool_n;
                ti.base_node = last_node;

                if (!subtractive)
                {
                    if (last_node < 0)
                    {
                        node = tool_n;
                        ti.op_kind = '0';
                    }
                    else
                    {
                        node = cg->AddOp("fuse", {last_node, tool_n},
                                          {}, feat.name);
                        ti.op_kind = 'f';
                    }
                }
                else
                {
                    if (last_node < 0)
                    {
                        step_ok     = false;
                        out.err_msg = "SubtractiveLoft without base shape: " + feat.name;
                        return;
                    }
                    node = cg->AddOp("cut", {last_node, tool_n},
                                      {}, feat.name);
                    ti.op_kind = 'c';
                }
                // See Extrude branch above for rationale.
                if (opt.refine_after_primitive && node >= 0) {
                    node = cg->AddOp("refine", {node}, {},
                                      feat.name + ":refine");
                }
                feature_tools[feat.id] = ti;
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
                // The body the resolver / ChFi3d looks at. Starts as
                // last_node (no pre-dressup refine; that pass was
                // tried via c17a1d22 and silently shifted resolver
                // matches to the wrong cax edge, so it's reverted)
                // and gets re-bound to a split_body_at_points node
                // below when the reader supplied split hints.
                int body_n = last_node;

                // (Page_015 Pad002 75mm+85mm -> 163mm closed BSpline).
                // Splitting before resolve means the downstream
                // resolve_edge_ref finds the post-split sub-edges,
                // and ChFi3d gets edges it can actually fillet.
                if (!p.split_hints.empty())
                {
                    std::vector<int> vert_nodes;
                    vert_nodes.reserve(p.split_hints.size());
                    for (const auto& h : p.split_hints)
                    {
                        TopoDS_Vertex tv = BRepBuilderAPI_MakeVertex(
                            gp_Pnt(h[0], h[1], h[2]));
                        auto ts = std::make_shared<brepkit::TopoShape>(tv);
                        int vn = cg->AddConst(ts, "split_hint");
                        vert_nodes.push_back(vn);
                    }
                    body_n = cg->AddOp("split_body_at_points",
                                        {body_n}, vert_nodes,
                                        feat.name + ":split_body");
                }

                // Build a resolve_(edge|face)_ref op per ref and feed
                // the resulting sub-shape nodes as variadic edge inputs
                // to fillet / chamfer. The match itself happens at
                // Eval time -- Replayer stays a pure graph builder.
                //
                // FreeCAD lets users pick faces for a fillet (round
                // every edge of the face). Those refs arrive with
                // kind=Face; route them through resolve_face_ref so
                // the resolver searches the face map, and TopoAlgo
                // explodes the resolved face into its edges via
                // TopExp_Explorer at apply time.
                // Dressup picks are inherently fuzzy: the ref midpoint
                // comes from FreeCAD's saved geometry (PartShape*.brp),
                // but cax replays the upstream feature chain with its
                // own Mirror / Pattern / BOP implementations, and the
                // resulting body's edges can sit mm away from FreeCAD's
                // in the same logical place (different BOP fuzzy
                // tolerance, different seam handling). We saw cax's
                // Mirror+refine body land an edge 8.9 mm away from
                // FreeCAD's Edge36 midpoint on Page_026_Exercise2D-18,
                // far beyond the default 1 mm topo_tolerance.
                //
                // Widen the resolver tolerance for dressup picks to
                // scale with the dressup size: a user picking an edge
                // for a 1.5 mm fillet is fundamentally OK with the
                // match landing within a few mm because the resulting
                // blend covers that neighbourhood anyway. 5x the
                // dressup size is the empirical sweet spot -- tight
                // enough that two distinct picked edges don't compete
                // for the same match on symmetric models, loose enough
                // to absorb the implementation drift.
                double dressup_size = 0.0;
                if constexpr (std::is_same_v<T, FeatPayloadFillet>) {
                    dressup_size = p.radius;
                } else {
                    dressup_size = p.distance1;
                }
                double dressup_tol = std::max(opt.topo_tolerance,
                                              5.0 * dressup_size);

                std::vector<int> edge_nodes;
                edge_nodes.reserve(p.edges.size());
                for (size_t k = 0; k < p.edges.size(); ++k)
                {
                    const char* op = (p.edges[k].kind == TopoRefIR::Kind::Face)
                                       ? "resolve_face_ref"
                                       : "resolve_edge_ref";
                    int rn = AddResolveRefNode(*cg, op,
                                                body_n, p.edges[k],
                                                dressup_tol,
                                                feat.name + ":edge");
                    edge_nodes.push_back(rn);
                    if (opt.write_back_resolved)
                        resolve_back.push_back({rn, &p.edges[k]});
                }

                if constexpr (std::is_same_v<T, FeatPayloadFillet>)
                {
                    int r = cg->AddConst(p.radius, "radius");
                    node = cg->AddOp("fillet", {body_n, r},
                                      edge_nodes, feat.name);
                }
                else
                {
                    int d = cg->AddConst(p.distance1, "dist");
                    node = cg->AddOp("chamfer", {body_n, d},
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
            //
            // FreeCAD's PartDesign::Mirrored produces orig + mirror.
            // Our `mirror` op only returns the mirror image, so we
            // always add the original back ourselves.
            //
            // Originals = [F1..Fn]: each Fi's tool effect already
            // lives in last_node (it was applied when Fi was replayed).
            // We add mirror(Fi.tool) for every Fi using Fi.op_kind so
            // every original contributes its mirror -- previously only
            // origs[0] was honored, which silently dropped pads/pockets
            // when users mirrored multiple features together.
            //
            // Originals empty: mirror the whole running body and fuse.
            else if constexpr (std::is_same_v<T, FeatPayloadMirror>)
            {
                brepgraph::Vec3 origin = {p.plane_origin[0],
                                         p.plane_origin[1],
                                         p.plane_origin[2]};
                brepgraph::Vec3 normal = {p.plane_normal[0],
                                         p.plane_normal[1],
                                         p.plane_normal[2]};

                auto origs = ResolveOriginals(feat, feature_tools);
                if (!origs.empty() && last_node >= 0)
                {
                    node = AddMirroredOriginals(*cg, last_node, origs,
                                                origin, normal, feat.name);
                }
                else if (last_node >= 0)
                {
                    node = AddMirrorWithOriginal(*cg, last_node, origin, normal,
                                                  feat.name);
                }
                else
                {
                    step_ok = false;
                    return;
                }
            }

            // ---- LinearPattern ----
            //
            // When Originals = [X] is set, multiply X's tool only and
            // combine with X's base; otherwise pattern the whole body.
            else if constexpr (std::is_same_v<T, FeatPayloadLinearPattern>)
            {
                brepgraph::Vec3 dir1 = {p.dir1[0], p.dir1[1], p.dir1[2]};
                brepgraph::Vec3 dir2 = {p.dir2[0], p.dir2[1], p.dir2[2]};
                int d1 = cg->AddConst(dir1, "dir1");
                int c1 = cg->AddConst((int)p.count1, "count1");
                int s1 = cg->AddConst(p.spacing1, "spacing1");
                int d2 = cg->AddConst(dir2, "dir2");
                int c2 = cg->AddConst((int)p.count2, "count2");
                int s2 = cg->AddConst(p.spacing2, "spacing2");

                auto origs = ResolveOriginals(feat, feature_tools);
                int target = -1;
                if (!origs.empty()) {
                    target = origs[0].tool_node;
                } else if (last_node >= 0) {
                    target = last_node;
                } else {
                    step_ok = false;
                    return;
                }
                int pat = cg->AddOp("linear_pattern",
                                     {target, d1, c1, s1, d2, c2, s2},
                                     {}, feat.name);
                node = origs.empty()
                        ? pat
                        : CombinePatternedTool(*cg, origs[0], pat, feat.name);
            }

            // ---- CircularPattern ----
            else if constexpr (std::is_same_v<T, FeatPayloadCircularPattern>)
            {
                brepgraph::Vec3 origin = {p.axis_origin[0],
                                         p.axis_origin[1],
                                         p.axis_origin[2]};
                brepgraph::Vec3 axis   = {p.axis_dir[0],
                                         p.axis_dir[1],
                                         p.axis_dir[2]};
                int o = cg->AddConst(origin, "axis_origin");
                int a = cg->AddConst(axis,   "axis_dir");
                int c = cg->AddConst((int)p.count, "count");
                int t = cg->AddConst(p.total_angle, "total_angle");

                auto origs = ResolveOriginals(feat, feature_tools);
                int target = -1;
                if (!origs.empty()) {
                    target = origs[0].tool_node;
                } else if (last_node >= 0) {
                    target = last_node;
                } else {
                    step_ok = false;
                    return;
                }
                int pat = cg->AddOp("circular_pattern",
                                     {target, o, a, c, t},
                                     {}, feat.name);
                node = origs.empty()
                        ? pat
                        : CombinePatternedTool(*cg, origs[0], pat, feat.name);
            }

            // ---- MultiTransform ----
            //
            // FreeCAD's MultiTransform applies a chain of Mirror /
            // LinearPattern / CircularPattern in order. Each step's
            // effect must include the result of the previous step
            // (FreeCAD's cartesian-product trsf semantics is order-
            // equivalent to "chain ops whose result includes the
            // input"), so each Mirror step here is materialised as
            // fuse(cur, mirror(cur)) -- mirror op alone returns only
            // the mirror image. LinearPattern / CircularPattern ops
            // already include the original in their result.
            //
            // When Originals = [F1..Fn] is set every original
            // contributes its own chained pattern -- previously only
            // origs[0] was honored, which silently dropped the rest
            // (see Page_023 where Originals = [Pad..Pad003, Pocket]
            // collapsed to "just Pad patterned" and the stacked pad
            // layers + pocket holes disappeared). Each Fi.tool is
            // already at position 0 in last_node, so re-applying the
            // full pattern (which includes position 0) is idempotent
            // for fuse / cut and only the new positions land on body.
            else if constexpr (std::is_same_v<T, FeatPayloadMultiTransform>)
            {
                auto apply_chain = [&](int input_node, const std::string& tag) -> int
                {
                    int cur = input_node;
                    for (size_t si = 0; si < p.steps.size(); ++si)
                    {
                        const auto& s = p.steps[si];
                        std::string step_name = tag + ":step" + std::to_string(si);
                        if (s.kind == MultiTransformStep::Kind::Mirror)
                        {
                            brepgraph::Vec3 origin = {s.plane_origin[0],
                                                     s.plane_origin[1],
                                                     s.plane_origin[2]};
                            brepgraph::Vec3 normal = {s.plane_normal[0],
                                                     s.plane_normal[1],
                                                     s.plane_normal[2]};
                            cur = AddMirrorWithOriginal(*cg, cur, origin, normal,
                                                        step_name);
                        }
                        else if (s.kind == MultiTransformStep::Kind::LinearPattern)
                        {
                            brepgraph::Vec3 dir1 = {s.dir1[0], s.dir1[1], s.dir1[2]};
                            brepgraph::Vec3 dir2 = {s.dir2[0], s.dir2[1], s.dir2[2]};
                            int d1 = cg->AddConst(dir1, "dir1");
                            int c1 = cg->AddConst((int)s.count1, "count1");
                            int sp1= cg->AddConst(s.spacing1,    "spacing1");
                            int d2 = cg->AddConst(dir2, "dir2");
                            int c2 = cg->AddConst((int)s.count2, "count2");
                            int sp2= cg->AddConst(s.spacing2,    "spacing2");
                            cur = cg->AddOp("linear_pattern",
                                             {cur, d1, c1, sp1, d2, c2, sp2},
                                             {}, step_name);
                        }
                        else  // CircularPattern
                        {
                            brepgraph::Vec3 origin = {s.axis_origin[0],
                                                     s.axis_origin[1],
                                                     s.axis_origin[2]};
                            brepgraph::Vec3 axis   = {s.axis_dir[0],
                                                     s.axis_dir[1],
                                                     s.axis_dir[2]};
                            int o = cg->AddConst(origin, "axis_origin");
                            int a = cg->AddConst(axis,   "axis_dir");
                            int c = cg->AddConst((int)s.count,    "count");
                            int t = cg->AddConst(s.total_angle,   "total_angle");
                            cur = cg->AddOp("circular_pattern",
                                             {cur, o, a, c, t},
                                             {}, step_name);
                        }
                    }
                    return cur;
                };

                auto origs = ResolveOriginals(feat, feature_tools);

                if (origs.empty())
                {
                    if (last_node < 0)
                    {
                        step_ok = false;
                        return;
                    }
                    node = apply_chain(last_node, feat.name);
                }
                else
                {
                    if (last_node < 0)
                    {
                        step_ok = false;
                        return;
                    }
                    int body = last_node;
                    for (size_t fi = 0; fi < origs.size(); ++fi)
                    {
                        std::string tag = feat.name + ":orig"
                                        + std::to_string(fi);
                        int pat = apply_chain(origs[fi].tool_node, tag);
                        const char* op_name =
                            (origs[fi].op_kind == 'c') ? "cut" : "fuse";
                        body = cg->AddOp(op_name, {body, pat}, {},
                                          tag + ":combine");
                    }
                    node = body;
                }
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

            // Dump-bodies hook: when CAX_DUMP_BODIES is set in the
            // environment, write each feature's running body to a
            // .brp file in the current working directory. Filename
            // "cax_<id>_<name>.brp"; pair with FreeCAD's PartShapeN
            // entries inside the source .FCStd zip via the brp_diff
            // tool (tools/brp_diff/) to localise where cax replay
            // diverges from FreeCAD's authored geometry. Off by
            // default so production loads don't litter CWD.
            //if (const char* dump = std::getenv("CAX_DUMP_BODIES");
            //    dump && dump[0] != '\0' && dump[0] != '0')
            {
                auto val = cg->Eval(node);
                if (auto* sv = std::get_if<brepgraph::ShapeVal>(&val);
                    sv && sv->shape && !sv->shape->GetShape().IsNull())
                {
                    char path[256];
                    std::snprintf(path, sizeof(path),
                                  "cax_%u_%s.brp",
                                  feat.id, feat.name.c_str());
                    BRepTools::Write(sv->shape->GetShape(), path);
                }
            }
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
        auto* sv = std::get_if<brepgraph::ShapeVal>(&val);
        if (sv && sv->shape) {
            out.shape = sv->shape;
        } else if (out.err_msg.empty()) {
            // The graph was assembled OK but evaluation collapsed to
            // an empty Val somewhere downstream -- typically a
            // sketch_face whose wire didn't close, or a boolean whose
            // operand was already empty. Pin the failure here so the
            // caller doesn't get a silent nullptr shape with ok=true.
            out.err_msg = "shape evaluation produced no result";
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
            if (auto* sv = std::get_if<brepgraph::ShapeVal>(&v))
            {
                rb.ref->resolved_uid        = sv->tag;
                rb.ref->resolved_topo_index = 0;
            }
        }
    }

    out.calc_graph = cg;
    out.ok         = true;
    return true;
}

} // namespace cadapp
