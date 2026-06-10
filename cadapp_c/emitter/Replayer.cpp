#include "cadapp_c/emitter/Replayer.h"
#include "cadapp_c/ops/sketch_ops.h"
#include "cadapp_c/ops/resolve_ops.h"

#include "brepgraph_c/computation/CalcGraph.h"
#include "brepgraph_c/history/TopoNaming.h"
#include "brepdb_c/VersionTree.h"
#include "brepkit_c/TopoShape.h"

#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepTools.hxx>
#include <TopAbs.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

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
    // false only for an up-to-X extrude bound to the running body: its
    // solid depends on that body's geometry, so it is NOT a rigid-motion
    // image of itself and a pattern must NOT merely transform it.
    bool equivariant = true;
};

// A reference-based dressup (chamfer / fillet), recorded so a pattern
// over the dressed seed can REPLICATE it: re-emit the same op on each
// instance body with the picked anchors moved by the instance
// transform. The dressup-side sibling of FeatureToolInfo -- together
// they form a seed's "contribution" that a pattern multiplies.
struct FeatureDressupInfo
{
    FeatType               kind = FeatType::Chamfer;  // Chamfer / Fillet
    double                 size = 0.0;   // distance1 (chamfer) | radius (fillet)
    std::vector<TopoRefIR> refs;         // picked edges/faces (world anchors)
    uint32_t               base_feat_id = 0;  // the feature it dressed
};

// Walk FeatureIR::input_feature_ids for Role::Tool entries and
// resolve each to the original feature's FeatureToolInfo record
// (tool_node + base_node + op_kind, populated at the original's
// own replay time). Pattern handlers use these to "multiply the
// tool, then recombine with the base" instead of patterning the
// whole running body.
//
// Returns empty when no Tool roles are present, which signals the
// caller to fall back to "apply pattern to the base body"
// (base_node from ResolveBaseNode). Originals whose id isn't in
// feature_tools (upstream not yet replayed / failed) are silently
// dropped; the surrounding pattern still runs on the remainder.
std::vector<FeatureToolInfo> ResolveOriginals(
    const FeatureIR&                                 feat,
    const std::map<uint32_t, FeatureToolInfo>&       feature_tools)
{
    std::vector<FeatureToolInfo> out;
    for (size_t i = 0; i < feat.input_feature_ids.size(); ++i)
    {
        InputRole role = (i < feat.input_roles.size())
                            ? feat.input_roles[i]
                            : InputRole::Base;
        if (role != InputRole::Tool) continue;
        uint32_t id = feat.input_feature_ids[i];
        auto t_it = feature_tools.find(id);
        if (t_it == feature_tools.end()) continue;
        out.push_back(t_it->second);
    }
    return out;
}

// Compute the base body node this feature visit should consume.
// Walks the typed (FeatureIR::input_feature_ids, input_roles)
// parallel vectors and picks the first entry with Role::Base:
//
//   no Base entry         -- standalone primitive, sketch, or a
//                            feature whose role inputs are all
//                            non-base (Boolean with only Operand
//                            roles). Returns -1; a base-consuming
//                            handler errors out cleanly.
//   Base id == 0          -- explicit "no predecessor" (body
//                            root). Returns -1 so the body's
//                            first 3D feature never fuses with
//                            the previous body's tip.
//   Base id == 0xFFFFFFFF -- unresolved link. Returns -1.
//   Base id >= 1          -- predecessor is that feature; returns
//                            feature_nodes[id], or -1 if upstream
//                            failed.
//
// Since P3.2 there is no running last_node cursor: every feature
// that consumes a base must declare it explicitly in the IR.
int ResolveBaseNode(const FeatureIR&                feat,
                    const std::map<uint32_t, int>&  feature_nodes)
{
    const size_t n = feat.input_feature_ids.size();
    for (size_t i = 0; i < n; ++i)
    {
        // Defensive: input_roles is enforced parallel by the Reader
        // (PushInput) but a malformed store could leave it shorter.
        // Missing role defaults to Base (FeatureStore::DecodeExt
        // pads the same way), so the loop still picks up the entry.
        InputRole role = (i < feat.input_roles.size())
                            ? feat.input_roles[i]
                            : InputRole::Base;
        if (role != InputRole::Base) {
            continue;
        }
        uint32_t iid = feat.input_feature_ids[i];
        if (iid == 0u || iid == 0xFFFFFFFFu) {
            return -1;
        }
        auto fit = feature_nodes.find(iid);
        return fit != feature_nodes.end() ? fit->second : -1;
    }
    return -1;
}

// Pattern / Mirror / MultiTransform target resolution. Folds two
// link sources into one struct:
//
//   originals  -- PartDesign Originals list, resolved from
//                 input_feature_ids entries with Role::Tool. One
//                 entry per original feature with (tool, base,
//                 op_kind) ready for "multiply the tool, recombine
//                 with the base".
//
//   body_target -- Draft Array's Base link, resolved by the Reader
//                  to a feature id and pushed into
//                  FeatureIR::input_feature_ids with
//                  Role::PatternTarget. -1 when absent. Only
//                  meaningful when originals is empty -- a
//                  PartDesign pattern with Originals never carries
//                  a Draft-style single-target link.
//
// Both empty -> caller falls back to its base_node (the body
// chain pred resolved from input_feature_ids).
struct PatternInputs
{
    std::vector<FeatureToolInfo> originals;
    int                          body_target = -1;
};

PatternInputs ResolvePatternInputs(
    const FeatureIR&                                 feat,
    const std::map<uint32_t, FeatureToolInfo>&       feature_tools,
    const std::map<uint32_t, int>&                   feature_nodes)
{
    PatternInputs out;
    out.originals = ResolveOriginals(feat, feature_tools);
    if (!out.originals.empty()) {
        return out;
    }

    // First Role::PatternTarget entry resolves to the target body.
    // Only Draft polar Arrays emit this today; the loop tolerates
    // extra inputs of other roles (e.g. a future body-owned Draft
    // Array could carry both Role::Base for its body chain and
    // Role::PatternTarget for its Base link).
    for (size_t i = 0; i < feat.input_feature_ids.size(); ++i)
    {
        InputRole role = (i < feat.input_roles.size())
                            ? feat.input_roles[i]
                            : InputRole::Base;
        if (role != InputRole::PatternTarget) {
            continue;
        }
        uint32_t tid = feat.input_feature_ids[i];
        if (tid == 0u || tid == 0xFFFFFFFFu) {
            break;
        }
        auto fit = feature_nodes.find(tid);
        if (fit != feature_nodes.end()) {
            out.body_target = fit->second;
        }
        break;
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

// ============================================================
// Generalized pattern lowering (shared, reusable)
//
// A feature pattern is the rigid-motion image of the seed's
// construction. Tool features (extrude/revolve/sweep/loft/prim) are
// rigid-motion EQUIVARIANT, so an instance's tool == transform(seed
// tool solid) -- no need to re-run sketch->extrude. Reference dressups
// (chamfer/fillet) are NOT solids: an instance's dressup == the same op
// re-resolved on that instance's body with the picked anchors moved by
// the instance transform. So a pattern replays the seed group's
// CONTRIBUTIONS (its tools AND the dressups on them) once per non-seed
// instance. One routine drives linear and circular, any tool, any
// dressup -- no per-feature-type special cases. Kept self-contained
// (takes a resolved contribution list + a transform) so the editable
// Pattern node can reuse it when it re-lowers on edit (Stage 2).
// ============================================================

// One rigid instance transform: a translation (linear pattern) or a
// rotation about an axis (circular). Carries the calc-op params
// (EmitTransform emits a translate/rotate op) AND drives the closed-
// form anchor map (TransformRef) -- the two MUST agree.
struct InstanceXform
{
    bool            is_rotation = false;
    brepgraph::Vec3 offset      = {};   // translation
    brepgraph::Vec3 axis_origin = {};   // rotation
    brepgraph::Vec3 axis_dir    = {};
    double          angle       = 0.0;
    // Optional small translation applied AFTER the rotation (zero vec = none).
    // Used by circular patterns to push each copy a hair into the body so a
    // coplanar thin-boss base becomes a clean volumetric overlap that fuses.
    brepgraph::Vec3 post_offset = {};
};

// Move a TopoRefIR's geometry by X: the anchor `point` by the full
// motion; `normal` + `extra_samples` by the rotational part only (a
// translation leaves directions unchanged -- which is why the linear
// case only had to move `point`). Closed form; no OCCT dependency.
void TransformRef(TopoRefIR& r, const InstanceXform& X)
{
    if (!X.is_rotation) {
        for (int k = 0; k < 3; ++k) r.point[k] += X.offset[k];
        for (int s = 0; s < (int)r.extra_sample_count && s < 4; ++s)
            for (int k = 0; k < 3; ++k) r.extra_samples[s][k] += X.offset[k];
        return;
    }
    double dl = std::sqrt(X.axis_dir[0]*X.axis_dir[0] +
                          X.axis_dir[1]*X.axis_dir[1] +
                          X.axis_dir[2]*X.axis_dir[2]);
    if (dl < 1e-15) return;
    const double dx = X.axis_dir[0]/dl, dy = X.axis_dir[1]/dl, dz = X.axis_dir[2]/dl;
    const double c = std::cos(X.angle), s = std::sin(X.angle);
    // Rodrigues rotation of v about the unit axis (dx,dy,dz) by angle.
    auto rot = [&](double v[3], bool about_origin) {
        double px = v[0], py = v[1], pz = v[2];
        if (about_origin) { px -= X.axis_origin[0]; py -= X.axis_origin[1]; pz -= X.axis_origin[2]; }
        const double dot = dx*px + dy*py + dz*pz;
        const double crx = dy*pz - dz*py, cry = dz*px - dx*pz, crz = dx*py - dy*px;
        double rx = px*c + crx*s + dx*dot*(1.0 - c);
        double ry = py*c + cry*s + dy*dot*(1.0 - c);
        double rz = pz*c + crz*s + dz*dot*(1.0 - c);
        if (about_origin) { rx += X.axis_origin[0]; ry += X.axis_origin[1]; rz += X.axis_origin[2]; }
        v[0] = rx; v[1] = ry; v[2] = rz;
    };
    rot(r.point, true);
    rot(r.normal, false);
    for (int s2 = 0; s2 < (int)r.extra_sample_count && s2 < 4; ++s2)
        rot(r.extra_samples[s2], true);
    // Post-rotation translation (the penetration offset) moves anchor points
    // but not directions, exactly like the linear case.
    for (int k = 0; k < 3; ++k) r.point[k] += X.post_offset[k];
    for (int s2 = 0; s2 < (int)r.extra_sample_count && s2 < 4; ++s2)
        for (int k = 0; k < 3; ++k) r.extra_samples[s2][k] += X.post_offset[k];
}

// Emit a calc node that rigid-transforms `node` by X (the translate /
// rotate ops -- the same motion TransformRef applies to anchors).
int EmitTransform(brepgraph::CalcGraph& cg, int node,
                  const InstanceXform& X, const std::string& desc)
{
    if (X.is_rotation) {
        int o = cg.AddConst(X.axis_origin, "axis_origin");
        int a = cg.AddConst(X.axis_dir,    "axis_dir");
        int g = cg.AddConst(X.angle,       "angle");
        int rotated = cg.AddOp("rotate", {node, o, a, g}, {}, desc);
        if (X.post_offset[0] != 0.0 || X.post_offset[1] != 0.0 ||
            X.post_offset[2] != 0.0) {
            int po = cg.AddConst(X.post_offset, "post_offset");
            return cg.AddOp("translate", {rotated, po}, {}, desc);
        }
        return rotated;
    }
    int off = cg.AddConst(X.offset, "offset");
    return cg.AddOp("translate", {node, off}, {}, desc);
}

// One seed contribution: a Tool (transform the solid + combine by
// op_kind) or a Dressup (re-resolve the moved anchors + re-apply).
struct Contribution
{
    bool     is_tool     = true;
    uint32_t feat_id     = 0;          // creation-order sort key
    int      tool_node   = -1;         // tool
    char     op_kind     = 'f';
    bool     equivariant = true;
    FeatType dress_kind  = FeatType::Chamfer;   // dressup
    double   dress_size  = 0.0;
    std::vector<TopoRefIR> dress_refs;
};

// Collect a pattern's seed group: its Tool-role seeds + every dressup
// that (transitively) operated on one of them, ordered by creation
// (feat_id == creation order for ZW3D). The list a pattern replays
// under each instance transform.
std::vector<Contribution> AssembleContributions(
    const FeatureIR&                              pat,
    const std::map<uint32_t, FeatureToolInfo>&    feature_tools,
    const std::map<uint32_t, FeatureDressupInfo>& feature_dressups)
{
    std::set<uint32_t> group;
    for (size_t i = 0; i < pat.input_feature_ids.size(); ++i) {
        InputRole role = (i < pat.input_roles.size())
                            ? pat.input_roles[i] : InputRole::Base;
        if (role == InputRole::Tool) group.insert(pat.input_feature_ids[i]);
    }
    // Pull in dressups whose base is in the group (transitive: a
    // chamfer on a filleted seed, a dressup on a dressup).
    bool grew = true;
    while (grew) {
        grew = false;
        for (const auto& kv : feature_dressups) {
            if (!group.count(kv.first) && group.count(kv.second.base_feat_id)) {
                group.insert(kv.first);
                grew = true;
            }
        }
    }
    std::vector<Contribution> out;
    out.reserve(group.size());
    for (uint32_t id : group) {
        auto ti = feature_tools.find(id);
        if (ti != feature_tools.end()) {
            Contribution c;
            c.is_tool = true; c.feat_id = id;
            c.tool_node = ti->second.tool_node;
            c.op_kind = ti->second.op_kind;
            c.equivariant = ti->second.equivariant;
            out.push_back(std::move(c));
            continue;
        }
        auto di = feature_dressups.find(id);
        if (di != feature_dressups.end()) {
            Contribution c;
            c.is_tool = false; c.feat_id = id;
            c.dress_kind = di->second.kind;
            c.dress_size = di->second.size;
            c.dress_refs = di->second.refs;
            out.push_back(std::move(c));
        }
    }
    std::sort(out.begin(), out.end(),
              [](const Contribution& a, const Contribution& b) {
                  return a.feat_id < b.feat_id;
              });
    return out;
}

// Replay one pattern instance onto `acc`: transform each tool by X and
// combine by op_kind; re-resolve + re-apply each dressup on the
// post-combine instance body with X-moved anchors.
int ApplyPatternInstance(brepgraph::CalcGraph& cg, int acc,
                         const std::vector<Contribution>& contribs,
                         const InstanceXform& X, double topo_tol,
                         const std::string& tag)
{
    int body = acc;
    for (const Contribution& c : contribs) {
        if (c.is_tool) {
            int t = EmitTransform(cg, c.tool_node, X, tag);
            const char* op = (c.op_kind == 'c') ? "cut" : "fuse";
            body = cg.AddOp(op, {body, t}, {}, tag);
        } else {
            double dtol = std::max(topo_tol, 5.0 * c.dress_size);
            std::vector<int> edge_nodes;
            edge_nodes.reserve(c.dress_refs.size());
            for (TopoRefIR ref : c.dress_refs) {
                TransformRef(ref, X);
                ref.resolved_uid = 0;
                ref.resolved_topo_index = 0;
                const char* rop = (ref.kind == TopoRefIR::Kind::Face)
                                    ? "resolve_face_ref" : "resolve_edge_ref";
                edge_nodes.push_back(
                    AddResolveRefNode(cg, rop, body, ref, dtol, tag + ":edge"));
            }
            if (c.dress_kind == FeatType::Fillet) {
                int r = cg.AddConst(c.dress_size, "radius");
                body = cg.AddOp("fillet", {body, r}, edge_nodes, tag);
            } else {
                int d = cg.AddConst(c.dress_size, "dist");
                body = cg.AddOp("chamfer", {body, d}, edge_nodes, tag);
            }
        }
    }
    return body;
}

// ============================================================
// Pattern-instance CLUSTER lowering (perf optimization)
//
// The per-instance path threads `acc` (the running body) through one
// boolean PER pattern copy: acc = fuse(acc, copy_i). Each fuse re-
// intersects the WHOLE, growing body -- on R2900_50 the body balloons
// past 1000 faces and the late fuses cost up to ~2s EACH (profiled via
// BREPKIT_BOP_PROF). Because fuse / cut are associative + commutative
// and the instance copies occupy mostly DISJOINT regions, the identical
// union can be built by first fusing the small copies together into a
// sub-body (each fuse is cheap -- small disjoint shapes), then combining
// that sub-body onto the base ONCE. N booleans-against-the-big-body
// become N cheap copy-fuses + 1 big combine.
//
// Only applies when every contribution is a TOOL of a single op_kind: a
// dressup (chamfer / fillet) must re-resolve its anchors on the post-
// combine body, which the cluster does not expose mid-build, and mixing
// additive + subtractive tools is order-sensitive when copies overlap.
// Those cases return -1 -> caller keeps the (correct, slower) per-
// instance path. Mirrors the already-shipped linear-equivariant n-ary
// path, but built from the SAME explicit per-copy transforms the per-
// instance loop uses, so it carries each copy's penetration post_offset
// and is geometrically identical (only the fuse association changes).
//
// Gated by BREPKIT_PATTERN_CLUSTER (default ON; =0 restores per-instance
// for A/B + regression bisect). `Xs` are the NON-seed instance
// transforms (the seed copy is already combined into `base`).
static bool pattern_cluster_on()
{
    static const bool on = [] {
        const char* e = std::getenv("BREPKIT_PATTERN_CLUSTER");
        return !(e && e[0] == '0');
    }();
    return on;
}

// Diagnostic: which pattern-lowering branch fires + whether clustering is
// reached. Reuses BREPKIT_BOP_PROF so one env turns on the full picture.
static bool pat_log_on()
{
    static const bool on = [] {
        const char* e = std::getenv("BREPKIT_BOP_PROF");
        return e && e[0] && e[0] != '0';
    }();
    return on;
}
#define PAT_LOG(...) do { if (pat_log_on()) { std::fprintf(stderr, __VA_ARGS__); std::fflush(stderr); } } while (0)

int ApplyPatternClustered(brepgraph::CalcGraph&            cg,
                          int                              base,
                          const std::vector<Contribution>& contribs,
                          const std::vector<InstanceXform>& Xs,
                          const std::string&               tag)
{
    if (contribs.empty() || Xs.empty()) {
        PAT_LOG("[pat] cluster SKIP empty contribs=%zu Xs=%zu (%s)\n",
                contribs.size(), Xs.size(), tag.c_str());
        return -1;
    }
    char kind = 0;
    for (const Contribution& c : contribs) {
        if (!c.is_tool) {
            PAT_LOG("[pat] cluster SKIP dressup-present (%s)\n", tag.c_str());
            return -1;                              // dressup -> per-instance
        }
        char k = (c.op_kind == 'c') ? 'c' : 'f';
        if (kind == 0)      kind = k;
        else if (kind != k) {
            PAT_LOG("[pat] cluster SKIP mixed-op_kind (%s)\n", tag.c_str());
            return -1;                              // mixed add/sub -> per-instance
        }
    }
    PAT_LOG("[pat] cluster APPLIED contribs=%zu Xs=%zu kind=%c (%s)\n",
            contribs.size(), Xs.size(), kind, tag.c_str());
    // Fuse every (instance copy, tool) together into one sub-body. cut-
    // tools are UNIONED here too, then applied as a single cut(base, union)
    // below -- equivalent to sequential cuts for disjoint instances.
    int cluster = -1;
    for (const InstanceXform& X : Xs) {
        for (const Contribution& c : contribs) {
            int t = EmitTransform(cg, c.tool_node, X, tag);
            cluster = (cluster < 0)
                        ? t
                        : cg.AddOp("fuse", {cluster, t}, {}, tag + ":cluster");
        }
    }
    if (cluster < 0) return -1;
    const char* op = (kind == 'c') ? "cut" : "fuse";
    return cg.AddOp(op, {base, cluster}, {}, tag + ":combine");
}

// Batch a pattern's per-instance DRESSUPS (fillet / chamfer) into ONE op
// per dressup contribution. The per-instance path applies fillet(body,
// edges_i) once PER instance; each call rebuilds the WHOLE (large) body in
// OCCT's BRepFilletAPI even though it only touches that instance's handful
// of edges -- measured on R2900_100 as 7 instances x ~5.5s = ~38s, the
// single biggest cost there (bigger than all booleans on some models).
// Pattern instances are DISJOINT and share the dressup's radius, so
// resolving EVERY instance's edges against the same pre-dressup body and
// filleting them ALL in one op is geometrically equivalent (disjoint edge
// sets round independently) while rebuilding the body ONCE. The
// resolve_*_ref ops still resolve per-instance geometry (each edge moved by
// its X), just against a single shared body node. Mirrors the boolean
// ApplyPatternClustered; same BREPKIT_PATTERN_CLUSTER gate. `Xs` are ALL
// instance transforms the per-instance loop would have visited (the
// equivariant Phase-2 includes the seed i=0).
int ApplyPatternDressupsBatched(brepgraph::CalcGraph&            cg,
                                int                              body,
                                const std::vector<Contribution>& dressups,
                                const std::vector<InstanceXform>& Xs,
                                double                           topo_tol,
                                const std::string&               tag)
{
    for (const Contribution& d : dressups) {
        if (d.is_tool) continue;
        const double dtol = std::max(topo_tol, 5.0 * d.dress_size);
        std::vector<int> edge_nodes;
        edge_nodes.reserve(Xs.size() * d.dress_refs.size());
        for (const InstanceXform& X : Xs) {
            for (TopoRefIR ref : d.dress_refs) {
                TransformRef(ref, X);
                ref.resolved_uid        = 0;
                ref.resolved_topo_index = 0;
                const char* rop = (ref.kind == TopoRefIR::Kind::Face)
                                    ? "resolve_face_ref" : "resolve_edge_ref";
                edge_nodes.push_back(
                    AddResolveRefNode(cg, rop, body, ref, dtol, tag + ":edge"));
            }
        }
        if (edge_nodes.empty()) continue;
        if (d.dress_kind == FeatType::Fillet) {
            int r = cg.AddConst(d.dress_size, "radius");
            body = cg.AddOp("fillet", {body, r}, edge_nodes, tag);
        } else {
            int dist = cg.AddConst(d.dress_size, "dist");
            body = cg.AddOp("chamfer", {body, dist}, edge_nodes, tag);
        }
    }
    return body;
}

// Unit vector from a raw double[3] direction (matches the normalization
// the linear/circular pattern ops apply internally).
brepgraph::Vec3 NormalizedDir(const double v[3])
{
    double l = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (l < 1e-15) return {v[0], v[1], v[2]};
    return {v[0]/l, v[1]/l, v[2]/l};
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
//   2. Fuse with the base body shape for PartDesign::Additive*, or
//      cut from it for PartDesign::Subtractive*. Plain Part::*
//      primitives (no freecad_type tag) stand alone -- they don't
//      consume base_node, which is -1 for them anyway.
//
// Returns the final node id (possibly the same as `prim_node`) and
// whether the caller should treat the step as ok. step_ok becomes
// false only when a subtractive op has no base shape to cut from.
//
// tool_info_out captures the placed primitive node (the "tool") and
// the base / op_kind it combined with. A later PolarPattern with
// Originals = [this feature] uses these to multiply the tool only,
// then re-apply cut / fuse against the recorded base.
int FinalizePrimitiveNode(brepgraph::CalcGraph& cg,
                          const FeatureIR&     feat,
                          int                  prim_node,
                          int                  base_node,
                          ReplayResult&        out,
                          bool&                step_ok,
                          FeatureToolInfo*     tool_info_out = nullptr)
{
    // Note: base_node has already been resolved by ResolveBaseNode
    // from the IR's input_feature_ids field. {0} (body root) and
    // missing inputs both yield -1, so a primitive at the root of
    // a body never fuses with the previous body's tip.
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
        tool_info_out->base_node = base_node;
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
        if (base_node < 0)
        {
            step_ok     = false;
            out.err_msg = "Subtractive primitive without base shape: " + feat.name;
            return cur;
        }
        if (tool_info_out) tool_info_out->op_kind = 'c';
        return cg.AddOp("cut", {base_node, cur}, {}, feat.name);
    }
    if (is_add)
    {
        if (base_node < 0) {
            return cur;
        }
        if (tool_info_out) tool_info_out->op_kind = 'f';
        return cg.AddOp("fuse", {base_node, cur}, {}, feat.name);
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

    if (pat_log_on()) {
        int npat = 0;
        for (const auto& f : doc.features) {
            std::visit([&](const auto& p) {
                using T = std::decay_t<decltype(p)>;
                if constexpr (std::is_same_v<T, FeatPayloadLinearPattern> ||
                              std::is_same_v<T, FeatPayloadCircularPattern> ||
                              std::is_same_v<T, FeatPayloadMirror> ||
                              std::is_same_v<T, FeatPayloadMultiTransform>) { ++npat; }
            }, f.data);
        }
        std::fprintf(stderr,
            "[replay] ENTER analyze_only=%d doc_features=%zu pattern_feats=%d\n",
            (int)opt.analyze_only, doc.features.size(), npat);
        std::fflush(stderr);
    }

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

    // feature_id -> its reference-based dressup (chamfer/fillet) record,
    // so a pattern over the dressed seed can replicate it per instance.
    std::map<uint32_t, FeatureDressupInfo> feature_dressups;

    // feature_id -> calc-graph node id of the body shape produced by
    // that feature. This is the central DAG store the Replayer reads
    // from: every feature looks up its inputs through this map
    // (FeatureIR::input_feature_ids[i] -> feature_nodes[id]), and
    // Booleans / standalone operators fold their operand shapes by
    // id rather than by emission order. Replaces the running
    // last_node cursor that earlier revisions used as an implicit
    // body chain pointer (P3.2 of the multi-last_node refactor).
    std::map<uint32_t, int> feature_nodes;

    // Output candidates collected in document declaration order. Each
    // entry pins the feat_id and calc-graph node of one potential
    // top-level result -- a PartDesign::Body tip (collapsed to a single
    // candidate per body via body_candidate_idx) or a standalone
    // Part::* feature. consumed_feat_ids tracks every operand_feature_id
    // referenced by a downstream boolean (Part::Cut / Fuse / Common /
    // MultiFuse / MultiCommon); candidates whose feat_id appears in that
    // set are dropped at emission time so a Common(Sphere001, Cut) that
    // already absorbs the two Body tips doesn't have them re-emitted as
    // free-standing solids alongside it. Without this filter Page_031
    // (Body+Body001 feeding a downstream Cut+Common) regressed from
    // 1 solid to 2 solids when multi-body compounding was first added.
    struct OutputCandidate
    {
        uint32_t feat_id;
        int      node;
    };
    std::vector<OutputCandidate>      output_candidates;
    std::map<std::string, size_t>     body_candidate_idx;
    std::set<uint32_t>                consumed_feat_ids;

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

        // base_node is the body shape this feature consumes, resolved
        // from FeatureIR::input_feature_ids via ResolveBaseNode. -1
        // means "no base" (sketch, body root, standalone primitive).
        // It's a local-per-iteration value -- there's no running
        // cursor that survives across features. Every handler that
        // needs a base reads base_node; handlers that don't (Sketch,
        // Boolean reading Role::Operand inputs, Pattern reading its
        // own Originals tools) ignore it.
        int base_node = ResolveBaseNode(feat, feature_nodes);

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
                // 3D body, so node stays -1 and feature_nodes is not
                // updated -- sketches are not addressable as bases.
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
                node = FinalizePrimitiveNode(*cg, feat, prim, base_node, out, step_ok, &ti);
                feature_tools[feat.id] = ti;
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimCylinder>)
            {
                int r = cg->AddConst(p.radius, "radius");
                int h = cg->AddConst(p.height, "height");
                int prim = cg->AddOp("cylinder", {r, h}, {}, feat.name);
                FeatureToolInfo ti;
                node = FinalizePrimitiveNode(*cg, feat, prim, base_node, out, step_ok, &ti);
                feature_tools[feat.id] = ti;
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimCone>)
            {
                int r1 = cg->AddConst(p.radius1, "radius1");
                int r2 = cg->AddConst(p.radius2, "radius2");
                int h  = cg->AddConst(p.height,  "height");
                int prim = cg->AddOp("cone", {r1, r2, h}, {}, feat.name);
                FeatureToolInfo ti;
                node = FinalizePrimitiveNode(*cg, feat, prim, base_node, out, step_ok, &ti);
                feature_tools[feat.id] = ti;
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimSphere>)
            {
                int r = cg->AddConst(p.radius, "radius");
                int prim = cg->AddOp("sphere", {r}, {}, feat.name);
                FeatureToolInfo ti;
                node = FinalizePrimitiveNode(*cg, feat, prim, base_node, out, step_ok, &ti);
                feature_tools[feat.id] = ti;
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimTorus>)
            {
                int r1 = cg->AddConst(p.major_radius, "major_radius");
                int r2 = cg->AddConst(p.minor_radius, "minor_radius");
                int prim = cg->AddOp("torus", {r1, r2}, {}, feat.name);
                FeatureToolInfo ti;
                node = FinalizePrimitiveNode(*cg, feat, prim, base_node, out, step_ok, &ti);
                feature_tools[feat.id] = ti;
            }
            else if constexpr (std::is_same_v<T, FeatPayloadPrimEllipsoid>)
            {
                int r1 = cg->AddConst(p.radius1, "radius1");
                int r2 = cg->AddConst(p.radius2, "radius2");
                int r3 = cg->AddConst(p.radius3, "radius3");
                int prim = cg->AddOp("ellipsoid", {r1, r2, r3}, {}, feat.name);
                FeatureToolInfo ti;
                node = FinalizePrimitiveNode(*cg, feat, prim, base_node, out, step_ok, &ti);
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
                    int ref_n = base_node >= 0
                        ? base_node
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
                ti.base_node = base_node;
                // Equivariant unless an up-to-X end type ties the solid to
                // the running body (then a pattern can't merely transform it).
                ti.equivariant = (p.end_type  == ExtrudeEndType::Blind &&
                                  p.end_type2 == ExtrudeEndType::Blind);

                if (feat.type == FeatType::BossExtrude)
                {
                    if (base_node < 0)
                    {
                        node = tool_n;
                        ti.op_kind = '0';
                    }
                    else
                    {
                        node = cg->AddOp("fuse", {base_node, tool_n},
                                          {}, feat.name);
                        ti.op_kind = 'f';
                    }
                }
                else
                {
                    if (base_node < 0)
                    {
                        step_ok     = false;
                        out.err_msg = "CutExtrude without base shape: " + feat.name;
                        return;
                    }
                    node = cg->AddOp("cut", {base_node, tool_n},
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

                // Midplane: FreeCAD sweeps the profile symmetrically
                // about the sketch plane (-angle/2 .. +angle/2). BRepPrim-
                // API_MakeRevol only sweeps one-sided from the profile's
                // current position, so pre-rotate the profile by
                // -sign*angle/2 around the axis; the subsequent
                // sign*angle sweep then lands the profile at +angle/2,
                // matching the symmetric range. Reader stashes the
                // flag in ext_params["midplane"]=1 because
                // FeatPayloadRevolve doesn't carry a bool field yet.
                // Don't update sketch_face_nodes -- the rotated face
                // is local to this revolve, and another feature
                // reusing the same sketch must see the unrotated
                // original. Full 2pi revolves are already symmetric
                // (sweep wraps onto itself), so the pre-rotation is
                // skipped -- still geometrically correct, but it
                // would waste a graph node.
                bool midplane = ExtParam(feat, "midplane", 0.0) != 0.0;
                int rev_face_n = face_n;
                if (midplane && !is_full)
                {
                    int half_a_n = cg->AddConst(-sign * 0.5 * p.angle,
                                                "midplane_pre_angle");
                    rev_face_n = cg->AddOp(
                        "rotate",
                        {face_n, o_n, d_n, half_a_n},
                        {}, feat.name + ":midplane_pre_rot");
                }

                int tool_n = cg->AddOp("revolve",
                                        {rev_face_n, o_n, d_n, a_n, f_n},
                                        {}, feat.name);

                FeatureToolInfo ti;
                ti.tool_node = tool_n;
                ti.base_node = base_node;

                if (feat.type == FeatType::BossRevolve)
                {
                    if (base_node < 0)
                    {
                        node = tool_n;
                        ti.op_kind = '0';
                    }
                    else
                    {
                        node = cg->AddOp("fuse", {base_node, tool_n},
                                          {}, feat.name);
                        ti.op_kind = 'f';
                    }
                }
                else
                {
                    if (base_node < 0)
                    {
                        step_ok     = false;
                        out.err_msg = "CutRevolve without base shape: " + feat.name;
                        return;
                    }
                    node = cg->AddOp("cut", {base_node, tool_n},
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

                // PartDesign Pipe always builds a capped solid; a Part
                // workbench Part::Sweep honors its Solid flag (default
                // false -> open tube shell), stashed as sweep_solid.
                // brepkit's Sweep (BRepOffsetAPI_MakePipe) derives the
                // result topology from the profile shape, NOT the is_solid
                // arg: a FACE profile yields a capped solid, a WIRE
                // profile yields an open shell. Pick the profile node
                // accordingly -- a face for a solid sweep, the boundary
                // wire for a shell sweep.
                bool is_solid = true;
                auto solid_it = feat.ext_params.find("sweep_solid");
                if (solid_it != feat.ext_params.end()) {
                    is_solid = (solid_it->second != 0.0);
                }

                int profile_n;
                if (is_solid)
                {
                    auto fit = sketch_face_nodes.find(p.profile_sketch_id);
                    if (fit != sketch_face_nodes.end()) {
                        profile_n = fit->second;
                    } else {
                        profile_n = AddSketchFaceNode(*cg, *profile_sk);
                        sketch_face_nodes[p.profile_sketch_id] = profile_n;
                    }
                }
                else
                {
                    profile_n = AddSketchWireNode(*cg, *profile_sk);
                }

                int wire_n;
                auto kit = feat.ext_strings.find("spine_kind");
                if (kit != feat.ext_strings.end() && kit->second == "helix")
                {
                    // Parametric helix spine (FreeCAD Part::Helix behind
                    // a PartDesign::FeatureBase). The reader couldn't
                    // map the spine to a sketch, so it stashed the coil
                    // params; rebuild the spine wire via the helix_wire
                    // op rather than from a sketch profile.
                    auto getp = [&](const char* k) -> double {
                        auto it = feat.ext_params.find(k);
                        return (it != feat.ext_params.end()) ? it->second : 0.0;
                    };
                    int pitch_n  = cg->AddConst(getp("helix_pitch"),  "helix_pitch");
                    int height_n = cg->AddConst(getp("helix_height"), "helix_height");
                    int radius_n = cg->AddConst(getp("helix_radius"), "helix_radius");
                    int angle_n  = cg->AddConst(getp("helix_angle"),  "helix_cone_angle");
                    int lh_n     = cg->AddConst(getp("helix_left_handed") != 0.0,
                                                 "helix_left_handed");
                    wire_n = cg->AddOp("helix_wire",
                                       {pitch_n, height_n, radius_n, angle_n, lh_n},
                                       {}, feat.name + ":helix");

                    // A Part-workbench Part::Sweep links a STANDALONE
                    // Part::Helix whose own Placement orients the coil in
                    // world space; the reader stashed it under
                    // helix_place_* (PartDesign's FeatureBase clone path
                    // leaves these absent -- it builds in the local +Z
                    // frame). Apply rotate-then-translate to the spine
                    // wire so the swept profile follows the placed coil,
                    // matching FreeCAD's authored shape.
                    if (feat.ext_params.count("helix_place_angle"))
                    {
                        brepgraph::Vec3 origin = {0.0, 0.0, 0.0};
                        brepgraph::Vec3 axis   = {
                            ExtParam(feat, "helix_place_ox", 0.0),
                            ExtParam(feat, "helix_place_oy", 0.0),
                            ExtParam(feat, "helix_place_oz", 1.0)};
                        int o_n = cg->AddConst(origin, "helix_place_origin");
                        int d_n = cg->AddConst(axis,   "helix_place_axis");
                        int a_n = cg->AddConst(
                            ExtParam(feat, "helix_place_angle", 0.0),
                            "helix_place_angle");
                        wire_n = cg->AddOp("rotate", {wire_n, o_n, d_n, a_n},
                                           {}, feat.name + ":helix_rot");
                    }
                    if (feat.ext_params.count("helix_place_px") ||
                        feat.ext_params.count("helix_place_py") ||
                        feat.ext_params.count("helix_place_pz"))
                    {
                        brepgraph::Vec3 off = {
                            ExtParam(feat, "helix_place_px", 0.0),
                            ExtParam(feat, "helix_place_py", 0.0),
                            ExtParam(feat, "helix_place_pz", 0.0)};
                        int o_n = cg->AddConst(off, "helix_place_offset");
                        wire_n = cg->AddOp("translate", {wire_n, o_n},
                                           {}, feat.name + ":helix_tr");
                    }
                }
                else
                {
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

                    wire_n = AddSketchWireNode(*cg, *spine_sk);
                }
                int solid_n  = cg->AddConst(is_solid, "is_solid");
                // FreeCAD Part::Sweep Frenet=true needs the true Frenet
                // trihedron so a non-symmetric section stays radially
                // oriented along the helix (a thread). Stashed as
                // sweep_frenet; absent (-> false / corrected Frenet) for
                // the PartDesign Pipe path, preserving its behavior.
                bool sweep_frenet = (ExtParam(feat, "sweep_frenet", 0.0) != 0.0);
                int frenet_n = cg->AddConst(sweep_frenet, "sweep_frenet");
                int tool_n   = cg->AddOp("sweep",
                                          {profile_n, wire_n, solid_n, frenet_n},
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
                ti.base_node = base_node;

                if (!subtractive)
                {
                    if (base_node < 0)
                    {
                        node = tool_n;
                        ti.op_kind = '0';
                    }
                    else
                    {
                        node = cg->AddOp("fuse", {base_node, tool_n},
                                          {}, feat.name);
                        ti.op_kind = 'f';
                    }
                }
                else
                {
                    if (base_node < 0)
                    {
                        step_ok     = false;
                        out.err_msg = "SubtractivePipe without base shape: " + feat.name;
                        return;
                    }
                    node = cg->AddOp("cut", {base_node, tool_n},
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
                ti.base_node = base_node;

                if (!subtractive)
                {
                    if (base_node < 0)
                    {
                        node = tool_n;
                        ti.op_kind = '0';
                    }
                    else
                    {
                        node = cg->AddOp("fuse", {base_node, tool_n},
                                          {}, feat.name);
                        ti.op_kind = 'f';
                    }
                }
                else
                {
                    if (base_node < 0)
                    {
                        step_ok     = false;
                        out.err_msg = "SubtractiveLoft without base shape: " + feat.name;
                        return;
                    }
                    node = cg->AddOp("cut", {base_node, tool_n},
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
                if (base_node < 0)
                {
                    step_ok = false;
                    return;
                }
                // The body the resolver / ChFi3d looks at. Starts as
                // base_node (no pre-dressup refine; that pass was
                // tried via c17a1d22 and silently shifted resolver
                // matches to the wrong cax edge, so it's reverted)
                // and gets re-bound to a split_body_at_points node
                // below when the reader supplied split hints.
                int body_n = base_node;

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

                // Record this dressup so a pattern over the dressed seed
                // can replicate it per instance (re-resolve the moved
                // anchors on each copy). base_feat_id = the feature it
                // dressed (its Role::Base input).
                {
                    FeatureDressupInfo di;
                    di.kind = std::is_same_v<T, FeatPayloadFillet>
                                  ? FeatType::Fillet : FeatType::Chamfer;
                    di.size = dressup_size;
                    di.refs = p.edges;
                    for (size_t bi = 0; bi < feat.input_feature_ids.size(); ++bi) {
                        InputRole role = (bi < feat.input_roles.size())
                                            ? feat.input_roles[bi] : InputRole::Base;
                        if (role == InputRole::Base) {
                            di.base_feat_id = feat.input_feature_ids[bi];
                            break;
                        }
                    }
                    feature_dressups[feat.id] = std::move(di);
                }
            }

            // ---- Shell ----
            else if constexpr (std::is_same_v<T, FeatPayloadShell>)
            {
                if (base_node < 0)
                {
                    step_ok = false;
                    return;
                }

                std::vector<int> face_nodes;
                face_nodes.reserve(p.faces_to_open.size());
                for (size_t k = 0; k < p.faces_to_open.size(); ++k)
                {
                    int rn = AddResolveRefNode(*cg, "resolve_face_ref",
                                                base_node, p.faces_to_open[k],
                                                opt.topo_tolerance,
                                                feat.name + ":face");
                    face_nodes.push_back(rn);
                    if (opt.write_back_resolved)
                        resolve_back.push_back({rn, &p.faces_to_open[k]});
                }

                int t = cg->AddConst(p.thickness, "thickness");
                node = cg->AddOp("shell", {base_node, t},
                                  face_nodes, feat.name);
            }

            // ---- Mirror ----
            //
            // FreeCAD's PartDesign::Mirrored produces orig + mirror.
            // Our `mirror` op only returns the mirror image, so we
            // always add the original back ourselves.
            //
            // Originals = [F1..Fn]: each Fi's tool effect already
            // lives in base_node (it was applied when Fi was replayed
            // and committed to feature_nodes). We add mirror(Fi.tool)
            // for every Fi using Fi.op_kind so every original
            // contributes its mirror -- previously only origs[0] was
            // honored, which silently dropped pads/pockets when users
            // mirrored multiple features together.
            //
            // Originals empty: mirror the whole base body and fuse.
            else if constexpr (std::is_same_v<T, FeatPayloadMirror>)
            {
                brepgraph::Vec3 origin = {p.plane_origin[0],
                                         p.plane_origin[1],
                                         p.plane_origin[2]};
                brepgraph::Vec3 normal = {p.plane_normal[0],
                                         p.plane_normal[1],
                                         p.plane_normal[2]};

                auto pi = ResolvePatternInputs(feat, feature_tools,
                                               feature_nodes);
                if (pi.body_target >= 0 && base_node >= 0)
                {
                    // DELTA mirror (ZW3D feature-mirror): reflect the CUMULATIVE
                    // geometry the mirrored features added -- (base) minus (the
                    // body before the first mirrored feature) -- so their
                    // fillets / chamfers / patterns / cuts all come along. The
                    // per-original-tool path below mirrors raw tool solids and
                    // silently drops dressups, leaving the mirror side missing
                    // its rounds. pi.body_target is that pre-originals body,
                    // wired Role::PatternTarget by the Zw reader; FreeCAD mirrors
                    // carry only Originals -> the per-tool path. delta =
                    // cut(base, pre); result = fuse(base, mirror(delta)).
                    int o_n   = cg->AddConst(origin, "origin");
                    int n_n   = cg->AddConst(normal, "normal");
                    int delta = cg->AddOp("cut", {base_node, pi.body_target},
                                          {}, feat.name + ":delta");
                    int m_n   = cg->AddOp("mirror", {delta, o_n, n_n},
                                          {}, feat.name + ":mirror");
                    node = cg->AddOp("fuse", {base_node, m_n},
                                     {}, feat.name + ":fuse");
                }
                else if (!pi.originals.empty() && base_node >= 0)
                {
                    node = AddMirroredOriginals(*cg, base_node, pi.originals,
                                                origin, normal, feat.name);
                }
                else if (base_node >= 0)
                {
                    node = AddMirrorWithOriginal(*cg, base_node, origin, normal,
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

                // Reader does not emit Role::PatternTarget on
                // LinearPattern (Draft only emits polar Arrays), so
                // pi.body_target is always -1 here -- flow through
                // the helper anyway for shape consistency with
                // CircularPattern.
                auto pi = ResolvePatternInputs(feat, feature_tools,
                                               feature_nodes);
                const bool onto_running =
                    ExtParam(feat, "pattern_onto_running", 0.0) != 0.0
                    && base_node >= 0;
                PAT_LOG("[pat] LINEAR feat=%s id=%u originals=%zu onto_running=%d "
                        "base_node=%d count1=%d count2=%d\n",
                        feat.name.c_str(), feat.id, pi.originals.size(),
                        (int)onto_running, base_node, (int)p.count1, (int)p.count2);

                if (pi.originals.empty()) {
                    // No Original tool -> pattern the whole body.
                    if (base_node < 0) {
                        step_ok = false;
                        return;
                    }
                    node = cg->AddOp("linear_pattern",
                                      {base_node, d1, c1, s1, d2, c2, s2},
                                      {}, feat.name);
                } else if (onto_running) {
                    // ZW3D: replay the seed group's CONTRIBUTION -- its Tool
                    // seeds AND the dressups on them -- once per NON-seed
                    // instance, each under the instance's rigid transform.
                    // i=0 is the live, already-dressed seed group (in
                    // base_node), so it is skipped; every other (i,j) cell
                    // transforms the tools by its offset and re-applies the
                    // dressups with the offset anchors, so a chamfer/fillet on
                    // the seed is replicated onto each copy. This is the general
                    // rigid image of the seed sub-graph -- no pos-0 cut, no
                    // per-dressup special case (it subsumes both). FreeCAD never
                    // sets pattern_onto_running, so its CombinePatternedTool
                    // path below is unchanged.
                    auto contribs = AssembleContributions(
                        feat, feature_tools, feature_dressups);
                    bool has_dressup    = false;
                    bool all_equivariant = true;
                    for (const auto& c : contribs) {
                        if (c.is_tool) {
                            if (!c.equivariant) all_equivariant = false;
                        } else {
                            has_dressup = true;
                        }
                    }
                    if (!all_equivariant) {
                        if (!out.err_msg.empty()) out.err_msg += "; ";
                        out.err_msg += "pattern " + feat.name +
                            " replicates a non-Blind (up-to) extrude by "
                            "rigid transform; geometry may be wrong";
                    }

                    // Decide HOW to lower from the pattern's GEOMETRY, not from
                    // the import-layer onto_running flag: a pure-tool, rigid-
                    // equivariant pattern needs no per-instance dressup re-
                    // resolution, so it lowers to the SHARED linear_pattern op
                    // (one n-ary instance union, see TopoAlgo_Ext) combined onto
                    // the running body ONCE per tool -- instead of N sequential
                    // per-instance booleans. The seed copy (i=0) is already in
                    // base_node; fusing/cutting the full instance set (which
                    // includes the coincident seed) over it is idempotent. Only
                    // patterns that carry a per-instance dressup, or a non-
                    // equivariant tool, fall back to the per-instance expansion.
                    if (all_equivariant) {
                        // Phase 1: replicate every tool with the shared n-ary
                        // linear_pattern op and combine each onto the running
                        // body once -- replaces count1*count2 sequential tool
                        // booleans with one pattern + one combine per tool.
                        int acc = base_node;
                        for (const auto& c : contribs) {
                            if (!c.is_tool) continue;
                            int pat = cg->AddOp(
                                "linear_pattern",
                                {c.tool_node, d1, c1, s1, d2, c2, s2},
                                {}, feat.name + ":pat");
                            const char* op = (c.op_kind == 'c') ? "cut" : "fuse";
                            acc = cg->AddOp(op, {acc, pat}, {}, feat.name);
                        }
                        // Phase 2: per-instance dressups (chamfer/fillet). All
                        // tools are present on `acc` now and pattern instances
                        // occupy DISJOINT regions, so re-resolving + applying
                        // each copy's dressup on the unified body is equivalent
                        // to interleaving it between the booleans (resolve is
                        // geometric, not index-based) -- without paying N
                        // sequential tool fuses.
                        //
                        // The seed (i=0,j=0) is NOT skipped here, unlike the
                        // per-instance path below. There, base_node carries the
                        // seed dressup untouched. Here, Phase 1 re-fused the
                        // raw (un-dressed) seed tool over base_node -- the
                        // n-ary linear_pattern includes the i=0 instance -- so a
                        // CONVEX seed chamfer/fillet (which removes material) got
                        // refilled and must be re-applied. Re-applying is safe
                        // for the concave case too: that dressup added material
                        // outside the raw tool, so it survived Phase 1, and the
                        // i=0 re-resolve simply misses the now-gone sharp edge
                        // and no-ops. (Without this, R2900_20's first chamfer --
                        // a 0.2 mm convex chamfer on a patterned 0.4 mm boss --
                        // silently vanished.)
                        if (has_dressup) {
                            std::vector<Contribution> dressups;
                            for (const auto& c : contribs)
                                if (!c.is_tool) dressups.push_back(c);
                            brepgraph::Vec3 u1 = NormalizedDir(p.dir1);
                            brepgraph::Vec3 u2 = NormalizedDir(p.dir2);
                            double l2sq = p.dir2[0] * p.dir2[0]
                                        + p.dir2[1] * p.dir2[1]
                                        + p.dir2[2] * p.dir2[2];
                            int eff2 = (l2sq > 1e-30 && p.count2 > 1)
                                            ? (int)p.count2 : 1;
                            std::vector<InstanceXform> Xs;
                            for (int i = 0; i < (int)p.count1; ++i) {
                                for (int j = 0; j < eff2; ++j) {
                                    InstanceXform X;
                                    for (int k = 0; k < 3; ++k)
                                        X.offset[k] = i * p.spacing1 * u1[k]
                                                    + j * p.spacing2 * u2[k];
                                    Xs.push_back(X);
                                }
                            }
                            // Batch all instances' dressup edges into ONE
                            // fillet/chamfer per contribution (one body rebuild)
                            // instead of N. Per-instance fallback under the gate.
                            if (pattern_cluster_on()) {
                                acc = ApplyPatternDressupsBatched(
                                    *cg, acc, dressups, Xs,
                                    opt.topo_tolerance, feat.name + ":dress");
                            } else {
                                for (const InstanceXform& X : Xs) {
                                    acc = ApplyPatternInstance(
                                        *cg, acc, dressups, X,
                                        opt.topo_tolerance,
                                        feat.name + ":dress");
                                }
                            }
                        }
                        node = acc;
                    } else {
                    brepgraph::Vec3 u1 = NormalizedDir(p.dir1);
                    brepgraph::Vec3 u2 = NormalizedDir(p.dir2);
                    double l2sq = p.dir2[0] * p.dir2[0]
                                + p.dir2[1] * p.dir2[1]
                                + p.dir2[2] * p.dir2[2];
                    int eff2 = (l2sq > 1e-30 && p.count2 > 1) ? (int)p.count2 : 1;
                    std::vector<InstanceXform> Xs;
                    for (int i = 0; i < (int)p.count1; ++i) {
                        for (int j = 0; j < eff2; ++j) {
                            if (i == 0 && j == 0) {
                                continue;   // the live seed group
                            }
                            InstanceXform X;   // linear -> pure translation
                            for (int k = 0; k < 3; ++k)
                                X.offset[k] = i * p.spacing1 * u1[k]
                                            + j * p.spacing2 * u2[k];
                            Xs.push_back(X);
                        }
                    }
                    // Cluster copies + single combine onto base (see
                    // ApplyPatternClustered); per-instance fallback otherwise.
                    int clustered = pattern_cluster_on()
                        ? ApplyPatternClustered(*cg, base_node, contribs, Xs,
                                                feat.name + ":clust")
                        : -1;
                    if (clustered >= 0) {
                        node = clustered;
                    } else {
                        int acc = base_node;
                        for (const InstanceXform& X : Xs) {
                            acc = ApplyPatternInstance(*cg, acc, contribs, X,
                                                       opt.topo_tolerance,
                                                       feat.name + ":inst");
                        }
                        node = acc;
                    }
                    }
                } else {
                    int pat = cg->AddOp("linear_pattern",
                                         {pi.originals[0].tool_node, d1, c1, s1, d2, c2, s2},
                                         {}, feat.name);
                    node = CombinePatternedTool(*cg, pi.originals[0], pat, feat.name);
                }
            }

            // ---- CircularPattern ----
            //
            // Target resolution priority:
            //   1. PartDesign Originals (input_feature_ids entries
            //      with Role::Tool) -- multiply each Original's
            //      tool, then combine with its base via op_kind.
            //      This is the PartDesign::PolarPattern path.
            //   2. Draft polar Array's Base link, pre-resolved by the
            //      Reader and pushed into input_feature_ids with
            //      Role::PatternTarget. Pattern that feature's body
            //      shape directly. The pre-resolution in the Reader
            //      means this doesn't depend on emission order -- a
            //      Draft Array sitting after later bodies still
            //      patterns the linked target rather than whatever
            //      the running tip happens to be.
            //   3. base_node -- body chain pred for body-owned
            //      Patterns; -1 / errors out for everyone else.
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

                auto pi = ResolvePatternInputs(feat, feature_tools,
                                               feature_nodes);
                const bool onto_running =
                    ExtParam(feat, "pattern_onto_running", 0.0) != 0.0
                    && base_node >= 0;
                PAT_LOG("[pat] CIRCULAR feat=%s id=%u originals=%zu onto_running=%d "
                        "base_node=%d count=%d\n",
                        feat.name.c_str(), feat.id, pi.originals.size(),
                        (int)onto_running, base_node, (int)p.count);

                if (pi.originals.empty()) {
                    int target = (pi.body_target >= 0) ? pi.body_target
                                                        : base_node;
                    if (target < 0) { step_ok = false; return; }
                    node = cg->AddOp("circular_pattern",
                                      {target, o, a, c, t}, {}, feat.name);
                } else if (onto_running) {
                    // ZW3D: replay the seed group's CONTRIBUTION (its Tool seeds
                    // AND the dressups on them) once per NON-seed instance, each
                    // under that instance's rigid ROTATION about the axis. i=0 is
                    // the live, already-dressed seed (in base_node) so it is
                    // skipped. Mirrors the linear onto_running path; only the
                    // per-instance transform differs (rotation vs translation),
                    // reusing the shared per-instance machinery. Per-instance
                    // (one fuse per copy onto the running body) is more robust
                    // than the n-ary union for many overlapping thin copies.
                    auto contribs = AssembleContributions(
                        feat, feature_tools, feature_dressups);
                    // step MUST match TopoAlgo_Ext::CircularPattern (angle/count).
                    double step = (p.count > 1)
                        ? (p.total_angle / static_cast<double>(p.count)) : 0.0;
                    // Push each copy p.penetration into the body along -axis_dir
                    // so the thin coplanar boss base overlaps the body (a pure
                    // coplanar contact does not fuse at mm scale).
                    brepgraph::Vec3 pen = {-p.penetration * axis[0],
                                           -p.penetration * axis[1],
                                           -p.penetration * axis[2]};
                    std::vector<InstanceXform> Xs;
                    for (int i = 1; i < (int)p.count; ++i) {
                        InstanceXform X;
                        X.is_rotation = true;
                        X.axis_origin = origin;
                        X.axis_dir    = axis;
                        X.angle       = step * i;
                        X.post_offset = pen;
                        Xs.push_back(X);
                    }
                    // Cluster the copies into one sub-body + a single combine
                    // onto the base instead of N fuses onto the growing body
                    // (see ApplyPatternClustered). Falls back to per-instance
                    // when a dressup / mixed op_kind needs the running body.
                    int clustered = pattern_cluster_on()
                        ? ApplyPatternClustered(*cg, base_node, contribs, Xs,
                                                feat.name + ":clust")
                        : -1;
                    if (clustered >= 0) {
                        node = clustered;
                    } else {
                        int acc = base_node;
                        for (const InstanceXform& X : Xs) {
                            acc = ApplyPatternInstance(*cg, acc, contribs, X,
                                                       opt.topo_tolerance,
                                                       feat.name + ":inst");
                        }
                        node = acc;
                    }

                    // Register this circular pattern as a reusable TOOL so a
                    // LATER pattern can NEST it -- a pattern of a pattern, e.g.
                    // R2900_50's Pattern4 (linear) copies Pattern3's whole RING.
                    // The tool node is the ISOLATED instance union (an n-ary
                    // circular_pattern op, which already includes the i=0 seed),
                    // NOT `node` (that ring is already fused into the body). The
                    // outer pattern then lowers as linear_pattern(this ring) and
                    // replicates the entire ring. Built from the tool contributions
                    // only; a dressup on the seed is not carried into the nested
                    // copy (known limitation). The extra node is orphaned (never
                    // pulled) unless a later pattern actually references it, so a
                    // non-nested circular pattern pays nothing.
                    {
                        int  ring    = -1;
                        char ring_op = 'f';
                        for (const auto& cc : contribs) {
                            if (!cc.is_tool) { continue; }
                            int cpn = cg->AddOp("circular_pattern",
                                                {cc.tool_node, o, a, c, t}, {},
                                                feat.name + ":toolring");
                            ring = (ring < 0)
                                 ? cpn
                                 : cg->AddOp("fuse", {ring, cpn}, {},
                                             feat.name + ":toolring");
                            ring_op = cc.op_kind;
                        }
                        if (ring >= 0) {
                            FeatureToolInfo ti;
                            ti.tool_node   = ring;
                            ti.base_node   = base_node;
                            ti.op_kind     = (ring_op == 'c') ? 'c' : 'f';
                            ti.equivariant = true;
                            feature_tools[feat.id] = ti;
                        }
                    }
                } else {
                    int pat = cg->AddOp("circular_pattern",
                                         {pi.originals[0].tool_node, o, a, c, t},
                                         {}, feat.name);
                    node = CombinePatternedTool(*cg, pi.originals[0], pat, feat.name);
                }
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
            // already at position 0 within base_node (it was applied
            // when Fi was replayed), so re-applying the
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

                // Reader does not emit Role::PatternTarget on
                // MultiTransform (only Draft polar Arrays carry it),
                // so pi.body_target is always -1 here. Flow through
                // the helper anyway for shape consistency.
                auto pi = ResolvePatternInputs(feat, feature_tools,
                                               feature_nodes);

                if (pi.originals.empty())
                {
                    if (base_node < 0)
                    {
                        step_ok = false;
                        return;
                    }
                    node = apply_chain(base_node, feat.name);
                }
                else
                {
                    if (base_node < 0)
                    {
                        step_ok = false;
                        return;
                    }
                    int body = base_node;
                    for (size_t fi = 0; fi < pi.originals.size(); ++fi)
                    {
                        std::string tag = feat.name + ":orig"
                                        + std::to_string(fi);
                        int pat = apply_chain(pi.originals[fi].tool_node, tag);
                        const char* op_name =
                            (pi.originals[fi].op_kind == 'c') ? "cut" : "fuse";
                        body = cg->AddOp(op_name, {body, pat}, {},
                                          tag + ":combine");
                    }
                    node = body;
                }
            }

            // ---- Boolean (Part::Cut / Fuse / Common / MultiFuse /
            //              MultiCommon) ----
            //
            // The operand list lives in FeatureIR::input_feature_ids
            // with Role::Operand on each entry (P3.3.B). Collect them
            // in declaration order, look each one up in feature_nodes,
            // and fold pairwise. For Cut the first operand is the
            // kept base; for Fuse / Common the order only matters for
            // graph layout. The boolean's result is committed to
            // feature_nodes under this feature's id, so a downstream
            // Fillet / pattern that names this id as its Role::Base
            // input picks it up via ResolveBaseNode.
            else if constexpr (std::is_same_v<T, FeatPayloadBoolean>)
            {
                (void)p; // empty payload; data lives in feat.input_*

                std::vector<uint32_t> operands;
                operands.reserve(feat.input_feature_ids.size());
                for (size_t i = 0; i < feat.input_feature_ids.size(); ++i)
                {
                    InputRole role = (i < feat.input_roles.size())
                                        ? feat.input_roles[i]
                                        : InputRole::Base;
                    if (role == InputRole::Operand) {
                        operands.push_back(feat.input_feature_ids[i]);
                    }
                }

                if (operands.size() < 2)
                {
                    step_ok     = false;
                    out.err_msg = "boolean " + feat.name +
                                  " needs at least 2 operands, got " +
                                  std::to_string(operands.size());
                    return;
                }

                const char* op = "fuse";
                if (feat.type == FeatType::Cut)    op = "cut";
                if (feat.type == FeatType::Common) op = "common";

                int cur = -1;
                for (size_t oi = 0; oi < operands.size(); ++oi)
                {
                    auto nit = feature_nodes.find(operands[oi]);
                    if (nit == feature_nodes.end())
                    {
                        step_ok = false;
                        out.err_msg = "boolean " + feat.name +
                                      " operand " + std::to_string(oi) +
                                      " (feature id " +
                                      std::to_string(operands[oi]) +
                                      ") has no replayed shape";
                        return;
                    }
                    if (cur < 0) {
                        cur = nit->second;
                    } else {
                        std::string tag = feat.name;
                        if (oi + 1 < operands.size()) {
                            tag += ":fold" + std::to_string(oi);
                        }
                        cur = cg->AddOp(op, {cur, nit->second}, {}, tag);
                    }
                }
                node = cur;
            }

            // ---- Link (Assembly4 part instance) ----
            //
            // The sub-doc's features were inlined by FreeCadReader's
            // App::Link branch ahead of this Link feature, so by the
            // time we visit the Link the sub-tip has already been
            // replayed and feature_nodes[link.sub_tip_feature_id]
            // holds its CalcGraph node. We layer the Link's baked
            // placement on top: rotate(axis, angle) about the origin,
            // then translate(p). FreeCAD-side LCS-pair solving has
            // already collapsed every constr_* contribution into this
            // single Placement, so no constraint evaluation is needed
            // here.
            //
            // Identity-or-near-identity placements skip the ops
            // entirely so the Link node is the sub-tip node verbatim,
            // matching the "viewer sees the part exactly where the
            // sub-doc puts it" intuition for the root part of an asm.
            else if constexpr (std::is_same_v<T, FeatPayloadLink>)
            {
                int cur = base_node;

                if (cur < 0 && p.sub_tip_feature_id != 0)
                {
                    // Defensive: a malformed doc with the Role::Base
                    // input missing but sub_tip_feature_id set still
                    // points us at the right sub-node.
                    auto sit = feature_nodes.find(p.sub_tip_feature_id);
                    if (sit != feature_nodes.end()) {
                        cur = sit->second;
                    }
                }

                if (cur < 0)
                {
                    // No sub-tip is a legitimate state, not an error:
                    // layout-only sub-docs (Crankshaft.FCStd in
                    // asm_Cylinders is the prototype -- App::Part
                    // container holding sketches + LCS reference
                    // frames but zero PartDesign Body) contribute no
                    // geometry, so the Link should produce no
                    // CalcGraph node. node stays at -1, feature_nodes
                    // does not record an entry for this id, and
                    // downstream features see "no upstream" which is
                    // correct -- there is no upstream solid to
                    // chain into.
                    return;
                }

                const double k_eps_angle = 1e-12;
                const double k_eps_pos   = 1e-15;

                if (std::fabs(p.placement_angle) > k_eps_angle)
                {
                    brepgraph::Vec3 origin = {0.0, 0.0, 0.0};
                    brepgraph::Vec3 axis   = {
                        p.placement_ox, p.placement_oy, p.placement_oz };
                    int o_n = cg->AddConst(origin, "link_origin");
                    int d_n = cg->AddConst(axis,   "link_axis");
                    int a_n = cg->AddConst(p.placement_angle, "link_angle");
                    cur = cg->AddOp("rotate",
                                    {cur, o_n, d_n, a_n}, {},
                                    feat.name + ":link_rot");
                }

                if (std::fabs(p.placement_px) > k_eps_pos
                    || std::fabs(p.placement_py) > k_eps_pos
                    || std::fabs(p.placement_pz) > k_eps_pos)
                {
                    brepgraph::Vec3 off = {
                        p.placement_px, p.placement_py, p.placement_pz };
                    int o_n = cg->AddConst(off, "link_offset");
                    cur = cg->AddOp("translate",
                                    {cur, o_n}, {},
                                    feat.name + ":link_tr");
                }

                node = cur;
            }
            else if constexpr (std::is_same_v<T, FeatPayloadAsmConstraint>)
            {
                // Geometry-free metadata: Phase 4 LCS-pair solver
                // (when added) will read FeatPayloadAsmConstraint to
                // compute a Link's Placement at edit time; today the
                // Reader trusts FreeCAD's already-baked Placement and
                // this payload contributes no CalcGraph node.
                (void)p;
            }

            // ---- HoleWizard / PartDesign::Hole ----
            //
            // FreeCAD's Hole is rich: a cylindrical bore + optional
            // counterbore / countersink / conical drill-point tip +
            // optional threaded helix. Synthesising the right Cut
            // shape from FeatPayloadHoleWizard's typed fields plus the
            // ext_params bag (hole_cut_*, drill_point_*, thread_*) is
            // a real chunk of work that this arm does NOT do today.
            //
            // Minimum-viable behaviour: if the source was a .FCStd
            // (so doc.authored_shapes carries the FreeCAD-emitted
            // post-Hole running body shape from the .brp dump), use
            // that authored shape verbatim as this feature's node.
            // The shape already includes everything up through and
            // including the Hole operation, so downstream features
            // in the same body chain see a correct running body. The
            // standard authored-substitution fallback further down
            // would also catch this, but it requires node>=0 first
            // (i.e. an attempted-but-null cax replay); we never
            // attempt here, so we have to seed the const node up
            // front.
            //
            // No authored shape (raw .xml fixtures, or future readers
            // that don't dump .brp): pass base_node through so the
            // body chain still produces SOMETHING -- the body without
            // the hole -- and record a diagnostic. Better than a
            // hard fail on the whole document.
            // ---- BakedShape (FreeCAD Part::Feature etc.) ----
            //
            // Same shape-from-authored mechanism as the HoleWizard
            // arm below, but for features that carry NO synthesizable
            // parameters at all in our IR. Geometry comes verbatim
            // from doc.authored_shapes (the .brp dump). Drives
            // Piston.FCStd's Fillet001_solid (a collapsed PartDesign
            // body surfaced as a Part::Feature).
            else if constexpr (std::is_same_v<T, FeatPayloadBakedShape>)
            {
                (void)p;
                auto auth_it = doc.authored_shapes.find(feat.id);
                if (auth_it != doc.authored_shapes.end() && auth_it->second)
                {
                    node = cg->AddConst(auth_it->second,
                                        "baked_" + feat.name);
                }
                else
                {
                    if (!out.err_msg.empty()) {
                        out.err_msg += "; ";
                    }
                    out.err_msg +=
                        "BakedShape " + feat.name +
                        " has no authored shape; skipped";
                }
            }
            else if constexpr (std::is_same_v<T, FeatPayloadHoleWizard>)
            {
                (void)p;
                auto auth_it = doc.authored_shapes.find(feat.id);
                if (auth_it != doc.authored_shapes.end() && auth_it->second)
                {
                    node = cg->AddConst(auth_it->second,
                                        "authored_hole_" + feat.name);
                }
                else if (base_node >= 0)
                {
                    node = base_node;
                    if (!out.err_msg.empty()) {
                        out.err_msg += "; ";
                    }
                    out.err_msg +=
                        "Hole " + feat.name +
                        " has no authored .brp; chain continues without "
                        "the hole geometry";
                }
                else
                {
                    if (!out.err_msg.empty()) {
                        out.err_msg += "; ";
                    }
                    out.err_msg +=
                        "Hole " + feat.name +
                        " has neither authored .brp nor base shape; "
                        "skipped";
                }
            }

            // ---- Not implemented yet ----
            else
            {
                // Native Assembly WB joints carry a FeatPayloadOpaque
                // (no dedicated payload type) but are a deliberate
                // geometry-free no-op, not an unimplemented gap: the
                // joint graph lives in ext_strings/ext_params and the
                // solved part poses are already baked onto each Body's
                // Placement, so this feature contributes no CalcGraph
                // node. Skip silently instead of logging a spurious
                // "skipped unimplemented feature" warning. Genuine
                // unknown features keep FeatType::Unknown and still
                // surface the diagnostic below.
                if (feat.type == FeatType::Joint)
                {
                    return;
                }

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
            // Hard-failure substitution: if cax's replay of this
            // feature returns a null/empty shape and the reader
            // pre-loaded a FreeCAD-authored brep for it, replace
            // the failed node with a const node holding the
            // authored shape. Catches OCCT-bug dead-ends like
            // Page_037's Thickness (MakeThickSolidByJoin SEH-AVs
            // inside BRepTools_History on certain geometries; the
            // SEH harness in TopoAlgo demotes the AV to a clean
            // nullptr). Without this fallback the whole document
            // fails to load even though FreeCAD has the correct
            // shelled body sitting right there in the .FCStd.
            //
            // Eager-eval cost: the node would be eval'd at final
            // emit anyway; CalcGraph caches, so the only added
            // work is the per-feature decision -- bounded by the
            // size of doc.authored_shapes (empty when the source
            // wasn't .FCStd, in which case this is a no-op).
            //
            // Trigger gated on hard nulls only -- "let the doc
            // finish loading when OCCT is wedged", not "silently
            // mask normal-path bugs". TopoNaming lineage breaks
            // for the substituted feature (the authored shape's
            // TShape* aren't in the running lineage chain), but
            // downstream refs in this codebase resolve
            // geometrically (centroid + normal + samples), not by
            // UID, so the chain break only affects
            // write-back-uid consumers.
            auto auth_it = doc.authored_shapes.find(feat.id);
            if (!opt.analyze_only
                && auth_it != doc.authored_shapes.end() && auth_it->second)
            {
                auto val = cg->Eval(node);
                auto* sv = std::get_if<brepgraph::ShapeVal>(&val);
                bool ok = sv && sv->shape && !sv->shape->GetShape().IsNull();
                if (!ok)
                {
                    int sub = cg->AddConst(auth_it->second,
                                           "authored_sub_" + feat.name);
                    std::fprintf(stderr,
                                 "[Replayer] feat %s (id=%u) cax replay "
                                 "null; substituting FreeCAD authored brep "
                                 "(node %d -> %d)\n",
                                 feat.name.c_str(), feat.id, node, sub);
                    node = sub;
                }
            }

            // Dead-feature substitution: a feature that evaluates to
            // NOTHING (null Val, or a shape with no solids) while sitting
            // on a non-empty base must not poison the chain -- every
            // downstream boolean would inherit the nothing, the running
            // body silently vanishes, and the document emits only
            // whatever post-collapse fragments rebuilt from scratch
            // (R2900: one fragmented-profile boss erased 68 features of
            // work and the editor displayed an empty viewport). Fall back
            // to the base node: the body continues WITHOUT this feature,
            // and the gap is reported in err_msg like a skipped feature.
            else if (!opt.analyze_only && base_node >= 0 && node != base_node)
            {
                auto val = cg->Eval(node);
                auto* sv = std::get_if<brepgraph::ShapeVal>(&val);
                bool dead = !sv || !sv->shape ||
                            sv->shape->GetShape().IsNull();
                if (!dead)
                {
                    TopExp_Explorer ex(sv->shape->GetShape(), TopAbs_SOLID);
                    dead = !ex.More();
                }
                if (dead)
                {
                    if (!out.err_msg.empty()) {
                        out.err_msg += "; ";
                    }
                    out.err_msg += "feature " + feat.name +
                                   " evaluated to no solid; chain continues "
                                   "without it";
                    node = base_node;
                }
            }

            feature_nodes[feat.id] = node;
            out.op_ids.push_back(cg->CalcOpId(node, 0));

            // Body assembly placement (native Assembly WB). The reader
            // stashes the owning Body's Placement as asm_* ext_params on
            // the body's TIP feature; here we layer it onto the PART
            // output (rotate(axis, angle) about the origin, then
            // translate) so the body lands in its assembly pose. Crucially
            // we transform a SEPARATE node and leave feature_nodes[feat.id]
            // in body-local frame: an in-document FeatureBase clone reads
            // the source body's tip via feature_nodes and must get the
            // local geometry (it carries its own Body.Placement), not the
            // already-placed one. Same convention as the FeatPayloadLink
            // arm above.
            int out_node = node;
            {
                auto a_it = feat.ext_params.find("asm_angle");
                auto px_it = feat.ext_params.find("asm_px");
                auto py_it = feat.ext_params.find("asm_py");
                auto pz_it = feat.ext_params.find("asm_pz");
                double a  = (a_it  != feat.ext_params.end()) ? a_it->second  : 0.0;
                double px = (px_it != feat.ext_params.end()) ? px_it->second : 0.0;
                double py = (py_it != feat.ext_params.end()) ? py_it->second : 0.0;
                double pz = (pz_it != feat.ext_params.end()) ? pz_it->second : 0.0;
                if (std::fabs(a) > 1e-12)
                {
                    brepgraph::Vec3 origin = {0.0, 0.0, 0.0};
                    brepgraph::Vec3 axis   = {
                        feat.ext_params.count("asm_ox") ? feat.ext_params.at("asm_ox") : 0.0,
                        feat.ext_params.count("asm_oy") ? feat.ext_params.at("asm_oy") : 0.0,
                        feat.ext_params.count("asm_oz") ? feat.ext_params.at("asm_oz") : 1.0 };
                    int o_n = cg->AddConst(origin, "asm_origin");
                    int d_n = cg->AddConst(axis,   "asm_axis");
                    int a_n = cg->AddConst(a,       "asm_angle");
                    out_node = cg->AddOp("rotate",
                                         {out_node, o_n, d_n, a_n}, {},
                                         feat.name + ":asm_rot");
                }
                if (std::fabs(px) > 1e-15 || std::fabs(py) > 1e-15
                    || std::fabs(pz) > 1e-15)
                {
                    brepgraph::Vec3 off = {px, py, pz};
                    int o_n = cg->AddConst(off, "asm_offset");
                    out_node = cg->AddOp("translate",
                                         {out_node, o_n}, {},
                                         feat.name + ":asm_tr");
                }
            }

            // Record this feature as a potential top-level output.
            // Body-owned features collapse to a single candidate per
            // body (the latest tip overwrites earlier members in
            // emission order); standalone Part::* features each get
            // their own candidate and are filtered out at emission
            // if a downstream boolean consumed them as an operand.
            // The candidate carries the PLACED node (out_node) so the
            // emitted part sits in its assembly pose.
            {
                auto bit = feat.ext_strings.find("freecad_body");
                std::string body_name = (bit != feat.ext_strings.end())
                                            ? bit->second : std::string{};
                if (!body_name.empty())
                {
                    auto cit = body_candidate_idx.find(body_name);
                    if (cit == body_candidate_idx.end()) {
                        body_candidate_idx[body_name] = output_candidates.size();
                        output_candidates.push_back({feat.id, out_node});
                    } else {
                        output_candidates[cit->second] = {feat.id, out_node};
                    }
                }
                else {
                    output_candidates.push_back({feat.id, out_node});
                }
            }

            // Input consumption: any feat whose shape gets folded
            // into this one is marked so the emitter doesn't re-emit
            // it alongside this feature. Roles split by semantics:
            //
            //   Base / Operand / Tool  -- this feature absorbs the
            //     upstream: the upstream's shape is no longer a
            //     standalone output, only the result of this feature
            //     is. Marking the input as consumed is what stops a
            //     Body candidate from appearing next to a standalone
            //     Thickness that absorbed it (Page_037 went
            //     1 solid -> 2 solids before this gate was added).
            //     Body-internal cases are no-ops in practice: the
            //     body candidate is keyed on the latest tip's
            //     feat_id, which differs from the prev's id, so the
            //     insert doesn't filter anything.
            //
            //   PatternTarget          -- the feature multiplies but
            //     does not replace the target. FreeCAD's document
            //     tree shows both the targeted body AND the Array
            //     as separate top-level objects (Page_056-48-1: 4
            //     solids = 3 body candidates + Array, with Array's
            //     angle=0 copy overlapping Pad002). Skipping the
            //     mark preserves that count.
            //
            //   Reference              -- the feature reads the
            //     upstream's geometry but produces independent
            //     output (sketch supports, datum parents). Not
            //     consumed by definition.
            for (size_t i = 0; i < feat.input_feature_ids.size(); ++i)
            {
                InputRole role = (i < feat.input_roles.size())
                                    ? feat.input_roles[i]
                                    : InputRole::Base;
                if (role == InputRole::Reference ||
                    role == InputRole::PatternTarget) {
                    continue;
                }
                uint32_t iid = feat.input_feature_ids[i];
                if (iid != 0u && iid != 0xFFFFFFFFu) {
                    consumed_feat_ids.insert(iid);
                }
            }

            // Dump-bodies hook: when CAX_DUMP_BODIES is set in the
            // environment, write each feature's running body to a
            // .brp file in the current working directory. Filename
            // "cax_<id>_<name>.brp"; pair with FreeCAD's PartShapeN
            // entries inside the source .FCStd zip via the brp_diff
            // tool (tools/brp_diff/) to localise where cax replay
            // diverges from FreeCAD's authored geometry. Off by
            // default so production loads don't litter CWD.
            if (const char* dump = std::getenv("CAX_DUMP_BODIES");
                dump && dump[0] != '\0' && dump[0] != '0')
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

        if (!step_ok && feature_nodes.empty())
        {
            // Fail-fast only when no feature has produced a shape yet.
            // After at least one success, a later failure leaves the
            // partial result in feature_nodes / output_candidates so
            // the doc still emits whatever did succeed.
            out.ok    = false;
            out.shape = nullptr;
            return false;
        }
    }

    // Pick the emission node(s). Walk output_candidates in declaration
    // order, drop anything a downstream boolean consumed as an operand,
    // and emit the survivors. 0 -> no live candidates (every feature
    // either failed or was absorbed); 1 -> emit that node directly;
    // >1 -> wrap in a TopoDS_Compound so
    // a multi-body doc like Page_042 (5 PartDesign::Body containers,
    // no downstream booleans) lands every body in out.shape instead
    // of just the last one. Filtering by consumed_feat_ids keeps a
    // doc like Page_031 -- two Body tips fed into a Part::Cut, whose
    // result is then fed into a Part::Common -- from emitting the
    // already-absorbed bodies alongside the Common result.
    // Keep the source feat_id alongside each live node so the
    // per-part appearance (transparency) can be recovered below.
    std::vector<std::pair<int, uint32_t>> live_nodes;  // (node, feat_id)
    live_nodes.reserve(output_candidates.size());
    for (const auto& c : output_candidates) {
        if (consumed_feat_ids.count(c.feat_id) == 0) {
            live_nodes.push_back({c.node, c.feat_id});
        }
    }

    // Publish the surviving part feat_ids (emission order) regardless of
    // analyze_only -- ReplayParts uses this to split the document.
    out.live_feat_ids.reserve(live_nodes.size());
    for (const auto& ln : live_nodes) {
        out.live_feat_ids.push_back(ln.second);
    }

    // analyze_only: graph + candidates are all the caller wanted; skip
    // every geometry eval (the eager authored-shape fallback above was
    // already gated off, and the per-part / write-back evals below are
    // skipped here).
    if (opt.analyze_only)
    {
        out.calc_graph = cg;
        out.ok         = true;
        return true;
    }

    // feat_id -> transparency, for tagging each emitted part. Built
    // once from the doc so the per-part loop is O(1) per node.
    std::unordered_map<uint32_t, double> feat_transparency;
    feat_transparency.reserve(doc.features.size());
    for (const auto& f : doc.features) {
        if (f.material.present) {
            feat_transparency[f.id] = f.material.transparency;
        }
    }
    auto transparency_of = [&](uint32_t feat_id) -> double {
        auto it = feat_transparency.find(feat_id);
        return it != feat_transparency.end() ? it->second : 0.0;
    };

    // Evaluate every live node once, collect (shape, transparency)
    // into out.parts, and assemble out.shape from the same set.
    out.parts.reserve(live_nodes.size());
    {
        BRep_Builder    bb;
        TopoDS_Compound comp;
        bb.MakeCompound(comp);
        int                                 added = 0;
        std::shared_ptr<brepkit::TopoShape> solo_ts;

        for (const auto& ln : live_nodes)
        {
            auto val = cg->Eval(ln.first);
            auto* sv = std::get_if<brepgraph::ShapeVal>(&val);
            if (!sv || !sv->shape) continue;
            const TopoDS_Shape& s = sv->shape->GetShape();
            if (s.IsNull()) continue;

            ReplayPart part;
            part.shape        = sv->shape;
            part.transparency = transparency_of(ln.second);
            part.feat_id      = ln.second;
            out.parts.push_back(std::move(part));

            bb.Add(comp, s);
            if (added == 0) solo_ts = sv->shape;
            ++added;
        }

        if (added > 1) {
            out.shape = std::make_shared<brepkit::TopoShape>(comp);
        } else if (added == 1) {
            out.shape = solo_ts;
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

// ============================================================
// Per-part replay (ReplayParts)
// ============================================================

namespace {

// Read-side mirror of FreeCadReader's RemapPayloadFeatureRefs: collect
// every feature-id this feature references through TYPED payload fields
// (sketch profiles / spines). input_feature_ids are followed separately
// by the closure walk. Keep this in lock-step with RemapPayloadFeatureRefs
// -- a payload that gains a feature-id ref there must gain one here, or
// the split sub-doc would drop a dependency.
void CollectPayloadFeatureRefs(const FeatureIR& feat, std::vector<uint32_t>& out)
{
    std::visit([&](const auto& p) {
        using T = std::decay_t<decltype(p)>;
        if constexpr (std::is_same_v<T, FeatPayloadSketch>) {
            out.push_back(p.sketch_id);
        } else if constexpr (std::is_same_v<T, FeatPayloadExtrude>) {
            out.push_back(p.sketch_id);
        } else if constexpr (std::is_same_v<T, FeatPayloadRevolve>) {
            out.push_back(p.sketch_id);
        } else if constexpr (std::is_same_v<T, FeatPayloadLoft>) {
            for (uint32_t sid : p.profile_sketch_ids) out.push_back(sid);
        } else if constexpr (std::is_same_v<T, FeatPayloadSweep>) {
            out.push_back(p.profile_sketch_id);
        } else if constexpr (std::is_same_v<T, FeatPayloadHoleWizard>) {
            out.push_back(p.sketch_id);
        } else if constexpr (std::is_same_v<T, FeatPayloadRib>) {
            out.push_back(p.sketch_id);
        }
    }, feat.data);

    auto sp = feat.ext_params.find("spine_sketch_id");
    if (sp != feat.ext_params.end()) {
        out.push_back((uint32_t)sp->second);
    }
}

// Transitive feature closure of `part_feat_id`: the part itself plus
// every feature it depends on via input_feature_ids and typed sketch
// refs. This is the exact set of features the part needs to replay in
// isolation.
std::unordered_set<uint32_t> CollectClosure(
    const std::unordered_map<uint32_t, const FeatureIR*>& by_id,
    uint32_t part_feat_id)
{
    std::unordered_set<uint32_t> keep;
    std::vector<uint32_t>        work{ part_feat_id };
    std::vector<uint32_t>        refs;

    while (!work.empty())
    {
        uint32_t fid = work.back();
        work.pop_back();
        if (fid == 0u || fid == 0xFFFFFFFFu) continue;
        if (!keep.insert(fid).second) continue;   // already visited

        auto it = by_id.find(fid);
        if (it == by_id.end()) continue;
        const FeatureIR* f = it->second;

        for (uint32_t iid : f->input_feature_ids) work.push_back(iid);
        refs.clear();
        CollectPayloadFeatureRefs(*f, refs);
        for (uint32_t r : refs) work.push_back(r);
    }
    return keep;
}

// Materialise a standalone sub-document from a feature closure. Features
// keep their original ids and document order, so the sub-doc replays
// identically to the same features inside the full doc -- only isolated.
DocumentIR BuildPartSubDoc(const DocumentIR& doc,
                           const std::unordered_set<uint32_t>& keep)
{
    DocumentIR sub;
    sub.source   = doc.source;
    sub.doc_path = doc.doc_path;
    for (const auto& f : doc.features) {
        if (keep.count(f.id)) sub.features.push_back(f);
    }
    for (const auto& sk : doc.sketches) {
        if (keep.count(sk.feature_id)) sub.sketches.push_back(sk);
    }
    for (const auto& kv : doc.authored_shapes) {
        if (keep.count(kv.first)) sub.authored_shapes.insert(kv);
    }
    return sub;
}

} // namespace

bool Replayer::ReplayParts(DocumentIR& doc, const ReplayOptions& opt,
                           ReplayResult& out, bool parallel)
{
    out = ReplayResult{};

    // 1. Analysis pass -- discover the top-level parts using the SAME
    //    candidate logic Replay uses, but without materialising geometry.
    ReplayResult ares;
    {
        ReplayOptions aopt       = opt;
        aopt.analyze_only        = true;
        aopt.write_back_resolved = false;
        aopt.commit_versions     = false;
        Replayer analyzer;   // own naming/vtree; discarded
        if (!analyzer.Replay(doc, aopt, ares)) {
            out.err_msg = ares.err_msg.empty()
                              ? "ReplayParts: analysis pass failed"
                              : ares.err_msg;
            return false;
        }
    }
    const std::vector<uint32_t> parts = ares.live_feat_ids;
    PAT_LOG("[parts] ReplayParts: live_parts=%zu doc_features=%zu\n",
            parts.size(), doc.features.size());

    // Nothing to gain from splitting 0 or 1 part -- replay whole serially.
    if (parts.size() <= 1) {
        PAT_LOG("[parts] -> FALLBACK serial Replay (parts<=1)\n");
        return Replay(doc, opt, out);
    }

    // 2. Compute each part's feature closure, then gate on independence.
    //    Splitting is only geometry-identical to a serial Replay when the
    //    parts are INDEPENDENT (disjoint closures). If two parts share a
    //    feature -- a Pattern's target (PatternTarget role) or a Clone's
    //    source (Reference role) is a non-consumed input that lands in
    //    BOTH closures -- building them in isolation can diverge in
    //    topology (shared sub-shapes counted/merged differently), so we
    //    fall back to a whole-document serial Replay. Pure assemblies
    //    (independent instances) stay disjoint and split cleanly.
    std::unordered_map<uint32_t, const FeatureIR*> by_id;
    by_id.reserve(doc.features.size());
    for (const auto& f : doc.features) by_id[f.id] = &f;

    std::vector<std::unordered_set<uint32_t>> closures;
    closures.reserve(parts.size());
    std::unordered_map<uint32_t, int> claim_count;
    bool disjoint = true;
    for (uint32_t pid : parts) {
        auto c = CollectClosure(by_id, pid);
        for (uint32_t fid : c) {
            if (++claim_count[fid] > 1) { disjoint = false; }
        }
        closures.push_back(std::move(c));
    }

    if (!disjoint) {
        // Not safely splittable -- replay the whole document serially.
        PAT_LOG("[parts] -> FALLBACK serial Replay (closures NOT disjoint)\n");
        return Replay(doc, opt, out);
    }
    PAT_LOG("[parts] -> SPLIT into %zu independent parts\n", parts.size());

    std::vector<DocumentIR> subdocs;
    subdocs.reserve(parts.size());
    for (auto& c : closures) {
        subdocs.push_back(BuildPartSubDoc(doc, c));
    }

    // 3. Replay each sub-doc through its OWN Replayer (isolated CalcGraph
    //    + TopoNaming + Evaluator), so the runs share no mutable state and
    //    can execute concurrently. Per-part naming is not persisted, so
    //    write-back / version commits are forced off.
    std::vector<ReplayResult> subres(subdocs.size());
    ReplayOptions sopt       = opt;
    sopt.analyze_only        = false;
    sopt.write_back_resolved = false;
    sopt.commit_versions     = false;

    auto run_one = [&](size_t i) {
        Replayer r;
        r.Replay(subdocs[i], sopt, subres[i]);
    };

    if (!parallel)
    {
        for (size_t i = 0; i < subdocs.size(); ++i) run_one(i);
    }
    else
    {
        unsigned hw      = std::thread::hardware_concurrency();
        size_t   workers = std::min<size_t>(subdocs.size(), hw ? hw : 4u);
        std::atomic<size_t>      next{ 0 };
        std::vector<std::thread> pool;
        pool.reserve(workers);
        for (size_t w = 0; w < workers; ++w) {
            pool.emplace_back([&] {
                for (;;) {
                    size_t i = next.fetch_add(1);
                    if (i >= subdocs.size()) break;
                    run_one(i);
                }
            });
        }
        for (auto& t : pool) t.join();
    }

    // 4. Merge parts in document (analysis) order; compound the shapes.
    //    Each sub-doc was built for ONE part (parts[i]); take exactly
    //    that part. A sub-doc can surface EXTRA live candidates -- a
    //    Pattern's target (PatternTarget role) or a Clone's source
    //    (Reference role) is a NON-consumed input, so it re-appears as a
    //    standalone output inside this sub-doc even though it's already
    //    emitted by its own sub-doc. Selecting feat_id == parts[i] keeps
    //    the merged set byte-identical to a serial Replay (no duplicates).
    BRep_Builder    bb;
    TopoDS_Compound comp;
    bb.MakeCompound(comp);
    int                                 added = 0;
    std::shared_ptr<brepkit::TopoShape> solo;

    for (size_t i = 0; i < subres.size(); ++i)
    {
        auto& sr = subres[i];

        ReplayPart* chosen = nullptr;
        for (auto& part : sr.parts) {
            if (part.feat_id == parts[i]) { chosen = &part; break; }
        }
        if (chosen && chosen->shape && !chosen->shape->GetShape().IsNull())
        {
            bb.Add(comp, chosen->shape->GetShape());
            if (added == 0) solo = chosen->shape;
            ++added;
            out.parts.push_back(std::move(*chosen));
        }

        if (!sr.err_msg.empty()) {
            if (!out.err_msg.empty()) out.err_msg += "; ";
            out.err_msg += sr.err_msg;
        }
    }

    if (added > 1) {
        out.shape = std::make_shared<brepkit::TopoShape>(comp);
    } else if (added == 1) {
        out.shape = solo;
    }

    // Hand back the analysis pass's (empty) naming/vtree so callers that
    // read these fields don't deref null. Per-part naming is intentionally
    // not merged -- ReplayParts is for the non-persisted load path.
    out.naming = ares.naming;
    out.vtree  = ares.vtree;
    out.ok     = true;
    return true;
}

} // namespace cadapp
