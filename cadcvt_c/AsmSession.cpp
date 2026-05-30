#include "cadcvt_c/AsmSession.h"

#include "cadcvt_c/reader/FreeCadReader.h"
#include "cadapp_c/emitter/Replayer.h"
#include "cadapp_c/ir/FeatureIR.h"

#include "asmsolver_c/AsmSolver.h"
#include "asmsolver_c/IrAdapter.h"
#include "asmsolver_c/IrAdapterGeom.h"

#include "brepkit_c/TopoShape.h"

#include <BRepBuilderAPI_Transform.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <Precision.hxx>
#include <gp_Dir.hxx>
#include <gp_Lin.hxx>
#include <gp_Pnt.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace cadcvt {

namespace {

// Pose (t + quat xyzw) -> OCCT transform (X' = R*X + t).
gp_Trsf TrsfOf(const asmsolver::Pose& p)
{
    gp_Trsf t;
    t.SetRotation(gp_Quaternion(p.q[0], p.q[1], p.q[2], p.q[3]));
    t.SetTranslationPart(gp_Vec(p.t[0], p.t[1], p.t[2]));
    return t;
}

bool PoseEqual(const asmsolver::Pose& a, const asmsolver::Pose& b)
{
    const double e = 1e-12;
    for (int i = 0; i < 3; ++i) if (std::abs(a.t[i] - b.t[i]) > e) return false;
    for (int i = 0; i < 4; ++i) if (std::abs(a.q[i] - b.q[i]) > e) return false;
    return true;
}

} // namespace

struct AsmSession::Impl {
    FreeCadReader            reader;
    cadapp::Replayer         replayer;
    cadapp::DocumentIR       doc;
    asmsolver::ImportResult  import;

    // Per-part replayed WORLD-frame shapes at the IMPORTED pose, with feat_id.
    std::vector<cadapp::ReplayPart> parts;
    // Snapshot of body poses at import time (the base for the pose delta).
    std::vector<asmsolver::Pose>    imported;

    std::vector<int> body_of_part;   // part i -> body index (-1 unknown)
    std::vector<int> part_of_body;   // body i -> part index (-1 none)

    // Last pick.
    int    grab_body          = -1;
    double grab_world[3]      = {0, 0, 0};
    double grab_anchor[3]     = {0, 0, 0};   // grab point in body-local frame

    int   nbodies() const {
        return static_cast<int>(import.assembly.bodies.size());
    }

    // Current placed shape for a body (base transformed by solved*imported^-1).
    std::shared_ptr<brepkit::TopoShape> placed_shape(int body) const {
        if (body < 0 || body >= static_cast<int>(part_of_body.size())) return nullptr;
        int p = part_of_body[body];
        if (p < 0 || p >= static_cast<int>(parts.size()) || !parts[p].shape) return nullptr;
        const asmsolver::Pose& cur = import.assembly.bodies[body];
        const asmsolver::Pose& imp = imported[body];
        if (PoseEqual(cur, imp)) return parts[p].shape;   // unmoved: keep identity
        gp_Trsf delta = TrsfOf(cur).Multiplied(TrsfOf(imp).Inverted());
        BRepBuilderAPI_Transform xf(parts[p].shape->GetShape(), delta, /*copy*/ true);
        return std::make_shared<brepkit::TopoShape>(xf.Shape());
    }
};

AsmSession::AsmSession() : m_impl(new Impl) {}
AsmSession::~AsmSession() = default;

bool AsmSession::Load(const std::string& path, double unit_scale, bool strict)
{
    m_err.clear();
    Impl& s = *m_impl;
    s.parts.clear();
    s.imported.clear();
    s.body_of_part.clear();
    s.part_of_body.clear();
    s.grab_body = -1;

    s.reader.SetUnitScale(unit_scale);
    s.reader.SetStrict(strict);

    std::string err;
    s.doc = cadapp::DocumentIR{};
    if (!s.reader.ReadFile(path, s.doc, &err)) {
        m_err = err.empty() ? "ReadFile failed" : err;
        return false;
    }

    cadapp::ReplayOptions opt;
    opt.write_back_resolved = false;
    opt.commit_versions     = false;
    cadapp::ReplayResult res;
    if (!s.replayer.ReplayParts(s.doc, opt, res, /*parallel*/ true)) {
        m_err = res.err_msg.empty() ? "Replay failed" : res.err_msg;
        return false;
    }
    s.parts = std::move(res.parts);

    // Build the constraint assembly + resolve plane-to-cylinder radii.
    s.import = asmsolver::BuildAssembly(s.doc);
    asmsolver::ResolveCylinderRadii(s.import, s.doc);
    s.imported = s.import.assembly.bodies;   // snapshot the imported poses

    // Map every feature id -> its owning body index (the reader tags every
    // feature of a body with `freecad_body`), so each part's tip feat_id
    // resolves to a body -- robust even for identity-imported bodies.
    std::unordered_map<uint32_t, int> feat_body;
    for (const auto& f : s.doc.features) {
        auto it = f.ext_strings.find("freecad_body");
        if (it == f.ext_strings.end()) continue;
        auto bi = s.import.body_index.find(it->second);
        if (bi != s.import.body_index.end()) feat_body[f.id] = bi->second;
    }

    s.body_of_part.assign(s.parts.size(), -1);
    s.part_of_body.assign(s.nbodies(), -1);
    for (size_t i = 0; i < s.parts.size(); ++i) {
        auto it = feat_body.find(s.parts[i].feat_id);
        if (it == feat_body.end()) continue;
        s.body_of_part[i]          = it->second;
        s.part_of_body[it->second] = static_cast<int>(i);
    }
    return true;
}

int AsmSession::body_count() const { return m_impl->nbodies(); }

std::string AsmSession::body_name(int body) const
{
    const auto& names = m_impl->import.body_names;
    if (body < 0 || body >= static_cast<int>(names.size())) return {};
    return names[body];
}

std::shared_ptr<brepkit::TopoShape> AsmSession::body_shape(int body) const
{
    return m_impl->placed_shape(body);
}

int AsmSession::Pick(double ox, double oy, double oz,
                     double dx, double dy, double dz)
{
    Impl& s = *m_impl;
    if (dx == 0.0 && dy == 0.0 && dz == 0.0) return -1;

    gp_Lin line(gp_Pnt(ox, oy, oz), gp_Dir(dx, dy, dz));

    int    best_body = -1;
    double best_w    = 0.0;
    gp_Pnt best_pnt;

    for (size_t i = 0; i < s.parts.size(); ++i) {
        int body = s.body_of_part[i];
        if (body < 0) continue;
        auto sh = s.placed_shape(body);
        if (!sh || sh->GetShape().IsNull()) continue;

        IntCurvesFace_ShapeIntersector its;
        its.Load(sh->GetShape(), Precision::Confusion());
        its.Perform(line, 0.0, Precision::Infinite());
        if (!its.IsDone()) continue;
        for (int k = 1; k <= its.NbPnt(); ++k) {
            double w = its.WParameter(k);
            if (w < 0.0) continue;                 // behind the ray origin
            if (best_body < 0 || w < best_w) {
                best_body = body;
                best_w    = w;
                best_pnt  = its.Pnt(k);
            }
        }
    }

    if (best_body < 0) { s.grab_body = -1; return -1; }

    s.grab_body     = best_body;
    s.grab_world[0] = best_pnt.X();
    s.grab_world[1] = best_pnt.Y();
    s.grab_world[2] = best_pnt.Z();

    // Grab point in the picked body's local frame, so pose*anchor tracks the
    // grabbed feature as the body moves.
    gp_Pnt local = best_pnt.Transformed(
        TrsfOf(s.import.assembly.bodies[best_body]).Inverted());
    s.grab_anchor[0] = local.X();
    s.grab_anchor[1] = local.Y();
    s.grab_anchor[2] = local.Z();
    return best_body;
}

void AsmSession::last_hit(double out[3]) const
{
    out[0] = m_impl->grab_world[0];
    out[1] = m_impl->grab_world[1];
    out[2] = m_impl->grab_world[2];
}

double AsmSession::Drag(int body, double tx, double ty, double tz, double weight)
{
    Impl& s = *m_impl;
    if (body < 0 || body >= s.nbodies()) return -1.0;

    asmsolver::Handle h;
    h.body         = body;
    h.target_world = {{ tx, ty, tz }};
    h.weight       = weight;
    // Use the grabbed anchor when dragging the picked body; otherwise pull the
    // body origin.
    if (body == s.grab_body) {
        h.anchor_local = {{ s.grab_anchor[0], s.grab_anchor[1], s.grab_anchor[2] }};
    } else {
        h.anchor_local = {{ 0, 0, 0 }};
    }

    std::vector<asmsolver::Handle> handles{ h };
    asmsolver::SolveResult r = asmsolver::SolveWithHandles(s.import.assembly, handles);
    return r.final_residual;
}

} // namespace cadcvt
