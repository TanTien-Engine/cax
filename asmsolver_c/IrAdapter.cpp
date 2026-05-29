#include "asmsolver_c/IrAdapter.h"

#include "cadapp_c/ir/Enums.h"
#include "cadapp_c/ir/FeatureIR.h"

#include <cmath>
#include <unordered_map>

namespace asmsolver {
namespace {

double getp(const cadapp::FeatureIR& f, const std::string& key, double def)
{
    auto it = f.ext_params.find(key);
    return it != f.ext_params.end() ? it->second : def;
}

std::string gets(const cadapp::FeatureIR& f, const std::string& key)
{
    auto it = f.ext_strings.find(key);
    return it != f.ext_strings.end() ? it->second : std::string{};
}

// Read a placement stored under `prefix` (px/py/pz + axis-angle
// ox/oy/oz/angle, the reader's StashPlacement convention) into a Pose
// with a unit quaternion (x,y,z,w).
Pose poseFrom(const cadapp::FeatureIR& f, const std::string& prefix)
{
    Pose p;
    p.t = {{ getp(f, prefix + "_px", 0.0),
             getp(f, prefix + "_py", 0.0),
             getp(f, prefix + "_pz", 0.0) }};
    double ox = getp(f, prefix + "_ox", 0.0);
    double oy = getp(f, prefix + "_oy", 0.0);
    double oz = getp(f, prefix + "_oz", 1.0);
    double angle = getp(f, prefix + "_angle", 0.0);
    double n = std::sqrt(ox*ox + oy*oy + oz*oz);
    if (n < 1e-12) { ox = 0; oy = 0; oz = 1; } else { ox/=n; oy/=n; oz/=n; }
    double h = 0.5 * angle, s = std::sin(h);
    p.q = {{ ox*s, oy*s, oz*s, std::cos(h) }};
    return p;
}

bool kindFromString(const std::string& s, JointKind& out, bool& grounded)
{
    grounded = false;
    if (s == "Grounded")        { out = JointKind::Fixed; grounded = true; return true; }
    if (s == "Fixed")           { out = JointKind::Fixed;        return true; }
    if (s == "Revolute")        { out = JointKind::Revolute;     return true; }
    if (s == "Cylindrical")     { out = JointKind::Cylindrical;  return true; }
    if (s == "Slider")          { out = JointKind::Slider;       return true; }
    if (s == "Ball")            { out = JointKind::Ball;         return true; }
    if (s == "Planar")          { out = JointKind::Planar;       return true; }
    if (s == "Distance")        { out = JointKind::Distance;     return true; }
    return false;   // Parallel / Perpendicular / Angle / gear / ... -- TODO
}

} // namespace

ImportResult BuildAssembly(const cadapp::DocumentIR& doc)
{
    ImportResult R;

    auto ensure_body = [&](const std::string& name) -> int {
        auto it = R.body_index.find(name);
        if (it != R.body_index.end()) return it->second;
        int idx = static_cast<int>(R.assembly.bodies.size());
        R.body_index[name] = idx;
        R.body_names.push_back(name);
        R.body_tip_feat.push_back(-1);
        R.assembly.bodies.push_back(Pose{});   // identity until a tip sets it
        return idx;
    };

    // ---- pass 1: enumerate bodies + their imported world placement ----
    // Every feature tagged freecad_body belongs to a body; the body's tip
    // carries the assembly placement under asm_* (absent => identity).
    for (const auto& f : doc.features) {
        std::string body = gets(f, "freecad_body");
        if (body.empty()) continue;
        int idx = ensure_body(body);
        if (f.ext_params.count("asm_px") || f.ext_params.count("asm_angle")) {
            R.assembly.bodies[idx] = poseFrom(f, "asm");
            R.body_tip_feat[idx]   = static_cast<int>(f.id);
        }
    }

    // ---- pass 2: joints ----
    for (const auto& f : doc.features) {
        if (f.type != cadapp::FeatType::Joint) continue;
        JointKind kind; bool grounded;
        if (!kindFromString(gets(f, "joint_kind"), kind, grounded)) {
            ++R.joints_skipped;
            continue;
        }

        Joint j;
        j.kind = kind;
        j.name = f.name;

        if (grounded) {
            std::string part = gets(f, "joint_ground_part");
            auto it = R.body_index.find(part);
            if (it == R.body_index.end()) { ++R.joints_skipped; continue; }
            j.grounded = true;
            j.body_a = it->second;
            j.conn_a = poseFrom(f, "joint_ground");
        } else {
            std::string pa = gets(f, "joint_ref1_part");
            std::string pb = gets(f, "joint_ref2_part");
            auto ia = R.body_index.find(pa);
            auto ib = R.body_index.find(pb);
            if (ia == R.body_index.end() || ib == R.body_index.end()) {
                ++R.joints_skipped;
                continue;
            }
            j.body_a = ia->second;
            j.body_b = ib->second;
            j.conn_a = poseFrom(f, "joint_p1");
            j.conn_b = poseFrom(f, "joint_p2");
            j.distance = getp(f, "joint_distance", 0.0);
            // radius (plane-to-cylinder) is geometry, not in the IR; a
            // later increment resolves it from the replayed OCCT shape.
            j.radius = 0.0;
        }

        R.assembly.joints.push_back(std::move(j));
        ++R.joints_built;
    }

    return R;
}

int ApplyToDocument(cadapp::DocumentIR& doc, const ImportResult& R)
{
    std::unordered_map<uint32_t, cadapp::FeatureIR*> by_id;
    for (auto& f : doc.features) by_id[f.id] = &f;

    int written = 0;
    for (size_t i = 0; i < R.assembly.bodies.size() && i < R.body_tip_feat.size(); ++i) {
        int fid = R.body_tip_feat[i];
        if (fid < 0) continue;
        auto it = by_id.find(static_cast<uint32_t>(fid));
        if (it == by_id.end()) continue;
        cadapp::FeatureIR& f = *it->second;
        const Pose& p = R.assembly.bodies[i];

        // quaternion (x,y,z,w) -> axis-angle (the reader/Replayer asm_*
        // convention); poseFrom() is the exact inverse, so this round-trips.
        double qw = p.q[3];
        if (qw >  1.0) qw =  1.0;
        if (qw < -1.0) qw = -1.0;
        double angle = 2.0 * std::acos(qw);
        double s = std::sqrt(1.0 - qw * qw > 0.0 ? 1.0 - qw * qw : 0.0);
        double ox, oy, oz;
        if (s < 1e-9) { ox = 0.0; oy = 0.0; oz = 1.0; angle = 0.0; }
        else          { ox = p.q[0]/s; oy = p.q[1]/s; oz = p.q[2]/s; }

        f.ext_params["asm_px"]    = p.t[0];
        f.ext_params["asm_py"]    = p.t[1];
        f.ext_params["asm_pz"]    = p.t[2];
        f.ext_params["asm_ox"]    = ox;
        f.ext_params["asm_oy"]    = oy;
        f.ext_params["asm_oz"]    = oz;
        f.ext_params["asm_angle"] = angle;
        ++written;
    }
    return written;
}

} // namespace asmsolver
