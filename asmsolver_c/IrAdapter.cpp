#include "asmsolver_c/IrAdapter.h"

#include "cadapp_c/ir/Enums.h"
#include "cadapp_c/ir/FeatureIR.h"

#include <cmath>

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

} // namespace asmsolver
