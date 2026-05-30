#pragma once

// asmsolver -- 3D assembly constraint solver.
//
// Solves rigid-body part poses (SE(3)) from a graph of kinematic joints,
// matching FreeCAD native Assembly WB semantics. Depends only on Ceres +
// Eigen (no OCCT / cax IR) so the core stays independently testable and
// reusable. A cax-side adapter (DocumentIR FeatType::Joint -> Assembly)
// lives elsewhere.
//
// Conventions: all lengths in the caller's unit (cax IR = metres; the
// FreeCAD reader already scales mm->m). Quaternions are (x,y,z,w),
// matching FreeCAD's PropertyPlacement Q0..Q3 order.

#include <array>
#include <string>
#include <vector>

namespace asmsolver {

// Rigid pose: translation + unit quaternion (x,y,z,w).
struct Pose {
    std::array<double, 3> t{{0, 0, 0}};
    std::array<double, 4> q{{0, 0, 0, 1}};
};

// Joint coordinate system (JCS) Z is the primary axis: rotation /
// translation axis for the kinematic pairs, plane normal for Planar /
// Distance. Each joint relates the JCS on body_a (conn_a, in a's local
// frame) to the JCS on body_b (conn_b).
enum class JointKind {
    Fixed,        // 0 DOF: the two JCS coincide
    Revolute,     // 1 DOF: free rotation about Z
    Cylindrical,  // 2 DOF: free rotation about + translation along Z
    Slider,       // 1 DOF: free translation along Z (no rotation)
    Ball,         // 3 DOF: free rotation, origins coincident
    Planar,       // 3 DOF: coincident planes (slide in XY + spin about Z)
    Distance,     // plane-plane offset `distance` along Z; if `radius`>0
                  // it is a plane-to-cylinder distance (axis lies in the
                  // plane, surface `distance` from it)
};

struct Joint {
    JointKind   kind = JointKind::Fixed;
    int         body_a = -1;
    int         body_b = -1;
    Pose        conn_a;            // JCS in body_a local coords
    Pose        conn_b;            // JCS in body_b local coords
    double      distance = 0.0;    // Distance joint offset
    double      radius   = 0.0;    // >0 => plane-to-cylinder (radius from BREP)
    bool        grounded = false;  // pin body_a's WORLD pose to conn_a
                                   // (body_b / conn_b ignored)
    std::string name;
};

struct Assembly {
    std::vector<Pose>  bodies;     // in: initial guess; out: solved poses
    std::vector<Joint> joints;
};

// A soft drag handle: pulls a point fixed in body `body` (anchor_local, in
// that body's local frame) toward a world target. Added as a low-weight
// residual so the hard joints dominate -- the handle only selects which
// point on the constraint manifold the solver lands on (interactive drag).
struct Handle {
    int                 body   = -1;
    std::array<double,3> anchor_local{{0, 0, 0}};
    std::array<double,3> target_world{{0, 0, 0}};
    double              weight = 1.0;
};

struct SolveOptions {
    int  max_iterations = 200;
    bool verbose        = false;   // stream Ceres progress to stdout
};

struct SolveResult {
    bool                converged        = false;
    double              initial_residual = 0.0;  // sqrt(sum of squares)
    double              final_residual   = 0.0;
    int                 iterations       = 0;
    std::string         termination;             // Ceres termination type
    std::vector<double> joint_residuals;         // per joint, post-solve
};

// Mobility / constraint diagnostics from the constraint Jacobian rank at
// the assembly's current poses.
struct DofInfo {
    int tangent_dofs        = 0;  // 6 * number of bodies (SE(3) tangent)
    int intended_constraints= 0;  // sum of DOF each joint is meant to remove
    int rank                = 0;  // numerical rank of the constraint Jacobian
    int mobility            = 0;  // tangent_dofs - rank: remaining DOF
                                  // (0 = fully constrained, >0 = under-)
    int redundancy          = 0;  // intended_constraints - rank: redundant
                                  // (over-constrained) equations
};

// Adjust assembly.bodies[] so every joint residual -> 0. bodies[] is read
// as the initial guess and overwritten with the result.
SolveResult Solve(Assembly& assembly, const SolveOptions& opts = {});

// Like Solve, but with extra soft drag handles (interactive editing): the
// hard joints are still satisfied while each handle pulls its body toward a
// world target. bodies[] is updated in place. The reported residuals are
// still the per-joint residuals (handles are not counted), so converged /
// final_residual measure whether the constraints stayed satisfied.
SolveResult SolveWithHandles(Assembly& assembly,
                             const std::vector<Handle>& handles,
                             const SolveOptions& opts = {});

// Analyse mobility / over-constraint without moving the assembly.
DofInfo AnalyzeDof(const Assembly& assembly);

// Residual L2 norm of a single joint at the assembly's current poses
// (no solving). For validation / over-constraint diagnostics.
double JointResidualNorm(const Assembly& assembly, const Joint& joint);

} // namespace asmsolver
