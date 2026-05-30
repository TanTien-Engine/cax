// asmsolver test: rebuild scissor_lift's native Assembly WB joint graph
// and confirm (1) FreeCAD's ground-truth poses satisfy every joint and
// (2) Ceres re-solves a perturbed start back to a valid configuration.
//
// Data is the same set validated in the standalone PoC, taken from
// scissor_lift.FCStd (mm, quaternion x,y,z,w). scissor lift is a 1-DOF
// mechanism, so a small perturbation returns near ground truth; the
// residual ~0 is the real correctness signal.

#include "asmsolver_c/AsmSolver.h"

#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>
#include <random>

using asmsolver::Assembly;
using asmsolver::Joint;
using asmsolver::JointKind;
using asmsolver::Pose;

namespace {

Pose P(double x, double y, double z,
       double qx, double qy, double qz, double qw)
{
    Pose p;
    p.t = {{x, y, z}};
    p.q = {{qx, qy, qz, qw}};
    return p;
}

// connector with identity rotation
Pose C(double x, double y, double z) { return P(x, y, z, 0, 0, 0, 1); }

Assembly BuildScissorLift()
{
    Assembly A;
    // ground-truth Body world placements (mm)
    A.bodies = {
        P(  0.0000,  1.5000,  1.5000,  0.70711, 0,       0,        0.70711), // base
        P(  6.3360,  1.5000, 25.8610,  0.17799, 0.68434, 0.68434, -0.17799), // arm_1
        P( -5.1624,  4.5000, 46.4704, -0.68434,-0.17799,-0.17799,  0.68434), // arm_2
        P( 50.0000,  7.5000,  1.5000,  0.70490,-0.05578, 0.05578,  0.70490), // pin_1
        P(-28.4890, -0.0000, 96.4365,  0.37333, 0.60052, 0.60052, -0.37333), // pin_2
        P( 28.0818, -1.5000, 45.2637, -0.67660,-0.20547,-0.20547,  0.67660), // pin_3
    };

    auto J = [](JointKind k, int a, int b, Pose ca, Pose cb,
                const char* nm, double dist = 0, double rad = 0) {
        Joint j; j.kind = k; j.body_a = a; j.body_b = b;
        j.conn_a = ca; j.conn_b = cb; j.distance = dist; j.radius = rad;
        j.name = nm; return j;
    };

    // GroundedJoint: pin base to its world pose
    Joint g; g.kind = JointKind::Fixed; g.grounded = true; g.body_a = 0;
    g.conn_a = P(0, 1.5, 1.5, 0.70711, 0, 0, 0.70711); g.name = "Grounded(base)";
    A.joints.push_back(g);

    A.joints.push_back(J(JointKind::Revolute,    3, 0, C(0,0,9.0),   C(50,0,3),    "Revolute pin_1-base"));
    A.joints.push_back(J(JointKind::Cylindrical, 3, 1, C(0,0,4.5),   C(-50,0,1.5), "Cylindrical pin_1-arm_1"));
    A.joints.push_back(J(JointKind::Distance,    1, 0, C(0.6208,0,0),C(1.8665,0,0),"Distance arm_1-base", 0.0));
    A.joints.push_back(J(JointKind::Revolute,    4, 1, C(-78.7,0,4.5),C(0,0,3),    "Revolute001 pin_2-arm_1"));
    A.joints.push_back(J(JointKind::Revolute,    2, 4, C(0,23.6,0),  C(-78.7,0,4.5),"Revolute002 arm_2-pin_2"));
    A.joints.push_back(J(JointKind::Revolute,    5, 2, C(-78.7,0,9.0),C(-50,23.6,3),"Revolute003 pin_3-arm_2"));
    // Distance001 base.Face6 (plane) <-> pin_3.Face1 (cylinder, r=1.5mm from
    // the BREP); plane sits d=0.25mm outside the surface -> axis at r+d=1.75.
    A.joints.push_back(J(JointKind::Distance,    0, 5,
                         P(-4,-1.75,1.5, -0.70711,0,0,0.70711), C(-78.7,0,4.5),
                         "Distance001 base-pin_3", 0.25, 1.5));
    return A;
}

} // namespace

int main()
{
    Assembly A = BuildScissorLift();

    // ---- (1) ground-truth must satisfy every joint ----
    std::printf("(1) residual at FreeCAD ground truth:\n");
    double worst = 0;
    for (const auto& j : A.joints) {
        double n = asmsolver::JointResidualNorm(A, j);
        worst = std::max(worst, n);
        std::printf("    %-26s |r| = %.6g\n", j.name.c_str(), n);
    }
    bool ok1 = worst < 1e-3;
    std::printf("    worst = %.6g  -> %s\n\n", worst, ok1 ? "PASS" : "FAIL");

    // ---- (2) perturb non-grounded bodies, re-solve ----
    Assembly G = A;  // keep ground truth for comparison
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> U(-1, 1);
    for (size_t i = 1; i < A.bodies.size(); ++i) {
        for (int k = 0; k < 3; ++k) A.bodies[i].t[k] += 1.5 * U(rng);
        Eigen::Quaterniond dq(Eigen::AngleAxisd(
            0.05 * U(rng),
            Eigen::Vector3d(U(rng), U(rng), U(rng)).normalized()));
        Eigen::Map<Eigen::Quaterniond> q(A.bodies[i].q.data());
        q = (dq * q).normalized();
    }

    asmsolver::SolveOptions opt;
    asmsolver::SolveResult res = asmsolver::Solve(A, opt);

    std::printf("(2) Ceres re-solve from perturbed start:\n");
    std::printf("    %s  iters=%d  init_residual=%.4g final_residual=%.6g\n",
                res.termination.c_str(), res.iterations,
                res.initial_residual, res.final_residual);
    bool ok2 = res.converged;
    std::printf("    converged = %s\n", ok2 ? "PASS" : "FAIL");

    std::printf("    per-body translation drift vs ground truth (mm):\n");
    const char* nm[6] = {"base","arm_1","arm_2","pin_1","pin_2","pin_3"};
    for (size_t i = 0; i < A.bodies.size(); ++i) {
        double e = std::sqrt(
            std::pow(A.bodies[i].t[0]-G.bodies[i].t[0], 2) +
            std::pow(A.bodies[i].t[1]-G.bodies[i].t[1], 2) +
            std::pow(A.bodies[i].t[2]-G.bodies[i].t[2], 2));
        std::printf("      %-6s |dt| = %.4f\n", nm[i], e);
    }
    std::printf("    (1-DOF mechanism: residual~0 = valid config; sub-mm drift "
                "= the free scissor DOF)\n\n");

    // ---- (3) mobility / constraint diagnostics ----
    // The constraint Jacobian has a clean rank gap (32 singular values
    // ~O(1), then 4 at ~1e-16). mobility = 36 - 32 = 4: ONE useful lift DOF
    // plus THREE idle DOF (each round pin spins freely about its own axis
    // through its Revolute/Cylindrical joints -- a passive freedom). The 3
    // redundant equations are the planar-Distance parallelism, already
    // pinned by the kinematic chain (a real, consistent over-constraint,
    // exactly what FreeCAD would also flag).
    asmsolver::DofInfo dof = asmsolver::AnalyzeDof(G);
    std::printf("(3) DOF analysis at ground truth:\n");
    std::printf("    tangent_dofs=%d rank=%d intended=%d -> mobility=%d redundancy=%d\n",
                dof.tangent_dofs, dof.rank, dof.intended_constraints,
                dof.mobility, dof.redundancy);
    bool ok3 = (dof.tangent_dofs == 36) && (dof.rank == 32) &&
               (dof.mobility == 4) && (dof.redundancy == 3);
    std::printf("    %s (mobility 4 = 1 lift + 3 idle pin spins; 3 redundant)\n\n",
                ok3 ? "PASS" : "FAIL");

    // ---- (4) rotation handle (HandleRotCost) ----
    // Self-contained: drives a body's orientation toward a target quaternion.
    const double PI = 3.14159265358979323846;
    const double s45 = std::sin(PI / 4), c45 = std::cos(PI / 4);  // 90deg
    bool ok4a = false, ok4b = false;

    // (a) free body + pure rotation handle -> orientation snaps to target.
    {
        asmsolver::Assembly R;
        R.bodies.resize(1);                       // identity pose
        asmsolver::Handle h;
        h.body = 0; h.weight = 0.0;               // no translation pull
        h.has_rot = true; h.rot_weight = 1.0;
        h.target_quat = {{0, 0, s45, c45}};       // 90deg about Z
        asmsolver::SolveWithHandles(R, {h});
        const auto& q = R.bodies[0].q;
        double dot = q[0]*0 + q[1]*0 + q[2]*s45 + q[3]*c45;
        ok4a = std::abs(dot) > 0.999999;          // same rotation (q ~ +/-target)
        std::printf("(4a) pure rotation handle: q.target dot=%.8f -> %s\n",
                    dot, ok4a ? "PASS" : "FAIL");
    }

    // (b) rotation about an off-origin anchor -> body spins about the pinned
    // point: q -> target AND pose*anchor stays at the anchor's world position.
    {
        asmsolver::Assembly R;
        R.bodies.resize(1);
        asmsolver::Handle h;
        h.body = 0; h.weight = 1.0;               // pin the anchor
        h.anchor_local = {{1, 0, 0}};
        h.target_world = {{1, 0, 0}};
        h.has_rot = true; h.rot_weight = 1.0;
        h.target_quat = {{0, 0, s45, c45}};
        asmsolver::SolveWithHandles(R, {h});
        const auto& q = R.bodies[0].q;
        const auto& t = R.bodies[0].t;
        Eigen::Quaterniond Q(q[3], q[0], q[1], q[2]);
        Eigen::Vector3d a(1, 0, 0);
        Eigen::Vector3d world = Q * a + Eigen::Vector3d(t[0], t[1], t[2]);
        double pin_err = (world - Eigen::Vector3d(1, 0, 0)).norm();
        double dot = q[2]*s45 + q[3]*c45;
        ok4b = std::abs(dot) > 0.999999 && pin_err < 1e-5;
        std::printf("(4b) rotate about anchor: dot=%.8f pin_err=%.2e -> %s\n",
                    dot, pin_err, ok4b ? "PASS" : "FAIL");
    }
    bool ok4 = ok4a && ok4b;

    bool pass = ok1 && ok2 && ok3 && ok4;
    std::printf("RESULT: %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
