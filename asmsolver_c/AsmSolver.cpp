#include "asmsolver_c/AsmSolver.h"

#include <ceres/ceres.h>
#include <Eigen/Geometry>

#include <cmath>

namespace asmsolver {
namespace {

// Relative JCS transform: given world body poses (tA,qA)/(tB,qB) and the
// fixed connector frames cA=(ta,qa)/cB=(tb,qb) in each body's local
// coords, compute T_rel = FrameA^-1 * FrameB where FrameX = poseX * cX.
// Outputs the relative translation, the image of B's Z axis in A's frame
// (for axis-alignment residuals), and the relative rotation quaternion
// (for full-rotation residuals).
template <typename T>
void RelFrame(const T* tA, const T* qA, const double* ta, const double* qa,
              const T* tB, const T* qB, const double* tb, const double* qb,
              Eigen::Matrix<T, 3, 1>& t_rel,
              Eigen::Matrix<T, 3, 1>& zB,
              Eigen::Quaternion<T>&   q_rel)
{
    using Vec  = Eigen::Matrix<T, 3, 1>;
    using Quat = Eigen::Quaternion<T>;
    Eigen::Map<const Quat> QA(qA), QB(qB);
    Eigen::Map<const Vec>  TA(tA), TB(tB);
    // connector frames as scalars (avoid most-vexing-parse)
    T aw = T(qa[3]), ax = T(qa[0]), ay = T(qa[1]), az = T(qa[2]);
    T bw = T(qb[3]), bx = T(qb[0]), by = T(qb[1]), bz = T(qb[2]);
    T at0 = T(ta[0]), at1 = T(ta[1]), at2 = T(ta[2]);
    T bt0 = T(tb[0]), bt1 = T(tb[1]), bt2 = T(tb[2]);
    Quat QCA(aw, ax, ay, az);  Vec TCA(at0, at1, at2);
    Quat QCB(bw, bx, by, bz);  Vec TCB(bt0, bt1, bt2);

    Quat FAq = QA * QCA;  Vec FAt = QA * TCA + TA;
    Quat FBq = QB * QCB;  Vec FBt = QB * TCB + TB;

    Quat FAqc = FAq.conjugate();
    t_rel = FAqc * (FBt - FAt);
    q_rel = FAqc * FBq;
    zB    = q_rel * Vec(T(0), T(0), T(1));
}

// Binary joint residual: always 6 components (unused ones padded with 0
// so a single AutoDiff residual size covers every kind). Constrained
// components are zeroed by the solver; free DOF are simply not listed.
struct JointCost {
    int    kind;
    double ca_t[3], ca_q[4], cb_t[3], cb_q[4];
    double distance, radius;

    template <typename T>
    bool operator()(const T* tA, const T* qA,
                    const T* tB, const T* qB, T* r) const
    {
        Eigen::Matrix<T, 3, 1> t_rel, zB;
        Eigen::Quaternion<T>   q_rel;
        RelFrame(tA, qA, ca_t, ca_q, tB, qB, cb_t, cb_q, t_rel, zB, q_rel);
        for (int i = 0; i < 6; ++i) r[i] = T(0);

        // full relative rotation as a small-angle vector (lock all rot)
        const T rvx = T(2) * q_rel.x();
        const T rvy = T(2) * q_rel.y();
        const T rvz = T(2) * q_rel.z();

        switch (static_cast<JointKind>(kind)) {
        case JointKind::Fixed:
            r[0]=t_rel[0]; r[1]=t_rel[1]; r[2]=t_rel[2];
            r[3]=rvx;      r[4]=rvy;      r[5]=rvz;            break;
        case JointKind::Revolute:                              // free: Z spin
            r[0]=t_rel[0]; r[1]=t_rel[1]; r[2]=t_rel[2];
            r[3]=zB[0];    r[4]=zB[1];                         break;
        case JointKind::Cylindrical:                  // free: Z spin + slide
            r[0]=t_rel[0]; r[1]=t_rel[1];
            r[2]=zB[0];    r[3]=zB[1];                         break;
        case JointKind::Slider:                             // free: Z slide
            r[0]=t_rel[0]; r[1]=t_rel[1];
            r[2]=rvx;      r[3]=rvy;      r[4]=rvz;            break;
        case JointKind::Ball:                          // free: all rotation
            r[0]=t_rel[0]; r[1]=t_rel[1]; r[2]=t_rel[2];      break;
        case JointKind::Planar:               // free: XY slide + Z spin
            r[0]=t_rel[2]; r[1]=zB[0]; r[2]=zB[1];            break;
        case JointKind::Distance:
            if (radius > 0.0) {                 // plane-to-cylinder
                // plane sits `distance` outside the cylinder surface, so
                // plane-to-axis = radius + distance (measured along the
                // plane normal = connector Z).
                r[0]=t_rel[2] - T(radius + distance);
                r[1]=zB[2];                     // axis lies in the plane
            } else {                            // plane-to-plane
                r[0]=t_rel[2] - T(distance);
                r[1]=zB[0]; r[2]=zB[1];
            }
            break;
        }
        return true;
    }
};

// Unary grounding: pin body's WORLD pose to a target (tg,qg).
struct GroundedCost {
    double tg[3], qg[4];
    template <typename T>
    bool operator()(const T* t, const T* q, T* r) const
    {
        r[0] = t[0] - T(tg[0]);
        r[1] = t[1] - T(tg[1]);
        r[2] = t[2] - T(tg[2]);
        Eigen::Map<const Eigen::Quaternion<T>> Q(q);
        T gw = T(qg[3]), gx = T(qg[0]), gy = T(qg[1]), gz = T(qg[2]);
        Eigen::Quaternion<T> QG(gw, gx, gy, gz);
        Eigen::Quaternion<T> e = QG.conjugate() * Q;
        r[3] = T(2) * e.x();
        r[4] = T(2) * e.y();
        r[5] = T(2) * e.z();
        return true;
    }
};

JointCost MakeJointCost(const Joint& j)
{
    JointCost c;
    c.kind = static_cast<int>(j.kind);
    for (int i = 0; i < 3; ++i) { c.ca_t[i] = j.conn_a.t[i]; c.cb_t[i] = j.conn_b.t[i]; }
    for (int i = 0; i < 4; ++i) { c.ca_q[i] = j.conn_a.q[i]; c.cb_q[i] = j.conn_b.q[i]; }
    c.distance = j.distance;
    c.radius   = j.radius;
    return c;
}

} // namespace

double JointResidualNorm(const Assembly& A, const Joint& j)
{
    double r[6] = {0, 0, 0, 0, 0, 0};
    if (j.grounded) {
        GroundedCost g;
        for (int i = 0; i < 3; ++i) g.tg[i] = j.conn_a.t[i];
        for (int i = 0; i < 4; ++i) g.qg[i] = j.conn_a.q[i];
        const Pose& b = A.bodies[j.body_a];
        g(b.t.data(), b.q.data(), r);
    } else {
        JointCost c = MakeJointCost(j);
        const Pose& a = A.bodies[j.body_a];
        const Pose& b = A.bodies[j.body_b];
        c(a.t.data(), a.q.data(), b.t.data(), b.q.data(), r);
    }
    double s = 0;
    for (int i = 0; i < 6; ++i) s += r[i] * r[i];
    return std::sqrt(s);
}

SolveResult Solve(Assembly& A, const SolveOptions& opt)
{
    SolveResult out;

    // initial residual
    {
        double s = 0;
        for (const auto& j : A.joints) {
            double n = JointResidualNorm(A, j);
            s += n * n;
        }
        out.initial_residual = std::sqrt(s);
    }

    ceres::Problem prob;
    for (const auto& j : A.joints) {
        if (j.grounded) {
            if (j.body_a < 0) continue;
            auto* g = new GroundedCost;
            for (int i = 0; i < 3; ++i) g->tg[i] = j.conn_a.t[i];
            for (int i = 0; i < 4; ++i) g->qg[i] = j.conn_a.q[i];
            prob.AddResidualBlock(
                new ceres::AutoDiffCostFunction<GroundedCost, 6, 3, 4>(g),
                nullptr,
                A.bodies[j.body_a].t.data(), A.bodies[j.body_a].q.data());
        } else {
            if (j.body_a < 0 || j.body_b < 0) continue;
            auto* c = new JointCost(MakeJointCost(j));
            prob.AddResidualBlock(
                new ceres::AutoDiffCostFunction<JointCost, 6, 3, 4, 3, 4>(c),
                nullptr,
                A.bodies[j.body_a].t.data(), A.bodies[j.body_a].q.data(),
                A.bodies[j.body_b].t.data(), A.bodies[j.body_b].q.data());
        }
    }
    // quaternion blocks live on the unit-quaternion manifold
    for (auto& b : A.bodies) {
        if (prob.HasParameterBlock(b.q.data())) {
            prob.SetManifold(b.q.data(), new ceres::EigenQuaternionManifold);
        }
    }

    ceres::Solver::Options sopt;
    sopt.linear_solver_type           = ceres::DENSE_QR;
    sopt.max_num_iterations           = opt.max_iterations;
    sopt.minimizer_progress_to_stdout = opt.verbose;
    ceres::Solver::Summary sum;
    ceres::Solve(sopt, &prob, &sum);

    out.iterations  = static_cast<int>(sum.iterations.size());
    out.termination = ceres::TerminationTypeToString(sum.termination_type);

    double s = 0;
    out.joint_residuals.reserve(A.joints.size());
    for (const auto& j : A.joints) {
        double n = JointResidualNorm(A, j);
        out.joint_residuals.push_back(n);
        s += n * n;
    }
    out.final_residual = std::sqrt(s);
    out.converged = (sum.termination_type == ceres::CONVERGENCE) &&
                    (out.final_residual < 1e-4);
    return out;
}

} // namespace asmsolver
