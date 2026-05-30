#include "asmsolver_c/AsmSolver.h"

#include <ceres/ceres.h>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

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

// Unary soft drag handle: pull a body-local anchor toward a world target.
// 3 residuals, scaled by weight (kept low so the hard joints dominate).
struct HandleCost {
    double anchor[3], target[3], w;
    template <typename T>
    bool operator()(const T* t, const T* q, T* r) const
    {
        Eigen::Map<const Eigen::Quaternion<T>> Q(q);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> Tt(t);
        // named temporaries avoid the most-vexing-parse (T(anchor[0]) in a
        // declaration context would be read as a parameter named anchor)
        const T ax = T(anchor[0]), ay = T(anchor[1]), az = T(anchor[2]);
        Eigen::Matrix<T, 3, 1> a(ax, ay, az);
        Eigen::Matrix<T, 3, 1> world = Q * a + Tt;
        r[0] = T(w) * (world[0] - T(target[0]));
        r[1] = T(w) * (world[1] - T(target[1]));
        r[2] = T(w) * (world[2] - T(target[2]));
        return true;
    }
};

// Unary soft orientation handle: pull a body's quaternion toward a target
// world orientation. 3 residuals = the vector part of the relative rotation
// (target^-1 * Q), ~the rotation vector for small angles, scaled by weight.
struct HandleRotCost {
    double target_q[4];   // x,y,z,w
    double w;
    template <typename T>
    bool operator()(const T* q, T* r) const
    {
        Eigen::Map<const Eigen::Quaternion<T>> Q(q);
        // Eigen's quaternion ctor takes (w,x,y,z); target is stored x,y,z,w.
        const T tx = T(target_q[0]), ty = T(target_q[1]),
                tz = T(target_q[2]), tw = T(target_q[3]);
        Eigen::Quaternion<T> Tq(tw, tx, ty, tz);
        Eigen::Quaternion<T> e = Tq.conjugate() * Q;
        r[0] = T(2) * T(w) * e.x();
        r[1] = T(2) * T(w) * e.y();
        r[2] = T(2) * T(w) * e.z();
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

// DOF each joint is meant to remove (its residual rank if non-degenerate).
int ConstraintCount(const Joint& j)
{
    if (j.grounded) return 6;
    switch (j.kind) {
    case JointKind::Fixed:       return 6;
    case JointKind::Revolute:    return 5;
    case JointKind::Cylindrical: return 4;
    case JointKind::Slider:      return 5;
    case JointKind::Ball:        return 3;
    case JointKind::Planar:      return 3;
    case JointKind::Distance:    return j.radius > 0.0 ? 2 : 3;
    }
    return 0;
}

// Add every joint as a residual block, plus any soft drag handles, then put
// each quaternion block on the unit-quaternion manifold. Shared by Solve,
// SolveWithHandles and AnalyzeDof.
void BuildProblem(Assembly& A, ceres::Problem& prob,
                  const std::vector<Handle>& handles = {})
{
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
    for (const auto& h : handles) {
        if (h.body < 0 || h.body >= static_cast<int>(A.bodies.size())) continue;
        auto* hc = new HandleCost;
        for (int i = 0; i < 3; ++i) { hc->anchor[i] = h.anchor_local[i];
                                      hc->target[i] = h.target_world[i]; }
        hc->w = h.weight;
        prob.AddResidualBlock(
            new ceres::AutoDiffCostFunction<HandleCost, 3, 3, 4>(hc),
            nullptr,
            A.bodies[h.body].t.data(), A.bodies[h.body].q.data());

        if (h.has_rot) {
            auto* rc = new HandleRotCost;
            for (int i = 0; i < 4; ++i) rc->target_q[i] = h.target_quat[i];
            rc->w = h.rot_weight;
            prob.AddResidualBlock(
                new ceres::AutoDiffCostFunction<HandleRotCost, 3, 4>(rc),
                nullptr,
                A.bodies[h.body].q.data());
        }
    }
    for (auto& b : A.bodies) {
        if (prob.HasParameterBlock(b.q.data())) {
            prob.SetManifold(b.q.data(), new ceres::EigenQuaternionManifold);
        }
    }
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

namespace {

SolveResult SolveImpl(Assembly& A, const std::vector<Handle>& handles,
                      const SolveOptions& opt)
{
    SolveResult out;

    // initial residual (joints only -- handles are not constraints)
    {
        double s = 0;
        for (const auto& j : A.joints) {
            double n = JointResidualNorm(A, j);
            s += n * n;
        }
        out.initial_residual = std::sqrt(s);
    }

    ceres::Problem prob;
    BuildProblem(A, prob, handles);

    // Grounded bodies are TRULY fixed during the solve: pin them at their
    // grounding pose and freeze the parameter blocks. The GroundedCost
    // residual alone is only a soft penalty, so a soft drag handle could
    // otherwise tug the base by a fraction of a millimetre; freezing the
    // block makes the ground immovable regardless of handle strength.
    // (AnalyzeDof keeps the soft model -- it calls BuildProblem directly --
    // so its mobility/rank counting is unchanged.)
    for (const auto& j : A.joints) {
        if (!j.grounded || j.body_a < 0) continue;
        A.bodies[j.body_a].t = j.conn_a.t;
        A.bodies[j.body_a].q = j.conn_a.q;
        if (prob.HasParameterBlock(A.bodies[j.body_a].t.data()))
            prob.SetParameterBlockConstant(A.bodies[j.body_a].t.data());
        if (prob.HasParameterBlock(A.bodies[j.body_a].q.data()))
            prob.SetParameterBlockConstant(A.bodies[j.body_a].q.data());
    }

    ceres::Solver::Options sopt;
    sopt.linear_solver_type           = ceres::DENSE_QR;
    sopt.max_num_iterations           = opt.max_iterations;
    sopt.minimizer_progress_to_stdout = opt.verbose;
    // Silence Ceres' per-iteration glog spam unless explicitly asked --
    // this runs once per interactive drag frame.
    sopt.logging_type = opt.verbose ? ceres::PER_MINIMIZER_ITERATION
                                    : ceres::SILENT;
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

} // namespace

SolveResult Solve(Assembly& A, const SolveOptions& opt)
{
    return SolveImpl(A, {}, opt);
}

SolveResult SolveWithHandles(Assembly& A, const std::vector<Handle>& handles,
                             const SolveOptions& opt)
{
    return SolveImpl(A, handles, opt);
}

DofInfo AnalyzeDof(const Assembly& assembly)
{
    DofInfo info;
    Assembly A = assembly;   // local copy: Ceres needs mutable param pointers
    info.tangent_dofs = 6 * static_cast<int>(A.bodies.size());
    for (const auto& j : A.joints) info.intended_constraints += ConstraintCount(j);

    // Non-dimensionalize: scale all lengths so translations are O(1),
    // commensurate with the dimensionless tilt residuals. Without this the
    // Jacobian's metre-scale translation rows vs O(1) rotation rows are so
    // ill-scaled that rank-revealing SVD undercounts (mobility inflated).
    double Lc = 0.0;
    for (const auto& b : A.bodies)
        Lc = std::max(Lc, std::sqrt(b.t[0]*b.t[0] + b.t[1]*b.t[1] + b.t[2]*b.t[2]));
    const double S = (Lc > 1e-9) ? (1.0 / Lc) : 1.0;
    for (auto& b : A.bodies) for (int k = 0; k < 3; ++k) b.t[k] *= S;
    for (auto& j : A.joints) {
        for (int k = 0; k < 3; ++k) { j.conn_a.t[k] *= S; j.conn_b.t[k] *= S; }
        j.distance *= S;
        j.radius   *= S;
    }

    ceres::Problem prob;
    BuildProblem(A, prob);

    ceres::Problem::EvaluateOptions eopt;
    ceres::CRSMatrix jac;
    double cost = 0.0;
    if (!prob.Evaluate(eopt, &cost, nullptr, nullptr, &jac) ||
        jac.num_rows == 0 || jac.num_cols == 0) {
        info.mobility = info.tangent_dofs;   // no constraints resolved
        return info;
    }

    // CRS -> dense (the Jacobian is in the manifold tangent space, so
    // num_cols == tangent_dofs).
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(jac.num_rows, jac.num_cols);
    for (int r = 0; r < jac.num_rows; ++r)
        for (int idx = jac.rows[r]; idx < jac.rows[r + 1]; ++idx)
            J(r, jac.cols[idx]) = jac.values[idx];

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    const auto& sv = svd.singularValues();
    double tol = (sv.size() ? sv(0) : 0.0) * 1e-6;
    int rank = 0;
    for (int i = 0; i < sv.size(); ++i) if (sv(i) > tol) ++rank;
    if (std::getenv("ASM_DEBUG_DOF")) {
        std::printf("[dof] %dx%d S=%g..%g tol=%g sv:", jac.num_rows,
                    jac.num_cols, sv(sv.size()-1), sv(0), tol);
        for (int i = 0; i < sv.size(); ++i) std::printf(" %.3g", sv(i));
        std::printf("\n");
    }

    info.rank       = rank;
    info.mobility   = info.tangent_dofs - rank;
    info.redundancy = info.intended_constraints - rank;
    return info;
}

} // namespace asmsolver
