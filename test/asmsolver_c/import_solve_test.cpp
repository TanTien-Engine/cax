// End-to-end: real FreeCadReader -> DocumentIR -> asmsolver IR adapter ->
// solve. Proves the adapter consumes actual reader output (not hand data)
// and the joint graph it builds is consistent with FreeCAD's solved
// configuration.
//
// Expectation: every joint residual ~0 at the imported poses EXCEPT the
// one plane-to-cylinder Distance (Distance001), whose true constraint
// needs the cylinder radius from the BREP -- the documented Increment-3
// gap. A perturbed re-solve then converges.

#include "asmsolver_c/AsmSolver.h"
#include "asmsolver_c/IrAdapter.h"
#include "cadcvt_c/reader/FreeCadReader.h"
#include "cadapp_c/ir/FeatureIR.h"

#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>
#include <random>

#ifndef ASM_SCISSOR_FCSTD
#define ASM_SCISSOR_FCSTD "scissor_lift.FCStd"
#endif

int main(int argc, char** argv)
{
    const char* path = (argc > 1) ? argv[1] : ASM_SCISSOR_FCSTD;

    cadcvt::FreeCadReader reader;
    cadapp::DocumentIR    doc;
    std::string           err;
    if (!reader.ReadFile(path, doc, &err)) {
        std::printf("reader failed: %s\n", err.c_str());
        return 2;
    }

    asmsolver::ImportResult R = asmsolver::BuildAssembly(doc);
    std::printf("imported: %zu bodies, %d joints built, %d skipped\n",
                R.assembly.bodies.size(), R.joints_built, R.joints_skipped);
    for (size_t i = 0; i < R.body_names.size(); ++i) {
        const auto& p = R.assembly.bodies[i];
        std::printf("  body[%zu] %-8s t=(%.4f,%.4f,%.4f)\n",
                    i, R.body_names[i].c_str(), p.t[0], p.t[1], p.t[2]);
    }

    // residual at the imported (FreeCAD-solved) configuration
    std::printf("joint residuals at imported poses (m):\n");
    int n_ok = 0, n_gap = 0;
    const double kTol = 1e-4;  // 0.1 mm
    for (const auto& j : R.assembly.joints) {
        double r = asmsolver::JointResidualNorm(R.assembly, j);
        bool ok = r < kTol;
        if (ok) ++n_ok; else ++n_gap;
        std::printf("  %-26s |r| = %.3e  %s\n", j.name.c_str(), r,
                    ok ? "ok" : "<-- radius gap (Inc3)");
    }

    // perturb non-grounded bodies, re-solve
    asmsolver::Assembly imported = R.assembly;   // keep for drift compare
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> U(-1, 1);
    for (size_t i = 0; i < R.assembly.bodies.size(); ++i) {
        // skip the grounded body (index of its joint_ground_part)
        bool grounded = false;
        for (const auto& j : R.assembly.joints)
            if (j.grounded && j.body_a == (int)i) grounded = true;
        if (grounded) continue;
        for (int k = 0; k < 3; ++k) R.assembly.bodies[i].t[k] += 0.0015 * U(rng);
        Eigen::Quaterniond dq(Eigen::AngleAxisd(
            0.05 * U(rng), Eigen::Vector3d(U(rng), U(rng), U(rng)).normalized()));
        Eigen::Map<Eigen::Quaterniond> q(R.assembly.bodies[i].q.data());
        q = (dq * q).normalized();
    }
    asmsolver::SolveResult sr = asmsolver::Solve(R.assembly);
    std::printf("re-solve: %s iters=%d init=%.3e final=%.3e\n",
                sr.termination.c_str(), sr.iterations,
                sr.initial_residual, sr.final_residual);

    // success: adapter rebuilt the graph (6 bodies, 8 joints, none skipped)
    // and exactly the documented plane-cylinder Distance is the lone gap.
    bool pass = (R.assembly.bodies.size() == 6) &&
                (R.joints_built == 8) && (R.joints_skipped == 0) &&
                (n_ok == 7) && (n_gap == 1) &&
                (sr.termination == "CONVERGENCE");
    std::printf("RESULT: %s  (%d ok / %d gap)\n", pass ? "PASS" : "FAIL",
                n_ok, n_gap);
    return pass ? 0 : 1;
}
