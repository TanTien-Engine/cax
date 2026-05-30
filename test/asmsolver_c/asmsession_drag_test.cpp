// AsmSession interactive pick + drag test (Increment 6).
//
// Loads scissor_lift.FCStd through cadcvt::AsmSession, picks the topmost
// body by a downward ray, drags it upward, and checks that:
//   (a) the constraints stayed satisfied (small joint residual),
//   (b) the dragged body actually rose,
//   (c) a kinematically-coupled body also moved (closed loop propagated),
//   (d) the grounded body stayed put.
//
// Links cax (AsmSession + reader + replayer) + OCCT + OGDF, same surface as
// asmsolver_import_test.

#include "cadcvt_c/AsmSession.h"
#include "brepkit_c/TopoShape.h"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

struct Vec3 { double x = 0, y = 0, z = 0; };

bool center_of(const cadcvt::AsmSession& s, int body, Vec3& out)
{
    auto sh = s.body_shape(body);
    if (!sh || sh->GetShape().IsNull()) return false;
    Bnd_Box box;
    BRepBndLib::Add(sh->GetShape(), box);
    if (box.IsVoid()) return false;
    double xmin, ymin, zmin, xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    out.x = 0.5 * (xmin + xmax);
    out.y = 0.5 * (ymin + ymax);
    out.z = 0.5 * (zmin + zmax);
    return true;
}

double dist(const Vec3& a, const Vec3& b)
{
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace

int main()
{
    cadcvt::AsmSession s;
    if (!s.Load(ASM_SCISSOR_FCSTD, 0.001, false)) {
        std::printf("FAIL: load: %s\n", s.last_error().c_str());
        return 1;
    }

    int n = s.body_count();
    std::printf("bodies = %d\n", n);
    if (n <= 0) { std::printf("FAIL: no bodies\n"); return 1; }

    // Imported centers; pick the topmost body (max Z) as the drag target.
    std::vector<Vec3> c0(n);
    int top = -1;
    for (int i = 0; i < n; ++i) {
        if (!center_of(s, i, c0[i])) { std::printf("FAIL: no shape for body %d\n", i); return 1; }
        std::printf("  body %2d %-16s center=(% .4f % .4f % .4f)\n",
                    i, s.body_name(i).c_str(), c0[i].x, c0[i].y, c0[i].z);
        if (top < 0 || c0[i].z > c0[top].z) top = i;
    }
    std::printf("top body = %d (%s)\n", top, s.body_name(top).c_str());

    // (a) Pick: cast a ray straight down through the top body's centre.
    int picked = s.Pick(c0[top].x, c0[top].y, c0[top].z + 1.0, 0.0, 0.0, -1.0);
    std::printf("pick straight-down through top -> body %d\n", picked);
    if (picked < 0) { std::printf("FAIL: pick missed\n"); return 1; }

    // (b) Drag the top body up by ~8 mm and re-solve.
    double target_z = c0[top].z + 0.008;
    double resid = s.Drag(top, c0[top].x, c0[top].y, target_z, 0.5);
    std::printf("drag residual = %.3e\n", resid);
    if (!(resid < 1e-3)) { std::printf("FAIL: constraints broke (residual too big)\n"); return 1; }

    // Post-drag centres + per-body displacement.
    std::vector<Vec3> c1(n);
    double max_disp = 0.0, min_disp = 1e30, dragged_dz = 0.0;
    int movers = 0;
    for (int i = 0; i < n; ++i) {
        if (!center_of(s, i, c1[i])) { std::printf("FAIL: no shape for body %d post-drag\n", i); return 1; }
        double d = dist(c0[i], c1[i]);
        if (d > max_disp) max_disp = d;
        if (d < min_disp) min_disp = d;
        if (d > 1e-4) ++movers;
        if (i == top) dragged_dz = c1[i].z - c0[i].z;
        std::printf("  body %2d disp=% .5f  dz=% .5f\n", i, d, c1[i].z - c0[i].z);
    }

    // (b) the dragged body rose meaningfully.
    if (!(dragged_dz > 5e-4)) {
        std::printf("FAIL: dragged body did not rise (dz=%.5f)\n", dragged_dz);
        return 1;
    }
    // (c) a coupled body also moved (more than just the dragged one).
    if (movers < 2) {
        std::printf("FAIL: no coupled motion (only %d body moved)\n", movers);
        return 1;
    }
    // (d) something stayed put (the grounded body): min displacement ~ 0.
    if (!(min_disp < 1e-4)) {
        std::printf("FAIL: nothing stayed grounded (min disp=%.5f)\n", min_disp);
        return 1;
    }

    std::printf("PASS: dragged dz=%.4f, movers=%d, max=%.4f, min=%.6f, residual=%.2e\n",
                dragged_dz, movers, max_disp, min_disp, resid);
    return 0;
}
