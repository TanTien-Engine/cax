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
#include <fstream>
#include <string>
#include <vector>

namespace {

struct Vec3 { double x = 0, y = 0, z = 0; };

bool copy_file(const std::string& from, const std::string& to)
{
    std::ifstream in(from, std::ios::binary);
    if (!in) return false;
    std::ofstream out(to, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out << in.rdbuf();
    return out.good();
}

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

    // (e) release-snap: a handle-free re-solve drops the soft-handle residual
    // to ~0 while preserving the dragged (lifted) configuration.
    double snap_resid = s.Snap();
    std::printf("snap residual = %.3e\n", snap_resid);
    if (!(snap_resid < 1e-6)) {
        std::printf("FAIL: snap did not settle onto the manifold (%.3e)\n", snap_resid);
        return 1;
    }
    // The lift must survive the snap (snap settles, it must NOT collapse back).
    std::vector<Vec3> c2(n);
    if (!center_of(s, top, c2[top])) { std::printf("FAIL: no top shape post-snap\n"); return 1; }
    double snap_top_dz = c2[top].z - c0[top].z;
    double snap_shift  = dist(c1[top], c2[top]);
    std::printf("post-snap: top dz=%.5f (was %.5f), shift=%.5f\n",
                snap_top_dz, dragged_dz, snap_shift);
    if (!(snap_top_dz > 5e-4)) {
        std::printf("FAIL: snap collapsed the lift (dz=%.5f)\n", snap_top_dz);
        return 1;
    }
    // The grounded base stays put across the snap too.
    Vec3 base2;
    if (center_of(s, 0, base2) && dist(c0[0], base2) > 1e-4) {
        std::printf("FAIL: snap moved the grounded base (disp=%.5f)\n", dist(c0[0], base2));
        return 1;
    }

    // (f) rotation path: re-pick the top body and rotate it about world Y via
    // the session DragRot binding. The scissor platform is orientation-
    // constrained, so this just checks the path runs and stays solvable (the
    // rotation math is proven decisively by scissor_test ok4). Snap recovers.
    int rep = s.Pick(c2[top].x, c2[top].y, c2[top].z + 1.0, 0.0, 0.0, -1.0);
    if (rep < 0) { std::printf("FAIL: re-pick for rotate missed\n"); return 1; }
    double rot_resid = s.DragRot(rep, 0.0, 1.0, 0.0, 0.3, 0.5);   // 0.3 rad about Y
    std::printf("rotate(session): re-pick body %d, residual=%.3e\n", rep, rot_resid);
    if (!(rot_resid >= 0.0 && rot_resid < 1e-1)) {
        std::printf("FAIL: DragRot path errored or diverged (%.3e)\n", rot_resid);
        return 1;
    }
    double snap2 = s.Snap();
    if (!(snap2 < 1e-6)) {
        std::printf("FAIL: snap after rotate did not settle (%.3e)\n", snap2);
        return 1;
    }

    // (g) write-back round trip -- operate on a COPY so the fixture is never
    // modified. Drag a fresh session, SaveBack (overwrite copy + .bak), reload
    // the saved file, and confirm the dragged poses persisted.
    const std::string copy = "/tmp/asm_save_test.FCStd";
    std::remove(copy.c_str());                 // clean slate: no stale artifacts
    std::remove((copy + ".bak").c_str());
    std::remove((copy + ".tmp").c_str());
    if (!copy_file(ASM_SCISSOR_FCSTD, copy)) {
        std::printf("FAIL: could not copy fixture for write test\n"); return 1;
    }
    cadcvt::AsmSession w;
    if (!w.Load(copy, 0.001, false)) {
        std::printf("FAIL: load copy: %s\n", w.last_error().c_str()); return 1;
    }
    int wn = w.body_count();
    std::vector<Vec3> wc0(wn);
    int wtop = -1;
    for (int i = 0; i < wn; ++i) {
        if (!center_of(w, i, wc0[i])) { std::printf("FAIL: copy body %d\n", i); return 1; }
        if (wtop < 0 || wc0[i].z > wc0[wtop].z) wtop = i;
    }
    if (w.Pick(wc0[wtop].x, wc0[wtop].y, wc0[wtop].z + 1.0, 0.0, 0.0, -1.0) < 0) {
        std::printf("FAIL: write-test pick missed\n"); return 1;
    }
    w.Drag(wtop, wc0[wtop].x, wc0[wtop].y, wc0[wtop].z + 0.006, 0.5);
    w.Snap();
    std::vector<Vec3> wdrag(wn);
    for (int i = 0; i < wn; ++i) center_of(w, i, wdrag[i]);

    if (!w.SaveBack()) {
        std::printf("FAIL: SaveBack: %s\n", w.last_error().c_str()); return 1;
    }
    { std::ifstream bak(copy + ".bak", std::ios::binary);
      if (!bak) { std::printf("FAIL: backup .bak not created\n"); return 1; } }

    cadcvt::AsmSession w2;
    if (!w2.Load(copy, 0.001, false)) {
        std::printf("FAIL: reload saved: %s\n", w2.last_error().c_str()); return 1;
    }
    if (w2.body_count() != wn) { std::printf("FAIL: body count changed by save\n"); return 1; }
    double maxe = 0.0;
    for (int i = 0; i < wn; ++i) {
        Vec3 c; center_of(w2, i, c);
        double e = dist(c, wdrag[i]);
        if (e > maxe) maxe = e;
    }
    std::printf("write round-trip: max center error after save+reload = %.3e\n", maxe);
    if (!(maxe < 1e-6)) {
        std::printf("FAIL: poses did not persist through SaveBack\n"); return 1;
    }

    // (h) DOF drive: a slider value lifts the highest body monotonically along
    // +Z (the linkage follows) and returns to baseline at value 0.
    cadcvt::AsmSession dv;
    if (!dv.Load(ASM_SCISSOR_FCSTD, 0.001, false)) {
        std::printf("FAIL: load for drive test: %s\n", dv.last_error().c_str()); return 1;
    }
    int dn = dv.body_count(), dtop = -1;
    std::vector<Vec3> dc(dn);
    for (int i = 0; i < dn; ++i) {
        if (!center_of(dv, i, dc[i])) { std::printf("FAIL: drive body %d\n", i); return 1; }
        if (dtop < 0 || dc[i].z > dc[dtop].z) dtop = i;
    }
    Vec3 z0, z5, z10, zb;
    dv.Drive(dtop, 0.0, 0.0, 1.0, 0.000, 0.5); center_of(dv, dtop, z0);
    dv.Drive(dtop, 0.0, 0.0, 1.0, 0.005, 0.5); center_of(dv, dtop, z5);
    dv.Drive(dtop, 0.0, 0.0, 1.0, 0.010, 0.5); center_of(dv, dtop, z10);
    double dres = dv.Drive(dtop, 0.0, 0.0, 1.0, 0.000, 0.5); center_of(dv, dtop, zb);
    std::printf("drive: top z0=%.4f z5=%.4f z10=%.4f z(back)=%.4f back_resid=%.2e\n",
                z0.z, z5.z, z10.z, zb.z, dres);
    if (!(z5.z > z0.z + 5e-4 && z10.z > z5.z + 5e-4)) {
        std::printf("FAIL: drive did not lift monotonically\n"); return 1;
    }
    if (!(std::abs(zb.z - z0.z) < 5e-4)) {
        std::printf("FAIL: drive value 0 did not return to baseline\n"); return 1;
    }
    if (!(dres < 1e-3)) {
        std::printf("FAIL: drive-to-baseline left a large residual (%.2e)\n", dres); return 1;
    }
    // auto-pick (body < 0) drives the same highest body.
    if (dv.Drive(-1, 0.0, 0.0, 1.0, 0.005, 0.5) < 0.0) {
        std::printf("FAIL: drive auto-pick errored\n"); return 1;
    }

    std::printf("PASS: dragged dz=%.4f, movers=%d, max=%.4f, min=%.6f, drag_resid=%.2e, "
                "snap_resid=%.2e, snap_top_dz=%.4f\n",
                dragged_dz, movers, max_disp, min_disp, resid, snap_resid, snap_top_dz);
    return 0;
}
