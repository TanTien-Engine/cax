#pragma once

// AsmSession -- stateful interactive assembly-editing session.
//
// Load a FreeCAD native-AssemblyWB document, build the asmsolver constraint
// graph ONCE, then pick parts by world ray and drag them: each Drag adds a
// soft handle on the picked body and re-solves, so the constrained parts
// follow according to the joints. Only body POSES change -- each part keeps
// its once-replayed geometry, transformed to the solved pose for rendering.
// Nothing is written back to the source .FCStd (render-only edit loop).
//
// All heavy deps (OCCT / Ceres / cadapp IR) live in the .cpp behind a Pimpl,
// so the binding TU only needs brepkit::TopoShape forward-declared. Compiled
// into cax only when CAX_ASMSOLVER_OK (Ceres submodule present).

#include <memory>
#include <string>

namespace brepkit { class TopoShape; }

namespace cadcvt {

class AsmSession {
public:
    AsmSession();
    ~AsmSession();

    AsmSession(const AsmSession&) = delete;
    AsmSession& operator=(const AsmSession&) = delete;

    // Read + replay + build the constraint assembly. unit_scale follows the
    // FreeCadLoader convention (0.001 = FreeCAD mm -> project metres). Returns
    // true on success; last_error() explains a failure.
    bool Load(const std::string& path, double unit_scale, bool strict);

    const std::string& last_error() const { return m_err; }

    // Number of bodies in the assembly (0 before a successful Load).
    int  body_count() const;
    std::string body_name(int body) const;

    // i-th body's shape at its CURRENT (solved) pose, ready to mesh/render.
    // Null on a bad index or a body with no replayed part.
    std::shared_ptr<brepkit::TopoShape> body_shape(int body) const;

    // Ray-pick the nearest body hit by the world ray (origin + direction).
    // Returns the body index, or -1 on a miss. On a hit the world hit point
    // becomes the grab anchor for the next Drag (also via last_hit()).
    int  Pick(double ox, double oy, double oz,
              double dx, double dy, double dz);
    // World hit point of the last successful Pick (out[3]).
    void last_hit(double out[3]) const;

    // Drag the grabbed anchor on `body` toward a world target and re-solve
    // the constraints (`weight` scales the soft handle). Returns the
    // post-solve joint residual L2 norm (small = constraints still satisfied).
    double Drag(int body, double tx, double ty, double tz, double weight);

    // Rotate `body` by `angle` (radians) about the world axis (ax,ay,az),
    // composed onto its grab-time orientation, while pinning the grabbed point
    // -- so the body spins about the pick point and the joints follow. Returns
    // the post-solve joint residual L2 norm.
    double DragRot(int body, double ax, double ay, double az,
                   double angle, double weight);

    // Release-snap: re-solve with NO drag handle, from the current (dragged)
    // poses, so the assembly settles exactly onto the constraint manifold --
    // the soft handle leaves a small mid-drag residual, this cleans it. The
    // dragged configuration is preserved (Solve converges to the nearest exact
    // config, it has no memory of the imported pose). Call on mouse release.
    // Returns the post-snap joint residual L2 norm (~0 on success).
    double Snap();

    // Write the current solved body poses back into a .FCStd. Save() writes a
    // copy to out_path (the loaded source is untouched). SaveBack() overwrites
    // the loaded source in place after backing it up to <source>.bak. Both
    // return false + set last_error() on failure (and never corrupt the source
    // -- the write goes to a temp file renamed into place only on success).
    bool Save(const std::string& out_path);
    bool SaveBack();

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
    std::string           m_err;
};

} // namespace cadcvt
