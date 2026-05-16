#pragma once

namespace deepbrep
{

// Per-face feature classes recognized by the GNN. Order is load-bearing -- the
// classifier's softmax output uses these indices and trained weights bind to
// this order. Append-only; do not reorder.
enum class FaceClass
{
    Stock   = 0,
    Hole    = 1,
    Slot    = 2,
    Fillet  = 3,
    Chamfer = 4,
    Pocket  = 5,

    Count
};

constexpr int kNumFaceClasses = static_cast<int>(FaceClass::Count);

const char* face_class_name(int cls);

// Node feature layout (NODE_FEAT_DIM = 14):
//   [0..5]  surface_type one-hot: plane, cyl, cone, sphere, torus, bspline
//   [6]     area (normalized)
//   [7..9]  normal direction (x, y, z)
//   [10]    num outer wire edges (normalized)
//   [11]    num inner wires
//   [12..13] curvature stats (mean, std)
constexpr int kNodeFeatDim = 14;

// Edge feature layout (EDGE_FEAT_DIM = 10):
//   [0..4]  curve_type one-hot: line, circle, ellipse, bspline, other
//   [5..7]  convexity one-hot: convex, concave, smooth
//   [8]     dihedral angle (normalized to [0, 1])
//   [9]     edge length (normalized)
constexpr int kEdgeFeatDim = 10;

enum class SurfaceType
{
    Plane   = 0,
    Cylinder = 1,
    Cone    = 2,
    Sphere  = 3,
    Torus   = 4,
    BSpline = 5,
    Count
};

enum class CurveType
{
    Line    = 0,
    Circle  = 1,
    Ellipse = 2,
    BSpline = 3,
    Other   = 4,
    Count
};

enum class Convexity
{
    Convex  = 0,
    Concave = 1,
    Smooth  = 2,
    Count
};

}
