#pragma once

#include "cadapp_c/ir/Enums.h"

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

// ============================================================
// cadapp/ir/SketchIR.h
//
// CAD-kernel independent sketch IR.
//
//   SketchIR
//     name, plane (origin / normal / x_dir)
//     geos: 2D geometries in sketch-local coords
//     cons: constraints referring to geos through SkGeoRef
//
//   SkGeoIR::params layout by type:
//     Point   : [x, y]
//     Line    : [x1, y1, x2, y2]
//     Arc     : [cx, cy, radius, start_angle, end_angle]
//     Circle  : [cx, cy, radius]
//     Ellipse : [cx, cy, major_r, minor_r]                  (simple form)
//               [cx, cy, mx, my, minor_r]                   (form with major-axis endpoint)
//     Spline  : [n, x0, y0, x1, y1, ..., xn-1, yn-1]
// ============================================================

namespace cadapp
{

struct SkGeoRef
{
    // geo_id == 0xFFFFFFFF marks the unused slot of a unary constraint.
    uint32_t   geo_id    = 0xFFFFFFFF;
    SkPointPos point_pos = SkPointPos::None;
};

struct SkGeoIR
{
    uint32_t            id           = 0;
    SkGeoType           type         = SkGeoType::None;
    bool                construction = false;
    std::vector<double> params;

    // ---- builders ----
    // Hide the params layout behind named factories. Reader / test
    // code should call these instead of writing the params vector
    // by hand; that keeps the param order in one place.

    static SkGeoIR Point(uint32_t id, double x, double y, bool construction = false)
    {
        SkGeoIR g;
        g.id           = id;
        g.type         = SkGeoType::Point;
        g.construction = construction;
        g.params       = { x, y };
        return g;
    }

    static SkGeoIR Line(uint32_t id,
                        double x1, double y1,
                        double x2, double y2,
                        bool construction = false)
    {
        SkGeoIR g;
        g.id           = id;
        g.type         = SkGeoType::Line;
        g.construction = construction;
        g.params       = { x1, y1, x2, y2 };
        return g;
    }

    static SkGeoIR Arc(uint32_t id,
                       double cx, double cy,
                       double radius,
                       double start_angle, double end_angle,
                       bool construction = false)
    {
        SkGeoIR g;
        g.id           = id;
        g.type         = SkGeoType::Arc;
        g.construction = construction;
        g.params       = { cx, cy, radius, start_angle, end_angle };
        return g;
    }

    static SkGeoIR Circle(uint32_t id,
                          double cx, double cy,
                          double radius,
                          bool construction = false)
    {
        SkGeoIR g;
        g.id           = id;
        g.type         = SkGeoType::Circle;
        g.construction = construction;
        g.params       = { cx, cy, radius };
        return g;
    }

    static SkGeoIR Ellipse(uint32_t id,
                           double cx, double cy,
                           double major_r, double minor_r,
                           bool construction = false)
    {
        SkGeoIR g;
        g.id           = id;
        g.type         = SkGeoType::Ellipse;
        g.construction = construction;
        g.params       = { cx, cy, major_r, minor_r };
        return g;
    }

    // Spline params: [N, x0, y0, x1, y1, ..., x(N-1), y(N-1)].
    // N is the number of control points.
    static SkGeoIR Spline(uint32_t id,
                          const std::vector<double>& xs,
                          const std::vector<double>& ys,
                          bool construction = false)
    {
        SkGeoIR g;
        g.id           = id;
        g.type         = SkGeoType::Spline;
        g.construction = construction;

        uint32_t n = (uint32_t)std::min(xs.size(), ys.size());
        g.params.reserve(1 + 2 * n);
        g.params.push_back((double)n);
        for (uint32_t i = 0; i < n; ++i)
        {
            g.params.push_back(xs[i]);
            g.params.push_back(ys[i]);
        }
        return g;
    }
};

struct SkConsIR
{
    uint32_t   id      = 0;
    SkConsType type    = SkConsType::None;
    SkGeoRef   a;
    SkGeoRef   b;             // unary constraints leave b at default (None)
    double     value   = 0.0; // distance / angle / radius etc.
    bool       driving = true;
};

struct SketchIR
{
    // feature_id locates the parent Sketch-type FeatureIR in the
    // same DocumentIR.
    uint32_t    feature_id = 0xFFFFFFFF;
    std::string name;

    // Sketch plane in 3D:
    //   origin = origin point
    //   normal = Z axis (sketch plane normal)
    //   x_dir  = local X axis
    // Y direction is implicit: normal x x_dir.
    double plane_origin[3] = { 0.0, 0.0, 0.0 };
    double plane_normal[3] = { 0.0, 0.0, 1.0 };
    double plane_x_dir [3] = { 1.0, 0.0, 0.0 };

    std::vector<SkGeoIR>  geos;
    std::vector<SkConsIR> cons;
};

} // namespace cadapp
