#include "cadcvt_c/store/SketchBridge.h"

#include "sketchlib/Scene.h"
#include "sketchlib/Constraint.h"
#include "sketchlib/Geometry.h"

#include <geoshape/Point2D.h>
#include <geoshape/Line2D.h>
#include <geoshape/Circle.h>
#include <geoshape/Arc.h>
#include <geoshape/Ellipse.h>

#include <SM_Vector.h>

#include <cmath>
#include <sstream>

// ============================================================
// SketchBridge.cpp
//
// Section A : cadcvt::SkConsType <-> sketchlib::ConsType mapping
// Section B : SkGeoIR::params -> gs::Shape2D construction
// Section C : ImportToScene (primary path)
// Section D : ExportFromScene
// Section E : EmitVes
// ============================================================

namespace cadcvt
{

namespace
{

// ---- Section A: enum mapping ----

sketchlib::ConsType MapConsType(SkConsType t)
{
    using S = sketchlib::ConsType;
    switch (t)
    {
    case SkConsType::Distance:            return S::Distance;
    case SkConsType::DistanceX:           return S::DistanceX;
    case SkConsType::DistanceY:           return S::DistanceY;
    case SkConsType::Angle:               return S::Angle;
    case SkConsType::Parallel:            return S::Parallel;
    case SkConsType::Perpendicular:       return S::Perpendicular;
    case SkConsType::Coincident:          return S::Coincident;
    case SkConsType::Horizontal:          return S::Horizontal;
    case SkConsType::Vertical:            return S::Vertical;
    case SkConsType::Equal:               return S::Equal;
    case SkConsType::PointOnLine:         return S::PointOnLine;
    case SkConsType::PointOnCircle:       return S::PointOnCircle;
    case SkConsType::PointOnArc:          return S::PointOnArc;
    case SkConsType::PointOnEllipse:      return S::PointOnEllipse;
    case SkConsType::PointOnPerpBisector: return S::PointOnPerpBisector;
    case SkConsType::MidpointOnLine:      return S::MidpointOnLine;
    case SkConsType::Tangent:             return S::Tangent;
    case SkConsType::TangentCircumf:      return S::TangentCircumf;
    case SkConsType::CircleRadius:        return S::CircleRadius;
    case SkConsType::CircleDiameter:      return S::CircleDiameter;
    case SkConsType::ArcRadius:           return S::ArcRadius;
    case SkConsType::ArcDiameter:         return S::ArcDiameter;
    // Symmetric / Concentric / Colinear / Fix have no direct
    // sketchlib counterpart yet; callers can pre-expand them or
    // accept that they are dropped here.
    default:                              return S::None;
    }
}

// SkPointPos + geo type -> sketchlib::Geo (pair<GeoID, GeoType>)
sketchlib::Geo MakeGeo(uint32_t geo_id, SkPointPos pos, SkGeoType geo_type)
{
    if (geo_id == 0xFFFFFFFF) {
        return { -1, sketchlib::GeoType::None };
    }

    sketchlib::GeoType t = sketchlib::GeoType::None;
    switch (pos)
    {
    case SkPointPos::None:
        // Whole-geometry reference; pick by geo_type.
        switch (geo_type)
        {
        case SkGeoType::Point:   t = sketchlib::GeoType::Point;   break;
        case SkGeoType::Line:    t = sketchlib::GeoType::Line;    break;
        case SkGeoType::Arc:     t = sketchlib::GeoType::Arc;     break;
        case SkGeoType::Circle:  t = sketchlib::GeoType::Circle;  break;
        case SkGeoType::Ellipse: t = sketchlib::GeoType::Ellipse; break;
        default:                 t = sketchlib::GeoType::None;    break;
        }
        break;
    case SkPointPos::Start:  t = sketchlib::GeoType::GeoPtStart; break;
    case SkPointPos::Mid:    t = sketchlib::GeoType::GeoPtMid;   break;
    case SkPointPos::End:    t = sketchlib::GeoType::GeoPtEnd;   break;
    // Treat Center as Mid (sketchlib uses Mid for arc/circle centers).
    case SkPointPos::Center: t = sketchlib::GeoType::GeoPtMid;   break;
    }

    return { (int)geo_id, t };
}

// ---- Section B: params -> gs::Shape2D ----

SketchBridge::GeoShape BuildShape(const SkGeoEntry& g, const double* pool)
{
    const double*  p = pool + g.param_offset;
    const uint32_t n = g.param_count;

    switch ((SkGeoType)g.type)
    {
    case SkGeoType::Point:
    {
        if (n < 2) {
            return nullptr;
        }
        return std::make_shared<gs::Point2D>(sm::vec2((float)p[0], (float)p[1]));
    }
    case SkGeoType::Line:
    {
        if (n < 4) {
            return nullptr;
        }
        return std::make_shared<gs::Line2D>(
            sm::vec2((float)p[0], (float)p[1]),
            sm::vec2((float)p[2], (float)p[3]));
    }
    case SkGeoType::Circle:
    {
        if (n < 3) {
            return nullptr;
        }
        return std::make_shared<gs::Circle>(
            sm::vec2((float)p[0], (float)p[1]),
            (float)p[2]);
    }
    case SkGeoType::Arc:
    {
        if (n < 5) {
            return nullptr;
        }
        return std::make_shared<gs::Arc>(
            sm::vec2((float)p[0], (float)p[1]),
            (float)p[2],
            (float)p[3],
            (float)p[4]);
    }
    case SkGeoType::Ellipse:
    {
        // [cx, cy, major_r, minor_r] (simple form without axis angle)
        if (n < 4) {
            return nullptr;
        }
        return std::make_shared<gs::Ellipse>(
            sm::vec2((float)p[0], (float)p[1]),
            (float)p[2],
            (float)p[3]);
    }
    case SkGeoType::Spline:
        // sketchlib doesn't solve splines; only emitted via ves.
        return nullptr;
    default:
        return nullptr;
    }
}

// IR variant where params live in a std::vector.
SketchBridge::GeoShape BuildShapeFromIR(const SkGeoIR& g)
{
    SkGeoEntry e{};
    e.type         = (uint8_t)g.type;
    e.construction = g.construction ? 1u : 0u;
    e.param_offset = 0;
    e.param_count  = (uint32_t)g.params.size();
    return BuildShape(e, g.params.data());
}

// Look up a geometry's type by id. Needed when a constraint
// references it (Line vs Circle decides the sketchlib::GeoType).
SkGeoType FindGeoType(const SketchStore& store,
                      uint32_t           sketch_idx,
                      uint32_t           geo_id)
{
    uint32_t n = 0;
    const SkGeoEntry* geos = store.GetGeos(sketch_idx, n);
    for (uint32_t i = 0; i < n; ++i)
    {
        if (geos[i].id == geo_id) {
            return (SkGeoType)geos[i].type;
        }
    }
    return SkGeoType::None;
}

SkGeoType FindGeoType(const SketchIR& sk, uint32_t geo_id)
{
    for (const auto& g : sk.geos)
    {
        if (g.id == geo_id) {
            return g.type;
        }
    }
    return SkGeoType::None;
}

const char* ConsNodeName(SkConsType t)
{
    switch (t)
    {
    case SkConsType::Distance:            return "Distance";
    case SkConsType::DistanceX:           return "DistanceX";
    case SkConsType::DistanceY:           return "DistanceY";
    case SkConsType::Angle:               return "Angle";
    case SkConsType::Parallel:            return "Parallel";
    case SkConsType::Perpendicular:       return "Perpendicular";
    case SkConsType::Coincident:          return "Coincident";
    case SkConsType::Horizontal:          return "Horizontal";
    case SkConsType::Vertical:            return "Vertical";
    case SkConsType::Equal:               return "Equal";
    case SkConsType::PointOnLine:         return "PointOnLine";
    case SkConsType::PointOnCircle:       return "PointOnCircle";
    case SkConsType::PointOnArc:          return "PointOnArc";
    case SkConsType::PointOnEllipse:      return "PointOnEllipse";
    case SkConsType::PointOnPerpBisector: return "PointOnPerpBisector";
    case SkConsType::MidpointOnLine:      return "MidpointOnLine";
    case SkConsType::Tangent:             return "Tangent";
    case SkConsType::TangentCircumf:      return "TangentCircumf";
    case SkConsType::CircleRadius:        return "CircleRadius";
    case SkConsType::CircleDiameter:      return "CircleDiameter";
    case SkConsType::ArcRadius:           return "ArcRadius";
    case SkConsType::ArcDiameter:         return "ArcDiameter";
    default:                              return nullptr;
    }
}

bool HasValueParam(SkConsType t)
{
    switch (t)
    {
    case SkConsType::Distance:
    case SkConsType::DistanceX:
    case SkConsType::DistanceY:
    case SkConsType::Angle:
    case SkConsType::CircleRadius:
    case SkConsType::CircleDiameter:
    case SkConsType::ArcRadius:
    case SkConsType::ArcDiameter:
        return true;
    default:
        return false;
    }
}

const char* ValueParamName(SkConsType t)
{
    switch (t)
    {
    case SkConsType::Distance:
    case SkConsType::DistanceX:
    case SkConsType::DistanceY:
        return "dist";
    case SkConsType::Angle:
        return "angle";
    case SkConsType::CircleRadius:
    case SkConsType::ArcRadius:
        return "r";
    case SkConsType::CircleDiameter:
    case SkConsType::ArcDiameter:
        return "d";
    default:
        return nullptr;
    }
}

} // anonymous namespace


// ============================================================
// Section C: ImportToScene
// ============================================================

bool SketchBridge::ImportToScene(const SketchStore& store,
                                 uint32_t           sketch_idx,
                                 sketchlib::Scene&  out_scene,
                                 GeoShapes&         out_geos)
{
    if (sketch_idx >= store.SketchCount()) {
        return false;
    }

    uint32_t          geo_n = 0;
    const SkGeoEntry* geos  = store.GetGeos(sketch_idx, geo_n);
    const double*     pool  = store.GetParamPool();

    out_geos.clear();
    out_geos.reserve(geo_n);

    for (uint32_t i = 0; i < geo_n; ++i)
    {
        const auto& g = geos[i];
        // Construction lines are excluded from the solver (project
        // convention).
        if (g.construction) {
            continue;
        }

        auto shape = BuildShape(g, pool);
        if (!shape) {
            continue;
        }

        out_scene.AddGeometry((int)g.id, shape);
        out_geos.emplace_back((int)g.id, shape);
    }

    uint32_t           cons_n = 0;
    const SkConsEntry* cons   = store.GetCons(sketch_idx, cons_n);
    for (uint32_t i = 0; i < cons_n; ++i)
    {
        const auto& c        = cons[i];
        auto        skl_type = MapConsType((SkConsType)c.type);
        if (skl_type == sketchlib::ConsType::None) {
            continue;
        }

        auto a_geo_type = FindGeoType(store, sketch_idx, c.a_geo_id);
        auto b_geo_type = FindGeoType(store, sketch_idx, c.b_geo_id);

        auto a = MakeGeo(c.a_geo_id, (SkPointPos)c.a_point_pos, a_geo_type);
        auto b = MakeGeo(c.b_geo_id, (SkPointPos)c.b_point_pos, b_geo_type);

        out_scene.AddConstraint((int)c.id, skl_type, a, b, c.value, c.driving != 0);
    }

    return true;
}

bool SketchBridge::ImportToScene(const SketchIR&   sketch,
                                 sketchlib::Scene& out_scene,
                                 GeoShapes&        out_geos)
{
    out_geos.clear();
    out_geos.reserve(sketch.geos.size());

    for (const auto& g : sketch.geos)
    {
        if (g.construction) {
            continue;
        }
        auto shape = BuildShapeFromIR(g);
        if (!shape) {
            continue;
        }
        out_scene.AddGeometry((int)g.id, shape);
        out_geos.emplace_back((int)g.id, shape);
    }

    for (const auto& c : sketch.cons)
    {
        auto skl_type = MapConsType(c.type);
        if (skl_type == sketchlib::ConsType::None) {
            continue;
        }

        auto a_geo_type = FindGeoType(sketch, c.a.geo_id);
        auto b_geo_type = FindGeoType(sketch, c.b.geo_id);

        auto a = MakeGeo(c.a.geo_id, c.a.point_pos, a_geo_type);
        auto b = MakeGeo(c.b.geo_id, c.b.point_pos, b_geo_type);

        out_scene.AddConstraint((int)c.id, skl_type, a, b, c.value, c.driving);
    }
    return true;
}


// ============================================================
// Section D: ExportFromScene
// ============================================================
// After Solve the gs::Shape2D state is already updated; readers
// can pull params straight off the shapes. This helper is a stub
// for the day SketchStore exposes a writeable param view.

bool SketchBridge::ExportFromScene(const GeoShapes& solved_geos,
                                   SketchStore&     /*store*/,
                                   uint32_t         /*sketch_idx*/)
{
    // TODO: add SketchStore::UpdateGeoParams(...) and write back
    // here. Until then, callers can read the solved coordinates
    // off solved_geos directly, since the shape objects are the
    // ones ImportToScene constructed.
    (void)solved_geos;
    return true;
}


// ============================================================
// Section E: EmitVes
// ============================================================

namespace
{

void EmitGeoNode(std::ostringstream& oss,
                 int&                node_idx,
                 SkGeoType           type,
                 const double*       params,
                 uint32_t            n,
                 bool                construction)
{
    oss << "var node" << node_idx
        << " = ::sketchgraph::nodes::"
        << (type == SkGeoType::Line     ? "line::Line"
          : type == SkGeoType::Arc      ? "arc::Arc"
          : type == SkGeoType::Circle   ? "circle::Circle"
          : type == SkGeoType::Ellipse  ? "ellipse::Ellipse"
          : type == SkGeoType::Point    ? "point::Point"
          :                                "polyline::Polyline")
        << "()\n";

    switch (type)
    {
    case SkGeoType::Line:
        if (n >= 4)
        {
            oss << "node" << node_idx
                << ".query_param(\"p1\").value.set("
                << params[0] << ", " << params[1] << ")\n";
            oss << "node" << node_idx
                << ".query_param(\"p2\").value.set("
                << params[2] << ", " << params[3] << ")\n";
        }
        break;

    case SkGeoType::Circle:
        if (n >= 3)
        {
            oss << "node" << node_idx
                << ".query_param(\"center\").value.set("
                << params[0] << ", " << params[1] << ")\n";
            oss << "node" << node_idx
                << ".query_param(\"radius\").value = "
                << params[2] << "\n";
        }
        break;

    case SkGeoType::Arc:
        if (n >= 5)
        {
            oss << "node" << node_idx
                << ".query_param(\"center\").value.set("
                << params[0] << ", " << params[1] << ")\n";
            oss << "node" << node_idx
                << ".query_param(\"radius\").value = "
                << params[2] << "\n";
            oss << "node" << node_idx
                << ".query_param(\"start_angle\").value = "
                << params[3] << "\n";
            oss << "node" << node_idx
                << ".query_param(\"end_angle\").value = "
                << params[4] << "\n";
        }
        break;

    case SkGeoType::Ellipse:
        if (n >= 4)
        {
            oss << "node" << node_idx
                << ".query_param(\"center\").value.set("
                << params[0] << ", " << params[1] << ")\n";
            oss << "node" << node_idx
                << ".query_param(\"rx\").value = "
                << params[2] << "\n";
            oss << "node" << node_idx
                << ".query_param(\"ry\").value = "
                << params[3] << "\n";
        }
        break;

    case SkGeoType::Point:
        if (n >= 2)
        {
            oss << "node" << node_idx
                << ".query_param(\"pos\").value.set("
                << params[0] << ", " << params[1] << ")\n";
        }
        break;

    default:
        break;
    }

    if (construction)
    {
        oss << "node" << node_idx
            << ".query_param(\"construction\").value = true\n";
    }

    oss << "_editor.add_node(node" << node_idx
        << ", " << (node_idx * 200) << ", 0)\n\n";
}

} // anonymous namespace

std::string SketchBridge::EmitVes(const SketchStore& store, uint32_t sketch_idx)
{
    SketchIR sk;
    if (!store.ExportToIR(sketch_idx, sk)) { return {}; }
    return EmitVes(sk);
}

std::string SketchBridge::EmitVes(const SketchIR& sketch)
{
    std::ostringstream oss;
    oss << "// auto-generated by cadcvt::SketchBridge::EmitVes\n";
    oss << "// sketch: " << sketch.name << "\n";
    oss << "// plane origin = ("
        << sketch.plane_origin[0] << ", "
        << sketch.plane_origin[1] << ", "
        << sketch.plane_origin[2] << ")\n";
    oss << "// plane normal = ("
        << sketch.plane_normal[0] << ", "
        << sketch.plane_normal[1] << ", "
        << sketch.plane_normal[2] << ")\n\n";

    int node_idx = 0;
    // (geo_id -> node_idx) so constraint nodes can wire inputs.
    std::vector<std::pair<uint32_t, int>> geo_id_to_node;
    geo_id_to_node.reserve(sketch.geos.size());

    for (const auto& g : sketch.geos)
    {
        EmitGeoNode(oss,
                    node_idx,
                    g.type,
                    g.params.data(),
                    (uint32_t)g.params.size(),
                    g.construction);
        geo_id_to_node.emplace_back(g.id, node_idx);
        ++node_idx;
    }

    auto find_node = [&](uint32_t gid) -> int
    {
        for (auto& kv : geo_id_to_node)
        {
            if (kv.first == gid) {
                return kv.second;
            }
        }
        return -1;
    };

    for (const auto& c : sketch.cons)
    {
        const char* name = ConsNodeName(c.type);
        if (!name) {
            continue;
        }

        oss << "var node" << node_idx
            << " = ::sketchgraph::nodes::cons_nodes::" << name << "()\n";
        oss << "node" << node_idx
            << ".query_param(\"driving\").value = "
            << (c.driving ? "true" : "false") << "\n";

        if (HasValueParam(c.type))
        {
            oss << "node" << node_idx
                << ".query_param(\"" << ValueParamName(c.type)
                << "\").value = " << c.value << "\n";
        }

        oss << "_editor.add_node(node" << node_idx
            << ", " << (node_idx * 200) << ", 300)\n";

        int na = find_node(c.a.geo_id);
        int nb = find_node(c.b.geo_id);
        if (na >= 0)
        {
            oss << "Tree.connect(node" << node_idx
                << ", \"a\", node" << na << ", \"geo\")\n";
        }
        if (nb >= 0)
        {
            oss << "Tree.connect(node" << node_idx
                << ", \"b\", node" << nb << ", \"geo\")\n";
        }

        oss << "\n";
        ++node_idx;
    }
    return oss.str();
}

} // namespace cadcvt
