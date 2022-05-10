#include "wrap_SketchLib.h"
#include "Constraint.h"
#include "Scene.h"
#include "modules/script/Proxy.h"
#include "modules/script/TransHelper.h"

#include <geoshape/Point2D.h>
#include <geoshape/Line2D.h>

#include <string>

namespace
{

sketchlib::GeoType to_geo_type(const std::shared_ptr<gs::Shape2D>& shape)
{
    if (!shape) {
        return sketchlib::GeoType::None;
    }

    auto type = shape->GetType();
    if (type == gs::ShapeType2D::Point) {
        return sketchlib::GeoType::Point;
    } else if (type == gs::ShapeType2D::Line) {
        return sketchlib::GeoType::Line;
    } else if (type == gs::ShapeType2D::Circle) {
        return sketchlib::GeoType::Circle;
    } else if (type == gs::ShapeType2D::Arc) {
        return sketchlib::GeoType::Arc;
    } else if (type == gs::ShapeType2D::Ellipse) {
        return sketchlib::GeoType::Ellipse;
    } else {
        return sketchlib::GeoType::None;
    }
}

void w_Scene_allocate()
{
    auto proxy = (tt::Proxy<sketchlib::Scene>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<sketchlib::Scene>));
    proxy->obj = std::make_shared<sketchlib::Scene>();
}

int w_Scene_finalize(void* data)
{
    auto proxy = (tt::Proxy<sketchlib::Scene>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<sketchlib::Scene>);
}

sketchlib::ConsType to_cons_type(int idx)
{
    sketchlib::ConsType type = sketchlib::ConsType::None;

    const char* type_s = ves_tostring(idx);
    // basic
    if (strcmp(type_s, "distance") == 0) {
        type = sketchlib::ConsType::Distance;
    } else if (strcmp(type_s, "distance_x") == 0) {
        type = sketchlib::ConsType::DistanceX;
    } else if (strcmp(type_s, "distance_y") == 0) {
        type = sketchlib::ConsType::DistanceY;
    } else if (strcmp(type_s, "angle") == 0) {
        type = sketchlib::ConsType::Angle;
    } else if (strcmp(type_s, "parallel") == 0) {
        type = sketchlib::ConsType::Parallel;
    } else if (strcmp(type_s, "perpendicular") == 0) {
        type = sketchlib::ConsType::Perpendicular;
    } else if (strcmp(type_s, "coincident") == 0) {
        type = sketchlib::ConsType::Coincident;
    } else if (strcmp(type_s, "horizontal") == 0) {
        type = sketchlib::ConsType::Horizontal;
    } else if (strcmp(type_s, "vertical") == 0) {
        type = sketchlib::ConsType::Vertical;
    } else if (strcmp(type_s, "equal") == 0) {
        type = sketchlib::ConsType::Equal;
    }
    // point on
    else if (strcmp(type_s, "point_on_line") == 0) {
        type = sketchlib::ConsType::PointOnLine;
    } else if (strcmp(type_s, "point_on_circle") == 0) {
        type = sketchlib::ConsType::PointOnCircle;
    } else if (strcmp(type_s, "point_on_arc") == 0) {
        type = sketchlib::ConsType::PointOnArc;
    } else if (strcmp(type_s, "point_on_ellipse") == 0) {
        type = sketchlib::ConsType::PointOnEllipse;
    } else if (strcmp(type_s, "point_on_perp_bisector") == 0) {
        type = sketchlib::ConsType::PointOnPerpBisector;
    } else if (strcmp(type_s, "midpoint_on_line") == 0) {
        type = sketchlib::ConsType::MidpointOnLine;
    } 
    // tangent
    else if (strcmp(type_s, "tangent") == 0) {
        type = sketchlib::ConsType::Tangent;
    } else if (strcmp(type_s, "tangent_circumf") == 0) {
        type = sketchlib::ConsType::TangentCircumf;
    }
    // params
    else if (strcmp(type_s, "circle_radius") == 0) {
        type = sketchlib::ConsType::CircleRadius;
    } else if (strcmp(type_s, "circle_diameter") == 0) {
        type = sketchlib::ConsType::CircleDiameter;
    } else if (strcmp(type_s, "arc_radius") == 0) {
        type = sketchlib::ConsType::ArcRadius;
    } else if (strcmp(type_s, "arc_diameter") == 0) {
        type = sketchlib::ConsType::ArcDiameter;
    }

    return type;
}

std::pair<sketchlib::GeoID, sketchlib::GeoType> to_geo_info(int idx, const std::shared_ptr<sketchlib::Scene>& scene)
{
    int geo_id = 0;
    if (ves_getfield(idx, "geo_id") == VES_TYPE_NUM) {
        geo_id = (int)ves_tonumber(-1);
    }
    ves_pop(1);

    int point_id = 0;
    if (ves_getfield(idx, "point_id") == VES_TYPE_NUM) {
        point_id = (int)ves_tonumber(-1);
    }
    ves_pop(1);

    std::shared_ptr<gs::Shape2D> shape = nullptr;
    if (ves_getfield(idx, "shape") == VES_TYPE_FOREIGN) {
        shape = ((tt::Proxy<gs::Shape2D>*)ves_toforeign(-1))->obj;
    }
    ves_pop(1);

    scene->AddGeometry(geo_id, shape);

    sketchlib::GeoType geo_type = sketchlib::GeoType::None;
    if (point_id > 0) 
    {
        geo_type = sketchlib::GeoType::None;
        switch (point_id)
        {
        case 1:
            geo_type = sketchlib::GeoType::GeoPtStart;
            break;
        case 2:
            geo_type = sketchlib::GeoType::GeoPtMid;
            break;
        case 3:
            geo_type = sketchlib::GeoType::GeoPtEnd;
            break;
        }
    } 
    else 
    {
        geo_type = to_geo_type(shape);
    }

    return std::make_pair(geo_id, geo_type);
}

void w_Scene_add_cons_2()
{
    auto scene = ((tt::Proxy<sketchlib::Scene>*)ves_toforeign(0))->obj;

    int cons_id = (int)ves_tonumber(1);
    auto cons_type = to_cons_type(2);
    auto geo1 = to_geo_info(3, scene);
    auto geo2 = to_geo_info(4, scene);
    double val = ves_tonumber(5);
    bool driving = ves_toboolean(6);

    scene->AddConstraint(cons_id, cons_type, geo1, geo2, val, driving);
}

void w_Scene_add_cons_4()
{
    auto scene = ((tt::Proxy<sketchlib::Scene>*)ves_toforeign(0))->obj;

    int cons_id = (int)ves_tonumber(1);
    auto cons_type = to_cons_type(2);
    auto geo1 = to_geo_info(3, scene);
    auto geo2 = to_geo_info(4, scene);
    auto geo3 = to_geo_info(5, scene);
    auto geo4 = to_geo_info(6, scene);
    double val = ves_tonumber(7);
    bool driving = ves_toboolean(8);

    scene->AddConstraint(cons_id, cons_type, std::pair(geo1, geo2), std::pair(geo3, geo4), val, driving);
}

void w_Scene_clear()
{
    auto scene = ((tt::Proxy<sketchlib::Scene>*)ves_toforeign(0))->obj;
    scene->Clear();
}

void w_Scene_solve()
{
    auto scene = ((tt::Proxy<sketchlib::Scene>*)ves_toforeign(0))->obj;

    auto ids = tt::list_to_array<int>(1);

    std::vector<std::shared_ptr<gs::Shape2D>> shapes;
    tt::list_to_foreigns(2, shapes);

    std::vector<std::pair<sketchlib::GeoID, std::shared_ptr<gs::Shape2D>>> geos;
    for (int i = 0, n = ids.size(); i < n; ++i) {
        geos.push_back(std::make_pair(ids[i], shapes[i]));
    }

    bool success = scene->Solve(geos);
    ves_set_boolean(0, success);
}

void w_Scene_get_dof()
{
    auto scene = ((tt::Proxy<sketchlib::Scene>*)ves_toforeign(0))->obj;

    int dofs = scene->GetDOF();
    ves_set_number(0, dofs);
}

}

namespace sketchlib
{

VesselForeignMethodFn SketchLibBindMethod(const char* signature)
{
    if (strcmp(signature, "SketchScene.add_cons_2(_,_,_,_,_,_)") == 0) return w_Scene_add_cons_2;
    if (strcmp(signature, "SketchScene.add_cons_4(_,_,_,_,_,_,_,_)") == 0) return w_Scene_add_cons_4;

    if (strcmp(signature, "SketchScene.clear()") == 0) return w_Scene_clear;

    if (strcmp(signature, "SketchScene.solve(_,_)") == 0) return w_Scene_solve;

    if (strcmp(signature, "SketchScene.get_dof()") == 0) return w_Scene_get_dof;

    return nullptr;
}

void SketchLibBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
    if (strcmp(class_name, "SketchScene") == 0)
    {
        methods->allocate = w_Scene_allocate;
        methods->finalize = w_Scene_finalize;
        return;
    }
}

}