#include "wrap_GCSGen.h"
#include "Constraint.h"
#include "Scene.h"
#include "modules/script/Proxy.h"
#include "modules/script/TransHelper.h"

#include <geoshape/Point2D.h>
#include <geoshape/Line2D.h>

#include <string>

namespace
{

gcsgen::GeoType get_geo_type(const std::shared_ptr<gs::Shape2D>& shape)
{
    if (!shape) {
        return gcsgen::GeoType::None;
    }

    auto type = shape->GetType();
    if (type == gs::ShapeType2D::Point) {
        return gcsgen::GeoType::Point;
    } else if (type == gs::ShapeType2D::Line) {
        return gcsgen::GeoType::Line;
    } else if (type == gs::ShapeType2D::Circle) {
        return gcsgen::GeoType::Circle;
    } else {
        return gcsgen::GeoType::None;
    }
}

void w_Scene_allocate()
{
    auto proxy = (tt::Proxy<gcsgen::Scene>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<gcsgen::Scene>));
    proxy->obj = std::make_shared<gcsgen::Scene>();
}

int w_Scene_finalize(void* data)
{
    auto proxy = (tt::Proxy<gcsgen::Scene>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<gcsgen::Scene>);
}

void w_Scene_add()
{
    auto scene = ((tt::Proxy<gcsgen::Scene>*)ves_toforeign(0))->obj;

    int cons_id = (int)ves_tonumber(1);
    const char* cons_type_s = ves_tostring(2);
    gcsgen::ConsType cons_type = gcsgen::ConsType::None;
    if (strcmp(cons_type_s, "distance") == 0) {
        cons_type = gcsgen::ConsType::Distance;
    } else if (strcmp(cons_type_s, "angle") == 0) {
        cons_type = gcsgen::ConsType::Angle;
    } else if (strcmp(cons_type_s, "point_on_line") == 0) {
        cons_type = gcsgen::ConsType::PointOnLine;
    } else if (strcmp(cons_type_s, "point_on_perp_bisector") == 0) {
        cons_type = gcsgen::ConsType::PointOnPerpBisector;
    } else if (strcmp(cons_type_s, "midpoint_on_line") == 0) {
        cons_type = gcsgen::ConsType::MidpointOnLine;
    } else if (strcmp(cons_type_s, "parallel") == 0) {
        cons_type = gcsgen::ConsType::Parallel;
    } else if (strcmp(cons_type_s, "perpendicular") == 0) {
        cons_type = gcsgen::ConsType::Perpendicular;
    } else if (strcmp(cons_type_s, "tangent_circumf") == 0) {
        cons_type = gcsgen::ConsType::TangentCircumf;
    } else if (strcmp(cons_type_s, "coincident") == 0) {
        cons_type = gcsgen::ConsType::Coincident;
    } else if (strcmp(cons_type_s, "horizontal") == 0) {
        cons_type = gcsgen::ConsType::Horizontal;
    } else if (strcmp(cons_type_s, "vertical") == 0) {
        cons_type = gcsgen::ConsType::Vertical;
    } else if (strcmp(cons_type_s, "point_on_circle") == 0) {
        cons_type = gcsgen::ConsType::PointOnCircle;
    } else if (strcmp(cons_type_s, "tangent") == 0) {
        cons_type = gcsgen::ConsType::Tangent;
    }

    std::shared_ptr<gs::Shape2D> shape1 = nullptr, shape2 = nullptr;

    int id1 = (int)ves_tonumber(3);
    auto shape1_w = ves_toforeign(4);
    if (shape1_w) 
    {
        shape1 = ((tt::Proxy<gs::Shape2D>*)shape1_w)->obj;
        scene->AddGeometry(id1, shape1);
    }

    int id2 = (int)ves_tonumber(5);
    auto shape2_w = ves_toforeign(6);
    if (shape2_w) 
    {
        shape2 = ((tt::Proxy<gs::Shape2D>*)shape2_w)->obj;
        scene->AddGeometry(id2, shape2);
    }

    double val = ves_tonumber(7);

    auto type1 = get_geo_type(shape1);
    auto type2 = get_geo_type(shape2);

    scene->AddConstraint(cons_id, cons_type, std::make_pair(id1, type1), std::make_pair(id2, type2), val);
}

void w_Scene_clear()
{
    auto scene = ((tt::Proxy<gcsgen::Scene>*)ves_toforeign(0))->obj;
    scene->Clear();
}

void w_Scene_solve()
{
    auto scene = ((tt::Proxy<gcsgen::Scene>*)ves_toforeign(0))->obj;

    auto ids = tt::list_to_array<int>(1);

    std::vector<std::shared_ptr<gs::Shape2D>> shapes;
    tt::list_to_foreigns(2, shapes);

    std::vector<std::pair<gcsgen::GeoID, std::shared_ptr<gs::Shape2D>>> geos;
    for (int i = 0, n = ids.size(); i < n; ++i) {
        geos.push_back(std::make_pair(ids[i], shapes[i]));
    }

    bool success = scene->Solve(geos);
    ves_set_boolean(0, success);
}

}

namespace gcsgen
{

VesselForeignMethodFn GcsGenBindMethod(const char* signature)
{
    if (strcmp(signature, "GCS_Scene.add(_,_,_,_,_,_,_)") == 0) return w_Scene_add;
    if (strcmp(signature, "GCS_Scene.clear()") == 0) return w_Scene_clear;

    if (strcmp(signature, "GCS_Scene.solve(_,_)") == 0) return w_Scene_solve;

    return nullptr;
}

void GcsGenBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
    if (strcmp(class_name, "GCS_Scene") == 0)
    {
        methods->allocate = w_Scene_allocate;
        methods->finalize = w_Scene_finalize;
        return;
    }
}

}