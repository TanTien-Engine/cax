#include "wrap_GCSGen.h"
#include "Constraint.h"
#include "Scene.h"
#include "modules/script/Proxy.h"

#include <geoshape/Point2D.h>
#include <geoshape/Line2D.h>

#include <string>

namespace
{

void w_Constraint_allocate()
{
    const char* type_str = ves_tostring(1);

    std::shared_ptr<gs::Shape2D> geo1 = nullptr, geo2 = nullptr;

    auto w_geo1 = ves_toforeign(2);
    if (w_geo1) {
        geo1 = ((tt::Proxy<gs::Shape2D>*)w_geo1)->obj;
    }

    auto w_geo2 = ves_toforeign(3);
    if (w_geo2) {
        geo2 = ((tt::Proxy<gs::Shape2D>*)w_geo2)->obj;
    }

    double value = ves_tonumber(4);

    gcsgen::ConstraintType type = gcsgen::ConstraintType::None;
    if (strcmp(type_str, "distance") == 0) {
        type = gcsgen::ConstraintType::Distance;
    } else if (strcmp(type_str, "horizontal") == 0) {
        type = gcsgen::ConstraintType::Horizontal;
    } else if (strcmp(type_str, "vertical") == 0) {
        type = gcsgen::ConstraintType::Vertical;
    }

    auto proxy = (tt::Proxy<gcsgen::Constraint>*)ves_set_newforeign(0, 0, sizeof(tt::Proxy<gcsgen::Constraint>));
    proxy->obj = std::make_shared<gcsgen::Constraint>(type, geo1, geo2, value);
}

int w_Constraint_finalize(void* data)
{
    auto proxy = (tt::Proxy<gcsgen::Constraint>*)(data);
    proxy->~Proxy();
    return sizeof(tt::Proxy<gcsgen::Constraint>);
}

void w_Constraint_set_value()
{
    auto cons = ((tt::Proxy<gcsgen::Constraint>*)ves_toforeign(0))->obj;
    cons->value = ves_tonumber(1);
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
    auto cons = ((tt::Proxy<gcsgen::Constraint>*)ves_toforeign(1))->obj;

    scene->AddConstraint(*cons);
}

void w_Scene_clear()
{
    auto scene = ((tt::Proxy<gcsgen::Scene>*)ves_toforeign(0))->obj;
    scene->Clear();
}

void w_Scene_solve()
{
    auto scene = ((tt::Proxy<gcsgen::Scene>*)ves_toforeign(0))->obj;
    scene->Solve();
}

}

namespace gcsgen
{

VesselForeignMethodFn GcsGenBindMethod(const char* signature)
{
    if (strcmp(signature, "GCS_Constraint.set_value(_)") == 0) return w_Constraint_set_value;

    if (strcmp(signature, "GCS_Scene.add(_)") == 0) return w_Scene_add;
    if (strcmp(signature, "GCS_Scene.clear()") == 0) return w_Scene_clear;

    if (strcmp(signature, "GCS_Scene.solve()") == 0) return w_Scene_solve;

    return nullptr;
}

void GcsGenBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
    if (strcmp(class_name, "GCS_Constraint") == 0)
    {
        methods->allocate = w_Constraint_allocate;
        methods->finalize = w_Constraint_finalize;
        return;
    }

    if (strcmp(class_name, "GCS_Scene") == 0)
    {
        methods->allocate = w_Scene_allocate;
        methods->finalize = w_Scene_finalize;
        return;
    }
}

}