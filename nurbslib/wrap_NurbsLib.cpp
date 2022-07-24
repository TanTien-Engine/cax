#include "wrap_NurbsLib.h"
#include "modules/script/TransHelper.h"

#include <nurbs/nurbs.h>
#include <geoshape/Polyline2D.h>
#include <SM_Calc.h>

namespace
{

void w_NurbsLib_bezier()
{
    auto polyline = ((tt::Proxy<gs::Polyline2D>*)ves_toforeign(1))->obj;

    auto& verts = polyline->GetVertices();

    auto poly_num = (int)ves_tonumber(2);
    std::vector<sm::vec2> out_poly(poly_num);

    nurbs::bezier(verts, out_poly);
    
    auto poly = std::make_shared<gs::Polyline2D>(out_poly, false);

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("geometry", "Polyline");
    auto proxy = (tt::Proxy<gs::Polyline2D>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<gs::Polyline2D>));
    proxy->obj = poly;
    ves_pop(1);
}

}

namespace nurbslib
{

VesselForeignMethodFn NurbsLibBindMethod(const char* signature)
{
    if (strcmp(signature, "static NurbsLib.bezier(_,_)") == 0) return w_NurbsLib_bezier;

    return nullptr;
}

void NurbsLibBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
}

}