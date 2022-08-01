#include "wrap_NurbsLib.h"
#include "modules/script/TransHelper.h"

#include <nurbs/nurbs.h>
#include <geoshape/Polyline2D.h>
#include <geoshape/Polyline3D.h>
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

void w_NurbsLib_bspline()
{
    auto polyline = ((tt::Proxy<gs::Polyline2D>*)ves_toforeign(1))->obj;
    auto& verts = polyline->GetVertices();

    auto order = (int)ves_tonumber(2);

    auto poly_num = (int)ves_tonumber(3);
    std::vector<sm::vec2> out_poly(poly_num);

    nurbs::bspline(verts, order, out_poly);
    
    auto poly = std::make_shared<gs::Polyline2D>(out_poly, false);

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("geometry", "Polyline");
    auto proxy = (tt::Proxy<gs::Polyline2D>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<gs::Polyline2D>));
    proxy->obj = poly;
    ves_pop(1);
}

void w_NurbsLib_rbspline()
{
    auto polyline = ((tt::Proxy<gs::Polyline2D>*)ves_toforeign(1))->obj;
    auto& verts = polyline->GetVertices();

    auto order = (int)ves_tonumber(2);

    auto poly_num = (int)ves_tonumber(3);
    std::vector<sm::vec2> out_poly(poly_num);

    nurbs::rbspline(verts, order, out_poly);
    
    auto poly = std::make_shared<gs::Polyline2D>(out_poly, false);

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("geometry", "Polyline");
    auto proxy = (tt::Proxy<gs::Polyline2D>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<gs::Polyline2D>));
    proxy->obj = poly;
    ves_pop(1);
}

void w_NurbsLib_bezsurf()
{
    auto polyline = ((tt::Proxy<gs::Polyline3D>*)ves_toforeign(1))->obj;
//    auto& verts = polyline->GetVertices();

    auto npts = (int)ves_tonumber(2);
    auto mpts = (int)ves_tonumber(3);
    auto p1 = (int)ves_tonumber(4);
    auto p2 = (int)ves_tonumber(5);

    auto poly_num = (int)ves_tonumber(3);
    std::vector<sm::vec3> surface(p1 * p2);

    nurbs::bezsurf(nullptr, npts, mpts, p1, p2, surface);

    auto poly = std::make_shared<gs::Polyline3D>(surface, false);

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("geometry", "Polyline3D");
    auto proxy = (tt::Proxy<gs::Polyline3D>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<gs::Polyline3D>));
    proxy->obj = poly;
    ves_pop(1);
}

}

namespace nurbslib
{

VesselForeignMethodFn NurbsLibBindMethod(const char* signature)
{
    if (strcmp(signature, "static NurbsLib.bezier(_,_)") == 0) return w_NurbsLib_bezier;
    if (strcmp(signature, "static NurbsLib.bspline(_,_,_)") == 0) return w_NurbsLib_bspline;
    if (strcmp(signature, "static NurbsLib.rbspline(_,_,_)") == 0) return w_NurbsLib_rbspline;
    if (strcmp(signature, "static NurbsLib.bezsurf(_,_,_,_,_)") == 0) return w_NurbsLib_bezsurf;

    return nullptr;
}

void NurbsLibBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
}

}