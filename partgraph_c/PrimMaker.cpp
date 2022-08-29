#include "PrimMaker.h"
#include "TopoShape.h"

// OCCT
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>

#include <logger/logger.h>

namespace partgraph
{

std::shared_ptr<TopoShape> PrimMaker::Box(double dx, double dy, double dz)
{
    std::shared_ptr<TopoShape> shape = nullptr;
    try {
        BRepPrimAPI_MakeBox mk_box(dx, dy, dz);
        shape = std::make_shared<partgraph::TopoShape>(mk_box.Shape());
    } catch (Standard_Failure& e) {
        LOGI("Build box fail: %s", e.GetMessageString());
    }

    return shape;
}

std::shared_ptr<TopoShape> PrimMaker::Cylinder(double radius, double length)
{
    std::shared_ptr<TopoShape> shape = nullptr;
    try {
        TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(radius, length);
        shape = std::make_shared<partgraph::TopoShape>(cylinder);
    } catch (Standard_Failure& e) {
        LOGI("Build cylinder fail: %s", e.GetMessageString());
    }

    return shape;
}

}