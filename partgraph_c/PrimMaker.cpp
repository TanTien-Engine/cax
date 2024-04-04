#include "PrimMaker.h"
#include "TopoDataset.h"

#include "BRepBuilder.h"
#include "BRepHistory.h"

#include "../breptopo_c/BrepTopo.h"
#include "../breptopo_c/HistGraph.h"

#include <logger/logger.h>

// OCCT
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <TopExp.hxx>
// fixme
#include <Geom_CylindricalSurface.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <GCE2d_MakeSegment.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepLib.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>

namespace partgraph
{

std::shared_ptr<TopoShape> PrimMaker::Box(double dx, double dy, double dz, uint32_t op_id)
{
    std::shared_ptr<TopoShape> shape = nullptr;
    try {
        BRepPrimAPI_MakeBox mk_box(dx, dy, dz);

        auto old_shp = BRepBuilder::MakeCompound({});
        BRepHistory hist(mk_box, TopAbs_FACE, mk_box.Shape(), old_shp->GetShape());

        auto hist_group = breptopo::Context::Instance()->GetHist();
        hist_group->Update(hist, op_id);

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

std::shared_ptr<TopoShape> PrimMaker::Cone(double r1, double r2, double height)
{
    std::shared_ptr<TopoShape> shape = nullptr;
    try {
        TopoDS_Shape cone = BRepPrimAPI_MakeCone(r1, r2, height);
        shape = std::make_shared<partgraph::TopoShape>(cone);
    } catch (Standard_Failure& e) {
        LOGI("Build cone fail: %s", e.GetMessageString());
    }

    return shape;
}

std::shared_ptr<TopoShape> PrimMaker::Sphere(double radius)
{
    std::shared_ptr<TopoShape> shape = nullptr;
    try {
        TopoDS_Shape sphere = BRepPrimAPI_MakeSphere(radius);
        shape = std::make_shared<partgraph::TopoShape>(sphere);
    } catch (Standard_Failure& e) {
        LOGI("Build sphere fail: %s", e.GetMessageString());
    }

    return shape;
}

std::shared_ptr<TopoShape> PrimMaker::Sphere(double radius, double angle)
{
    std::shared_ptr<TopoShape> shape = nullptr;
    try {
        TopoDS_Shape sphere = BRepPrimAPI_MakeSphere(radius, angle);
        shape = std::make_shared<partgraph::TopoShape>(sphere);
    } catch (Standard_Failure& e) {
        LOGI("Build sphere fail: %s", e.GetMessageString());
    }

    return shape;
}

std::shared_ptr<TopoShape> PrimMaker::Torus(double r1, double r2)
{
    std::shared_ptr<TopoShape> shape = nullptr;
    try {
        TopoDS_Shape torus = BRepPrimAPI_MakeTorus(r1, r2);
        shape = std::make_shared<partgraph::TopoShape>(torus);
    } catch (Standard_Failure& e) {
        LOGI("Build torus fail: %s", e.GetMessageString());
    }

    return shape;
}

std::shared_ptr<TopoShape> PrimMaker::Torus(double r1, double r2, double angle)
{
    std::shared_ptr<TopoShape> shape = nullptr;
    try {
        TopoDS_Shape torus = BRepPrimAPI_MakeTorus(r1, r2, angle);
        shape = std::make_shared<partgraph::TopoShape>(torus);
    } catch (Standard_Failure& e) {
        LOGI("Build torus fail: %s", e.GetMessageString());
    }

    return shape;
}

std::shared_ptr<TopoShape> PrimMaker::Threading(double thickness, double height)
{
    gp_Pnt neckLocation(0, 0, height);
    gp_Dir neckAxis = gp::DZ();
    gp_Ax2 neckAx2(neckLocation, neckAxis);

    Standard_Real myNeckRadius = thickness / 4.;
    Standard_Real myNeckHeight = height / 10.;

    // Threading : Create Surfaces
    Handle(Geom_CylindricalSurface) aCyl1 = new Geom_CylindricalSurface(neckAx2, myNeckRadius * 0.99);
    Handle(Geom_CylindricalSurface) aCyl2 = new Geom_CylindricalSurface(neckAx2, myNeckRadius * 1.05);

    // Threading : Define 2D Curves
    gp_Pnt2d aPnt(2. * M_PI, myNeckHeight / 2.);
    gp_Dir2d aDir(2. * M_PI, myNeckHeight / 4.);
    gp_Ax2d anAx2d(aPnt, aDir);

    Standard_Real aMajor = 2. * M_PI;
    Standard_Real aMinor = myNeckHeight / 10;

    Handle(Geom2d_Ellipse) anEllipse1 = new Geom2d_Ellipse(anAx2d, aMajor, aMinor);
    Handle(Geom2d_Ellipse) anEllipse2 = new Geom2d_Ellipse(anAx2d, aMajor, aMinor / 4);
    Handle(Geom2d_TrimmedCurve) anArc1 = new Geom2d_TrimmedCurve(anEllipse1, 0, M_PI);
    Handle(Geom2d_TrimmedCurve) anArc2 = new Geom2d_TrimmedCurve(anEllipse2, 0, M_PI);
    gp_Pnt2d anEllipsePnt1 = anEllipse1->Value(0);
    gp_Pnt2d anEllipsePnt2 = anEllipse1->Value(M_PI);

    Handle(Geom2d_TrimmedCurve) aSegment = GCE2d_MakeSegment(anEllipsePnt1, anEllipsePnt2);
    // Threading : Build Edges and Wires
    TopoDS_Edge anEdge1OnSurf1 = BRepBuilderAPI_MakeEdge(anArc1, aCyl1);
    TopoDS_Edge anEdge2OnSurf1 = BRepBuilderAPI_MakeEdge(aSegment, aCyl1);
    TopoDS_Edge anEdge1OnSurf2 = BRepBuilderAPI_MakeEdge(anArc2, aCyl2);
    TopoDS_Edge anEdge2OnSurf2 = BRepBuilderAPI_MakeEdge(aSegment, aCyl2);
    TopoDS_Wire threadingWire1 = BRepBuilderAPI_MakeWire(anEdge1OnSurf1, anEdge2OnSurf1);
    TopoDS_Wire threadingWire2 = BRepBuilderAPI_MakeWire(anEdge1OnSurf2, anEdge2OnSurf2);
    BRepLib::BuildCurves3d(threadingWire1);
    BRepLib::BuildCurves3d(threadingWire2);

    // Create Threading 
    BRepOffsetAPI_ThruSections aTool(Standard_True);
    aTool.AddWire(threadingWire1);
    aTool.AddWire(threadingWire2);
    aTool.CheckCompatibility(Standard_False);

    TopoDS_Shape myThreading = aTool.Shape();
    return std::make_shared<partgraph::TopoShape>(myThreading);
}

}