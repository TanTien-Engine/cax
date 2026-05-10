#include "PrimMaker.h"
#include "TopoShape.h"
#include "occt_adapter.h"

#include "BRepBuilder.h"
#include "BRepHistory.h"

#include <breptopo_c/TopoNaming.h>
#include <brepdb_c/WorldSender.h>
#include <brepdb_c/GeomPool.h>
#include <brepdb_c/VersionTree.h>

#include <logger/logger.h>

// OCCT
#include <gp_Pln.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
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

std::shared_ptr<TopoShape> PrimMaker::Plane(double x, double y, double z, double nx, double ny, double nz,
                                            uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                            const std::shared_ptr<brepdb::VersionTree>& vt)
{
    std::shared_ptr<TopoShape> shape = nullptr;
    try {
        gp_Pln pln(gp_Pnt(x, y, z), gp_Dir(nx, ny, nz));
        BRepBuilderAPI_MakeFace mk_face(pln, -20.0, 20.0, -20.0, 20.0);
        breptopo::TopoNaming::PidMap pid_map;
        if (tn)
        {
            auto old_shp = BRepBuilder::MakeCompound({});
            pid_map = tn->Update(mk_face, mk_face.Shape(), old_shp->GetShape(), op_id);
        }
        shape = std::make_shared<partgraph::TopoShape>(mk_face.Face());
        if (tn && vt)
        {
            brepdb::WorldSender sender(tn);
            brepdb::BRepWorld world;
            sender.Serialize(shape->GetShape(), world);
            brepdb::GeometryPool new_pool = world.ExportToPool();
            shape->SetVersionId(vt->AddRoot(new_pool, "plane"));
        }
    }
    catch (Standard_Failure& e) {
        LOGI("Build plane fail: %s", e.GetMessageString());
    }

    return shape;
}

std::shared_ptr<TopoShape> PrimMaker::Box(double dx, double dy, double dz, uint32_t op_id,
                                          const std::shared_ptr<breptopo::TopoNaming>& tn,
                                          const std::shared_ptr<brepdb::VersionTree>& vt)
{
    std::shared_ptr<TopoShape> shape = nullptr;
    try {
        BRepPrimAPI_MakeBox mk_box(dx, dy, dz);
        breptopo::TopoNaming::PidMap pid_map;
        if (tn)
        {
            auto old_shp = BRepBuilder::MakeCompound({});
            pid_map = tn->Update(mk_box, mk_box.Shape(), old_shp->GetShape(), op_id);
        }
        shape = std::make_shared<partgraph::TopoShape>(mk_box.Shape());
        if (tn && vt)
        {
            brepdb::WorldSender sender(tn);
            brepdb::BRepWorld world;
            sender.Serialize(shape->GetShape(), world);
            brepdb::GeometryPool new_pool = world.ExportToPool();
            shape->SetVersionId(vt->AddRoot(new_pool, "box"));
        }
    } catch (Standard_Failure& e) {
        LOGI("Build box fail: %s", e.GetMessageString());
    }

    return shape;
}

std::shared_ptr<TopoShape> PrimMaker::Cylinder(double radius, double length, uint32_t op_id,
                                               const std::shared_ptr<breptopo::TopoNaming>& tn,
                                               const std::shared_ptr<brepdb::VersionTree>& vt)
{
    std::shared_ptr<TopoShape> shape = nullptr;
    try {
        BRepPrimAPI_MakeCylinder mk_cyl(radius, length);
        breptopo::TopoNaming::PidMap pid_map;
        if (tn)
        {
            auto old_shp = BRepBuilder::MakeCompound({});
            pid_map = tn->Update(mk_cyl, mk_cyl.Shape(), old_shp->GetShape(), op_id);
        }
        shape = std::make_shared<partgraph::TopoShape>(mk_cyl.Shape());
        if (tn && vt)
        {
            brepdb::WorldSender sender(tn);
            brepdb::BRepWorld world;
            sender.Serialize(shape->GetShape(), world);
            brepdb::GeometryPool new_pool = world.ExportToPool();
            shape->SetVersionId(vt->AddRoot(new_pool, "cylinder"));
        }
    } catch (Standard_Failure& e) {
        LOGI("Build cylinder fail: %s", e.GetMessageString());
    }

    return shape;
}

std::shared_ptr<TopoShape> PrimMaker::Cone(double r1, double r2, double height, uint32_t op_id,
                                           const std::shared_ptr<breptopo::TopoNaming>& tn,
                                           const std::shared_ptr<brepdb::VersionTree>& vt)
{
    std::shared_ptr<TopoShape> shape = nullptr;
    try {
        BRepPrimAPI_MakeCone mk_cone(r1, r2, height);
        breptopo::TopoNaming::PidMap pid_map;
        if (tn)
        {
            auto old_shp = BRepBuilder::MakeCompound({});
            pid_map = tn->Update(mk_cone, mk_cone.Shape(), old_shp->GetShape(), op_id);
        }
        shape = std::make_shared<partgraph::TopoShape>(mk_cone.Shape());
        if (tn && vt)
        {
            brepdb::WorldSender sender(tn);
            brepdb::BRepWorld world;
            sender.Serialize(shape->GetShape(), world);
            brepdb::GeometryPool new_pool = world.ExportToPool();
            shape->SetVersionId(vt->AddRoot(new_pool, "cone"));
        }
    } catch (Standard_Failure& e) {
        LOGI("Build cone fail: %s", e.GetMessageString());
    }

    return shape;
}

std::shared_ptr<TopoShape> PrimMaker::Sphere(double radius, uint32_t op_id,
                                             const std::shared_ptr<breptopo::TopoNaming>& tn,
                                             const std::shared_ptr<brepdb::VersionTree>& vt)
{
    std::shared_ptr<TopoShape> shape = nullptr;
    try {
        BRepPrimAPI_MakeSphere mk_sphere(radius);
        breptopo::TopoNaming::PidMap pid_map;
        if (tn)
        {
            auto old_shp = BRepBuilder::MakeCompound({});
            pid_map = tn->Update(mk_sphere, mk_sphere.Shape(), old_shp->GetShape(), op_id);
        }
        shape = std::make_shared<partgraph::TopoShape>(mk_sphere.Shape());
        if (tn && vt)
        {
            brepdb::WorldSender sender(tn);
            brepdb::BRepWorld world;
            sender.Serialize(shape->GetShape(), world);
            brepdb::GeometryPool new_pool = world.ExportToPool();
            shape->SetVersionId(vt->AddRoot(new_pool, "sphere"));
        }
    } catch (Standard_Failure& e) {
        LOGI("Build sphere fail: %s", e.GetMessageString());
    }

    return shape;
}

std::shared_ptr<TopoShape> PrimMaker::Sphere(double radius, double angle, uint32_t op_id,
                                             const std::shared_ptr<breptopo::TopoNaming>& tn,
                                             const std::shared_ptr<brepdb::VersionTree>& vt)
{
    std::shared_ptr<TopoShape> shape = nullptr;
    try {
        BRepPrimAPI_MakeSphere mk_sphere(radius, angle);
        breptopo::TopoNaming::PidMap pid_map;
        if (tn)
        {
            auto old_shp = BRepBuilder::MakeCompound({});
            pid_map = tn->Update(mk_sphere, mk_sphere.Shape(), old_shp->GetShape(), op_id);
        }
        shape = std::make_shared<partgraph::TopoShape>(mk_sphere.Shape());
        if (tn && vt)
        {
            brepdb::WorldSender sender(tn);
            brepdb::BRepWorld world;
            sender.Serialize(shape->GetShape(), world);
            brepdb::GeometryPool new_pool = world.ExportToPool();
            shape->SetVersionId(vt->AddRoot(new_pool, "sphere"));
        }
    } catch (Standard_Failure& e) {
        LOGI("Build sphere fail: %s", e.GetMessageString());
    }

    return shape;
}

std::shared_ptr<TopoShape> PrimMaker::Torus(double r1, double r2, uint32_t op_id,
                                            const std::shared_ptr<breptopo::TopoNaming>& tn,
                                            const std::shared_ptr<brepdb::VersionTree>& vt)
{
    std::shared_ptr<TopoShape> shape = nullptr;
    try {
        BRepPrimAPI_MakeTorus mk_torus(r1, r2);
        breptopo::TopoNaming::PidMap pid_map;
        if (tn)
        {
            auto old_shp = BRepBuilder::MakeCompound({});
            pid_map = tn->Update(mk_torus, mk_torus.Shape(), old_shp->GetShape(), op_id);
        }
        shape = std::make_shared<partgraph::TopoShape>(mk_torus.Shape());
        if (tn && vt)
        {
            brepdb::WorldSender sender(tn);
            brepdb::BRepWorld world;
            sender.Serialize(shape->GetShape(), world);
            brepdb::GeometryPool new_pool = world.ExportToPool();
            shape->SetVersionId(vt->AddRoot(new_pool, "torus"));
        }
    } catch (Standard_Failure& e) {
        LOGI("Build torus fail: %s", e.GetMessageString());
    }

    return shape;
}

std::shared_ptr<TopoShape> PrimMaker::Torus(double r1, double r2, double angle, uint32_t op_id,
                                            const std::shared_ptr<breptopo::TopoNaming>& tn,
                                            const std::shared_ptr<brepdb::VersionTree>& vt)
{
    std::shared_ptr<TopoShape> shape = nullptr;
    try {
        BRepPrimAPI_MakeTorus mk_torus(r1, r2, angle);
        breptopo::TopoNaming::PidMap pid_map;
        if (tn)
        {
            auto old_shp = BRepBuilder::MakeCompound({});
            pid_map = tn->Update(mk_torus, mk_torus.Shape(), old_shp->GetShape(), op_id);
        }
        shape = std::make_shared<partgraph::TopoShape>(mk_torus.Shape());
        if (tn && vt)
        {
            brepdb::WorldSender sender(tn);
            brepdb::BRepWorld world;
            sender.Serialize(shape->GetShape(), world);
            brepdb::GeometryPool new_pool = world.ExportToPool();
            shape->SetVersionId(vt->AddRoot(new_pool, "torus"));
        }
    } catch (Standard_Failure& e) {
        LOGI("Build torus fail: %s", e.GetMessageString());
    }

    return shape;
}

std::shared_ptr<TopoShape> PrimMaker::Threading(double thickness, double height, uint32_t op_id,
                                                const std::shared_ptr<breptopo::TopoNaming>& tn,
                                                const std::shared_ptr<brepdb::VersionTree>& vt)
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

    breptopo::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = BRepBuilder::MakeCompound({});
        pid_map = tn->Update(aTool, aTool.Shape(), old_shp->GetShape(), op_id);
    }
    auto shape = std::make_shared<partgraph::TopoShape>(aTool.Shape());
    if (tn && vt)
    {
        brepdb::WorldSender sender(tn);
        brepdb::BRepWorld world;
        sender.Serialize(shape->GetShape(), world);
        brepdb::GeometryPool new_pool = world.ExportToPool();
        shape->SetVersionId(vt->AddRoot(new_pool, "threading"));
    }
    return shape;
}

}