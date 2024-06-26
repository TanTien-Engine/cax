#include "BRepSelector.h"
#include "TopoShape.h"
#include "occt_adapter.h"

// OCCT
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopExp.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Line.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <BRepIntCurveSurface_Inter.hxx>
#include <GeomAPI_ExtremaCurveCurve.hxx>

namespace partgraph
{

std::shared_ptr<TopoShape> BRepSelector::SelectFace(const std::shared_ptr<TopoShape>& shape, int index)
{
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape->GetShape(), TopAbs_FACE, faces);
    if (index >= 1 && index < faces.Extent())
    {
        TopoDS_Face selected = TopoDS::Face(faces.FindKey(index));
        return std::make_shared<TopoShape>(selected);
    }
    else
    {
        return nullptr;
    }
}
  
std::shared_ptr<TopoShape> BRepSelector::SelectFace(const std::shared_ptr<TopoShape>& shape, FacePos pos)
{
    TopoDS_Face selected;

    Standard_Real v_min = DBL_MAX, v_max = -DBL_MAX;
    for (TopExp_Explorer face_explorer(shape->GetShape(), TopAbs_FACE); face_explorer.More(); face_explorer.Next())
    {
        TopoDS_Face face = TopoDS::Face(face_explorer.Current());
        Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
        if (surface->DynamicType() == STANDARD_TYPE(Geom_Plane))
        {
            Handle(Geom_Plane) plane = Handle(Geom_Plane)::DownCast(surface);
            gp_Pnt pnt = plane->Location();
            switch (pos)
            {
            case FacePos::X_MIN:
                if (pnt.X() < v_min) 
                {
                    v_min = pnt.X();
                    selected = face;
                }
                break;
            case FacePos::X_MAX:
                if (pnt.X() > v_max)
                {
                    v_max = pnt.X();
                    selected = face;
                }
                break;
            case FacePos::Y_MIN:
                if (pnt.Y() < v_min)
                {
                    v_min = pnt.Y();
                    selected = face;
                }
                break;
            case FacePos::Y_MAX:
                if (pnt.Y() > v_max)
                {
                    v_max = pnt.Y();
                    selected = face;
                }
                break;
            case FacePos::Z_MIN:
                if (pnt.Z() < v_min)
                {
                    v_min = pnt.Z();
                    selected = face;
                }
                break;
            case FacePos::Z_MAX:
                if (pnt.Z() > v_max)
                {
                    v_max = pnt.Z();
                    selected = face;
                }
                break;
            }
        }
    }

    if (v_min == DBL_MAX && v_max == -DBL_MAX) {
        return nullptr;
    } else {
        return std::make_shared<TopoShape>(selected);
    }
}

std::shared_ptr<TopoShape> BRepSelector::SelectFace(const std::shared_ptr<TopoShape>& shape, const sm::Ray& ray)
{
    gp_Pnt pos = trans_pnt(ray.origin);
    gp_Dir dir = trans_dir(ray.dir);
    gp_Lin line(pos, dir);

    TopoDS_Face selected;
    double min_dist = DBL_MAX;

    for (TopExp_Explorer face_explorer(shape->GetShape(), TopAbs_FACE); face_explorer.More(); face_explorer.Next())
    {
        TopoDS_Face face = TopoDS::Face(face_explorer.Current());

        BRepIntCurveSurface_Inter inter;
        inter.Init(face, line, 1e-6);//Precision::PConfusion());
        for (; inter.More(); inter.Next())
        {
            const gp_Pnt& p = inter.Point().Pnt();
            double dist = p.Distance(pos);
            if (dist < min_dist)
            {
                selected = face;
                min_dist = dist;
            }
        }
    }

    return min_dist == DBL_MAX ? nullptr : std::make_shared<TopoShape>(selected);
}

std::shared_ptr<TopoShape> BRepSelector::SelectEdge(const std::shared_ptr<TopoShape>& shape, const sm::Ray& ray)
{
    gp_Pnt pos = trans_pnt(ray.origin);
    gp_Dir dir = trans_dir(ray.dir);
    gp_Lin line(pos, dir);
    Handle(Geom_Line) geom_line = new Geom_Line(line);

    TopoDS_Edge selected;
    double min_dist = DBL_MAX;

    for (TopExp_Explorer edge_explorer(shape->GetShape(), TopAbs_EDGE); edge_explorer.More(); edge_explorer.Next())
    {
        TopoDS_Edge edge = TopoDS::Edge(edge_explorer.Current());
        Standard_Real f, l;
        auto curve = BRep_Tool::Curve(edge, f, l);

        GeomAPI_ExtremaCurveCurve inter(curve, geom_line, f, l, 0, 10);
        gp_Pnt p1, p2;
        if (inter.TotalNearestPoints(p1, p2))
        {
            const double uncertainty = p1.Distance(p2);
            if (uncertainty < 0.01)
            {
                gp_Pnt p = 0.5 * (p1.XYZ() + p2.XYZ());
                double dist = p.Distance(pos);
                if (dist < min_dist)
                {
                    selected = edge;
                    min_dist = dist;
                }
            }
        }
    }

    return min_dist == DBL_MAX ? nullptr : std::make_shared<TopoShape>(selected);
}

}