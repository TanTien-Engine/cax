#include "BRepSelector.h"
#include "TopoDataset.h"

// OCCT
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Plane.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopExp.hxx>

namespace partgraph
{

std::shared_ptr<TopoFace> BRepSelector::SelectFace(const std::shared_ptr<TopoShape>& shape)
{
    TopoDS_Face selected;
    Standard_Real z_max = -1;

    for (TopExp_Explorer face_explorer(shape->GetShape(), TopAbs_FACE); face_explorer.More(); face_explorer.Next())
    {
        TopoDS_Face face = TopoDS::Face(face_explorer.Current());
        Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
        if (surface->DynamicType() == STANDARD_TYPE(Geom_Plane))
        {
            Handle(Geom_Plane) plane = Handle(Geom_Plane)::DownCast(surface);
            gp_Pnt pnt = plane->Location();
            Standard_Real z = pnt.Z();
            if (z > z_max) {
                z_max = z;
                selected = face;
            }
        }
    }

    //TopTools_IndexedMapOfShape faces;
    //TopExp::MapShapes(shape->GetShape(), TopAbs_FACE, faces);
    //selected = TopoDS::Face(faces.FindKey(1));

    return std::make_shared<TopoFace>(selected);
}

}