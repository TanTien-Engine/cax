#include "BRepHistory.h"
#include "TopoDataset.h"

// OCCT
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopExp.hxx>
#include <BRepBuilderAPI_MakeShape.hxx>

namespace partgraph
{

// From FreeCAD
BRepHistory::BRepHistory(BRepBuilderAPI_MakeShape& builder, TopAbs_ShapeEnum type, 
	                     const TopoShape& new_shape, const TopoShape& old_shape)
{
    TopTools_IndexedMapOfShape new_map, old_map;
    TopExp::MapShapes(new_shape.GetShape(), type, new_map); // map containing all old objects of type "type"
    TopExp::MapShapes(old_shape.GetShape(), type, old_map); // map containing all new objects of type "type"

    // Look at all objects in the old shape and try to find the modified object in the new shape
    for (int i = 1; i <= old_map.Extent(); i++) 
    {
        bool found = false;
        TopTools_ListIteratorOfListOfShape it;
        // Find all new objects that are a modification of the old object (e.g. a face was resized)
        for (it.Initialize(builder.Modified(old_map(i))); it.More(); it.Next()) 
        {
            found = true;
            for (int j=1; j<=new_map.Extent(); j++) { // one old object might create several new ones!
                if (new_map(j).IsPartner(it.Value())) {
                    m_shape_map[i-1].push_back(j-1); // adjust indices to start at zero
                    break;
                }
            }
        }

        // Find all new objects that were generated from an old object (e.g. a face generated from an edge)
        for (it.Initialize(builder.Generated(old_map(i))); it.More(); it.Next()) 
        {
            found = true;
            for (int j=1; j<=new_map.Extent(); j++) {
                if (new_map(j).IsPartner(it.Value())) {
                    m_shape_map[i-1].push_back(j-1);
                    break;
                }
            }
        }

        if (!found) 
        {
            // Find all old objects that don't exist any more (e.g. a face was completely cut away)
            if (builder.IsDeleted(old_map(i))) 
            {
                m_shape_map[i-1] = std::vector<int>();
            }
            else 
            {
                // Mop up the rest (will this ever be reached?)
                for (int j=1; j<=new_map.Extent(); j++) {
                    if (new_map(j).IsPartner(old_map(i))) {
                        m_shape_map[i-1].push_back(j-1);
                        break;
                    }
                }
            }
        }
    }
}

}