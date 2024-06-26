#include "BRepTools.h"
#include "TopoShape.h"

// OCCT
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>

namespace partgraph
{

int BRepTools::FindEdgeIdx(const std::shared_ptr<TopoShape>& shape, const std::shared_ptr<TopoShape>& key)
{
	TopTools_IndexedMapOfShape edges;
	TopExp::MapShapes(shape->GetShape(), TopAbs_EDGE, edges);
	return edges.FindIndex(key->GetShape());
}

std::shared_ptr<TopoShape> BRepTools::FindEdgeKey(const std::shared_ptr<TopoShape>& shape, int idx)
{
	TopTools_IndexedMapOfShape edges;
	TopExp::MapShapes(shape->GetShape(), TopAbs_EDGE, edges);
	if (idx >= 1 && idx <= edges.Extent())
	{
		TopoDS_Edge edge = TopoDS::Edge(edges.FindKey(idx));
		return std::make_shared<TopoShape>(edge);
	}
	else
	{
		return nullptr;
	}
}

int BRepTools::FindFaceIdx(const std::shared_ptr<TopoShape>& shape, const std::shared_ptr<TopoShape>& key)
{
	TopTools_IndexedMapOfShape faces;
	TopExp::MapShapes(shape->GetShape(), TopAbs_FACE, faces);
	return faces.FindIndex(key->GetShape());
}

std::shared_ptr<TopoShape> BRepTools::FindFaceKey(const std::shared_ptr<TopoShape>& shape, int idx)
{
	TopTools_IndexedMapOfShape faces;
	TopExp::MapShapes(shape->GetShape(), TopAbs_FACE, faces);
	if (idx >= 1 && idx <= faces.Extent())
	{
		TopoDS_Face face = TopoDS::Face(faces.FindKey(idx));
		return std::make_shared<TopoShape>(face);
	}
	else
	{
		return nullptr;
	}
}

}