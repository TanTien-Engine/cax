#include "BRepTools.h"
#include "TopoDataset.h"

// OCCT
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>

namespace partgraph
{

int BRepTools::FindEdgeIdx(const std::shared_ptr<TopoShape>& shape, const std::shared_ptr<TopoEdge>& key)
{
	TopTools_IndexedMapOfShape edges;
	TopExp::MapShapes(shape->GetShape(), TopAbs_EDGE, edges);
	return edges.FindIndex(key->GetEdge());
}

std::shared_ptr<TopoEdge> BRepTools::FindEdgeKey(const std::shared_ptr<TopoShape>& shape, int idx)
{
	TopTools_IndexedMapOfShape edges;
	TopExp::MapShapes(shape->GetShape(), TopAbs_EDGE, edges);
	if (idx >= 1 && idx <= edges.Extent())
	{
		TopoDS_Edge edge = TopoDS::Edge(edges.FindKey(idx));
		return std::make_shared<TopoEdge>(edge);
	}
	else
	{
		return nullptr;
	}
}

int BRepTools::FindFaceIdx(const std::shared_ptr<TopoShape>& shape, const std::shared_ptr<TopoFace>& key)
{
	TopTools_IndexedMapOfShape faces;
	TopExp::MapShapes(shape->GetShape(), TopAbs_FACE, faces);
	return faces.FindIndex(key->GetFace());
}

std::shared_ptr<TopoFace> BRepTools::FindFaceKey(const std::shared_ptr<TopoShape>& shape, int idx)
{
	TopTools_IndexedMapOfShape faces;
	TopExp::MapShapes(shape->GetShape(), TopAbs_FACE, faces);
	if (idx >= 1 && idx <= faces.Extent())
	{
		TopoDS_Face face = TopoDS::Face(faces.FindKey(idx));
		return std::make_shared<TopoFace>(face);
	}
	else
	{
		return nullptr;
	}
}

}