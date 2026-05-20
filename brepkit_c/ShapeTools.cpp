#include "ShapeTools.h"
#include "TopoShape.h"

// OCCT
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>

namespace brepkit
{

int ShapeTools::FindEdgeIdx(const std::shared_ptr<TopoShape>& shape, const std::shared_ptr<TopoShape>& key)
{
	TopTools_IndexedMapOfShape edges;
	TopExp::MapShapes(shape->GetShape(), TopAbs_EDGE, edges);
	return edges.FindIndex(key->GetShape());
}

std::shared_ptr<TopoShape> ShapeTools::FindEdgeKey(const std::shared_ptr<TopoShape>& shape, int idx)
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

int ShapeTools::FindFaceIdx(const std::shared_ptr<TopoShape>& shape, const std::shared_ptr<TopoShape>& key)
{
	TopTools_IndexedMapOfShape faces;
	TopExp::MapShapes(shape->GetShape(), TopAbs_FACE, faces);
	return faces.FindIndex(key->GetShape());
}

std::shared_ptr<TopoShape> ShapeTools::FindFaceKey(const std::shared_ptr<TopoShape>& shape, int idx)
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

std::vector<std::shared_ptr<TopoShape>>
ShapeTools::MapShells(const std::shared_ptr<TopoShape>& shape)
{
	TopTools_IndexedMapOfShape shells;
	TopExp::MapShapes(shape->GetShape(), TopAbs_SHELL, shells);
	std::vector<std::shared_ptr<brepkit::TopoShape>> ret;
	for (int i = 1, n = shells.Extent(); i <= n; ++i)
	{
		TopoDS_Shell shell = TopoDS::Shell(shells.FindKey(i));
		ret.push_back(std::make_shared<brepkit::TopoShape>(shell));
	}
	return ret;
}

std::vector<std::shared_ptr<TopoShape>> 
ShapeTools::MapFaces(const std::shared_ptr<TopoShape>& shape)
{
	TopTools_IndexedMapOfShape faces;
	TopExp::MapShapes(shape->GetShape(), TopAbs_FACE, faces);
	std::vector<std::shared_ptr<brepkit::TopoShape>> ret;
	for (int i = 1, n = faces.Extent(); i <= n; ++i)
	{
		TopoDS_Face face = TopoDS::Face(faces.FindKey(i));
		ret.push_back(std::make_shared<brepkit::TopoShape>(face));
	}
	return ret;
}

std::vector<std::shared_ptr<TopoShape>>
ShapeTools::MapEdges(const std::shared_ptr<TopoShape>& shape)
{
	TopTools_IndexedMapOfShape edges;
	TopExp::MapShapes(shape->GetShape(), TopAbs_EDGE, edges);
	std::vector<std::shared_ptr<brepkit::TopoShape>> ret;
	for (int i = 1, n = edges.Extent(); i <= n; ++i)
	{
		TopoDS_Edge edge = TopoDS::Edge(edges.FindKey(i));
		ret.push_back(std::make_shared<brepkit::TopoShape>(edge));
	}
	return ret;
}

}