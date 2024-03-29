#pragma once

// OCCT
#include <TopAbs_ShapeEnum.hxx>
#include <Standard_Handle.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include <map>
#include <vector>

class BRepBuilderAPI_MakeShape;
class BRepTools_History;
class TopoDS_Shape;

namespace partgraph
{

class TopoShape;

class BRepHistory
{
public:
	BRepHistory(BRepBuilderAPI_MakeShape& builder, TopAbs_ShapeEnum type,
		const TopoShape& new_shape, const TopoShape& old_shape);
	BRepHistory(opencascade::handle<BRepTools_History> hist, TopAbs_ShapeEnum type,
		const TopoShape& new_shape, const TopoShape& old_shape);

	auto& GetIdxMap() const { return m_shape_map; }

	auto& GetOldMap() const { return m_old_map; }
	auto& GetNewMap() const { return m_new_map; }

public:
	class History
	{
	public:
		History() {}

		virtual const TopTools_ListOfShape& Generated(const TopoDS_Shape& S) = 0;
		virtual const TopTools_ListOfShape& Modified(const TopoDS_Shape& S) = 0;
		virtual Standard_Boolean IsDeleted(const TopoDS_Shape& S) = 0;

	}; // History

	void BuildHistory(const std::shared_ptr<History>& hist, TopAbs_ShapeEnum type,
		const TopoShape& new_shape, const TopoShape& old_shape);

private:
	std::map<int, std::vector<int>> m_shape_map;

	TopTools_IndexedMapOfShape m_new_map, m_old_map;

}; // BRepHistory

}

