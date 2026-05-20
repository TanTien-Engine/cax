#pragma once

// OCCT
#include <TopAbs_ShapeEnum.hxx>
#include <Standard_Handle.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include <map>
#include <set>
#include <vector>

class BRepBuilderAPI_MakeShape;
class BRepTools_History;
class TopoDS_Shape;
class BRepOffset_MakeSimpleOffset;

namespace brepkit
{

class TopoShape;

class ShapeHistory
{
public:
	ShapeHistory(const TopoShape& new_shape);
	ShapeHistory(BRepBuilderAPI_MakeShape& builder, TopAbs_ShapeEnum type,
		const TopoShape& new_shape, const TopoShape& old_shape);
	ShapeHistory(opencascade::handle<BRepTools_History> hist, TopAbs_ShapeEnum type,
		const TopoShape& new_shape, const TopoShape& old_shape);
	ShapeHistory(const BRepOffset_MakeSimpleOffset& builder, const TopoShape& old_shape);

	auto& GetIdxMap() const { return m_shape_map; }

	auto& GetOldMap() const { return m_old_map; }
	auto& GetNewMap() const { return m_new_map; }

	// New-map indices that were Generated (truly new geometry created from
	// a different-type parent, e.g. fillet surface from an edge).
	const std::set<int>& GetGeneratedIndices() const { return m_generated_indices; }


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
	std::set<int> m_generated_indices;  // new_map indices from Generated()

	TopTools_IndexedMapOfShape m_new_map, m_old_map;

}; // ShapeHistory

}

