#pragma once

#include <TopAbs_ShapeEnum.hxx>

#include <map>
#include <vector>

class BRepBuilderAPI_MakeShape;

namespace partgraph
{

class TopoShape;

class BRepHistory
{
public:
	BRepHistory(BRepBuilderAPI_MakeShape& builder, TopAbs_ShapeEnum type,
		const TopoShape& new_shape, const TopoShape& old_shape);

private:
	std::map<int, std::vector<int>> m_shape_map;

}; // BRepHistory

}

