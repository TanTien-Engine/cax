#pragma once

#include <memory>

namespace partgraph
{

class TopoShape;
class TopoFace;

class BRepSelector
{
public:
	static std::shared_ptr<TopoFace> SelectFace(const std::shared_ptr<TopoShape>& shape);

}; // BRepSelector

}