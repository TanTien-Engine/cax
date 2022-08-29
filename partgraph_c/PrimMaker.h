#pragma once

#include <memory>

namespace partgraph
{

class TopoShape;

class PrimMaker
{
public:
	static std::shared_ptr<TopoShape> Box(double dx, double dy, double dz);
	static std::shared_ptr<TopoShape> Cylinder(double radius, double length);

}; // PrimMaker

}