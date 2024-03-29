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
	static std::shared_ptr<TopoShape> Cone(double r1, double r2, double height);
	static std::shared_ptr<TopoShape> Sphere(double radius);
	static std::shared_ptr<TopoShape> Sphere(double radius, double angle);
	static std::shared_ptr<TopoShape> Torus(double r1, double r2);
	static std::shared_ptr<TopoShape> Torus(double r1, double r2, double angle);

	// todo
	static std::shared_ptr<TopoShape> Threading(double thickness, double height);

}; // PrimMaker

}