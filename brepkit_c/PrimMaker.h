#pragma once

#include <memory>

namespace brepgraph { class TopoNaming; }
namespace brepdb { class VersionTree; }

namespace brepkit
{

class TopoShape;

class PrimMaker
{
public:
	static std::shared_ptr<TopoShape> Plane(double x, double y, double z, double nx, double ny, double nz,
		uint32_t op_id = 0, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

	static std::shared_ptr<TopoShape> Box(double dx, double dy, double dz, uint32_t op_id = 0,
		const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr, const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Cylinder(double radius, double length, uint32_t op_id = 0,
		const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr, const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Cone(double r1, double r2, double height, uint32_t op_id = 0,
		const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr, const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Sphere(double radius, uint32_t op_id = 0,
		const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr, const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Sphere(double radius, double angle, uint32_t op_id = 0,
		const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr, const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Torus(double r1, double r2, uint32_t op_id = 0,
		const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr, const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Torus(double r1, double r2, double angle, uint32_t op_id = 0,
		const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr, const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

	// FreeCAD-style ellipsoid (Part::Ellipsoid). Builds a unit-axis
	// sphere of radius=r2 and scales the Z semi-axis by r1/r2 (or by
	// r3/r2 when r3 >= Precision::Confusion). r1/r2 must be positive;
	// r3 == 0 selects the r1-driven Z scale.
	static std::shared_ptr<TopoShape> Ellipsoid(double r1, double r2, double r3, uint32_t op_id = 0,
		const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr, const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

	// todo
	static std::shared_ptr<TopoShape> Threading(double thickness, double height, uint32_t op_id = 0,
		const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr, const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

}; // PrimMaker

}
