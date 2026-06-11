#pragma once

#include <SM_Vector.h>

#include <memory>
#include <vector>

namespace brepgraph { class TopoNaming; }
namespace brepdb { class VersionTree; }

namespace brepkit
{

class TopoShape;

class TopoAlgo
{
public:
	static std::shared_ptr<TopoShape> Fillet(const std::shared_ptr<TopoShape>& shape, double radius,
		const std::vector<std::shared_ptr<TopoShape>>& edges, uint32_t op_id,
		const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Chamfer(const std::shared_ptr<TopoShape>& shape, double dist,
		const std::vector<std::shared_ptr<TopoShape>>& edges, uint32_t op_id,
		const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

	static std::shared_ptr<TopoShape> Prism(const std::shared_ptr<TopoShape>& face, double x, double y, double z,
		uint32_t op_id = 0,
		const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

	// Drafted (tapered) prism: extrude the planar face along (x,y,z) with
	// the lateral walls tilted by `angle` radians; angle > 0 shrinks the
	// section along the extrusion direction (ZW3D "Draft angle" -- see
	// FeatPayloadExtrude::draft). Falls back to the straight Prism with a
	// stderr WARN when the face is not planar or LocOpe_DPrism refuses.
	static std::shared_ptr<TopoShape> DPrism(const std::shared_ptr<TopoShape>& face, double x, double y, double z,
		double angle,
		uint32_t op_id = 0,
		const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

	static std::shared_ptr<TopoShape> Split(const std::shared_ptr<TopoShape>& base, const std::shared_ptr<TopoShape>& tool,
		uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Cut(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
		uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Fuse(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
		uint32_t op_id = 0, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Common(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
		uint32_t op_id = 0, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Section(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
		uint32_t op_id = 0, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

	static std::shared_ptr<TopoShape> Sew(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
		uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> UnifySameDomain(const std::shared_ptr<TopoShape>& shape,
		uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

	// Splits the body's edges at any point in `points` that lies on
	// the interior of an existing edge curve. Vertices at points
	// that don't intersect any edge interior are silently ignored
	// (they may coincide with existing vertices or lie off the body
	// entirely). Used to pre-split BOP-merged edges before a
	// dressup, when the source CAD kept them as separate segments.
	// See Page_015 Fillet002: cax merges 75 mm + 85 mm outer edges
	// of Pad002 Face5 into a 163 mm closed BSpline that ChFi3d
	// can't fillet; this op restores the two separate edges so
	// fillet runs cleanly.
	static std::shared_ptr<TopoShape> SplitBodyAtPoints(
		const std::shared_ptr<TopoShape>&            shape,
		const std::vector<sm::vec3>&                 points,
		uint32_t                                     op_id = 0,
		const std::shared_ptr<brepgraph::TopoNaming>& tn   = nullptr,
		const std::shared_ptr<brepdb::VersionTree>&   vt   = nullptr);

	static std::shared_ptr<TopoShape> Translate(const std::shared_ptr<TopoShape>& shape, double x, double y, double z,
		uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Rotate(const std::shared_ptr<TopoShape>& shape,
		const sm::vec3& pos, const sm::vec3& dir, double angle,
		uint32_t op_id = 0, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Scale(const std::shared_ptr<TopoShape>& shape,
		const sm::vec3& center, double factor,
		uint32_t op_id = 0, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Transform(const std::shared_ptr<TopoShape>& shape,
		const double* mat4x4,
		uint32_t op_id = 0, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Mirror(const std::shared_ptr<TopoShape>& shape, const sm::vec3& pos, const sm::vec3& dir,
		uint32_t op_id = 0, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

	static std::shared_ptr<TopoShape> Draft(const std::shared_ptr<TopoShape>& shape, const sm::vec3& dir, float angle, float len_max,
		uint32_t op_id = 0, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> ThickSolid(const std::shared_ptr<TopoShape>& shape, const std::vector<std::shared_ptr<TopoShape>>& faces, float offset,
		uint32_t op_id = 0, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	// is_solid: true makes the result a closed solid (FreeCAD AdditiveLoft
	// expects this), false leaves it as a shell.
	static std::shared_ptr<TopoShape> ThruSections(const std::vector<std::shared_ptr<TopoShape>>& wires,
		bool is_solid = false,
		uint32_t op_id = 0, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> OffsetShape(const std::shared_ptr<TopoShape>& shape, float offset, bool is_solid,
		uint32_t op_id, const std::shared_ptr<brepgraph::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

}; // TopoAlgo

}
