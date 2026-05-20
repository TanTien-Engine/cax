#pragma once

#include <SM_Vector.h>

#include <memory>
#include <vector>

namespace breptopo { class TopoNaming; }
namespace brepdb { class VersionTree; }

namespace brepkit
{

class TopoShape;

class TopoAlgo
{
public:
	static std::shared_ptr<TopoShape> Fillet(const std::shared_ptr<TopoShape>& shape, double radius,
		const std::vector<std::shared_ptr<TopoShape>>& edges, uint32_t op_id,
		const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Chamfer(const std::shared_ptr<TopoShape>& shape, double dist,
		const std::vector<std::shared_ptr<TopoShape>>& edges, uint32_t op_id,
		const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

	static std::shared_ptr<TopoShape> Prism(const std::shared_ptr<TopoShape>& face, double x, double y, double z,
		uint32_t op_id = 0,
		const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

	static std::shared_ptr<TopoShape> Split(const std::shared_ptr<TopoShape>& base, const std::shared_ptr<TopoShape>& tool,
		uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Cut(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
		uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Fuse(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
		uint32_t op_id = 0, const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Common(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
		uint32_t op_id = 0, const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Section(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
		uint32_t op_id = 0, const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

	static std::shared_ptr<TopoShape> Sew(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
		uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> UnifySameDomain(const std::shared_ptr<TopoShape>& shape,
		uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

	static std::shared_ptr<TopoShape> Translate(const std::shared_ptr<TopoShape>& shape, double x, double y, double z,
		uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Rotate(const std::shared_ptr<TopoShape>& shape,
		const sm::vec3& pos, const sm::vec3& dir, double angle,
		uint32_t op_id = 0, const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Scale(const std::shared_ptr<TopoShape>& shape,
		const sm::vec3& center, double factor,
		uint32_t op_id = 0, const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Transform(const std::shared_ptr<TopoShape>& shape,
		const double* mat4x4,
		uint32_t op_id = 0, const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> Mirror(const std::shared_ptr<TopoShape>& shape, const sm::vec3& pos, const sm::vec3& dir,
		uint32_t op_id = 0, const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

	static std::shared_ptr<TopoShape> Draft(const std::shared_ptr<TopoShape>& shape, const sm::vec3& dir, float angle, float len_max,
		uint32_t op_id = 0, const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> ThickSolid(const std::shared_ptr<TopoShape>& shape, const std::vector<std::shared_ptr<TopoShape>>& faces, float offset,
		uint32_t op_id = 0, const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> ThruSections(const std::vector<std::shared_ptr<TopoShape>>& wires,
		uint32_t op_id = 0, const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);
	static std::shared_ptr<TopoShape> OffsetShape(const std::shared_ptr<TopoShape>& shape, float offset, bool is_solid,
		uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn = nullptr,
		const std::shared_ptr<brepdb::VersionTree>& vt = nullptr);

}; // TopoAlgo

}
