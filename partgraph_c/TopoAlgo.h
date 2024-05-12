#pragma once

#include <SM_Vector.h>

#include <memory>
#include <vector>

namespace breptopo { class HistMgr; }

namespace partgraph
{

class TopoShape;

class TopoAlgo
{
public:
	static std::shared_ptr<TopoShape> Fillet(const std::shared_ptr<TopoShape>& shape, double radius,
		const std::vector<std::shared_ptr<TopoShape>>& edges);
	static std::shared_ptr<TopoShape> Chamfer(const std::shared_ptr<TopoShape>& shape, double dist,
		const std::vector<std::shared_ptr<TopoShape>>& edges);

	static std::shared_ptr<TopoShape> Prism(const std::shared_ptr<TopoShape>& face, double x, double y, double z);

	static std::shared_ptr<TopoShape> Cut(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2, 
		uint16_t op_id, const std::shared_ptr<breptopo::HistMgr>& hm = nullptr);
	static std::shared_ptr<TopoShape> Fuse(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2);
	static std::shared_ptr<TopoShape> Common(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2);
	static std::shared_ptr<TopoShape> Section(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2);

	static std::shared_ptr<TopoShape> Translate(const std::shared_ptr<TopoShape>& shape, double x, double y, double z, 
		uint16_t op_id, const std::shared_ptr<breptopo::HistMgr>& hm = nullptr);
	static std::shared_ptr<TopoShape> Mirror(const std::shared_ptr<TopoShape>& shape, const sm::vec3& pos, const sm::vec3& dir);

	static std::shared_ptr<TopoShape> Draft(const std::shared_ptr<TopoShape>& shape, const sm::vec3& dir, float angle, float len_max);
	static std::shared_ptr<TopoShape> ThickSolid(const std::shared_ptr<TopoShape>& shape, const std::vector<std::shared_ptr<TopoShape>>& faces, float offset);
	static std::shared_ptr<TopoShape> ThruSections(const std::vector<std::shared_ptr<TopoShape>>& wires);
	static std::shared_ptr<TopoShape> OffsetShape(const std::shared_ptr<TopoShape>& shape, float offset, bool is_solid, 
		uint16_t op_id, const std::shared_ptr<breptopo::HistMgr>& hm = nullptr);

}; // TopoAlgo

}