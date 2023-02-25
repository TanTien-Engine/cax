#pragma once

#include <SM_Vector.h>

#include <memory>

namespace partgraph
{

class TopoShape;
class TopoFace;

class TopoAlgo
{
public:
	static std::shared_ptr<TopoShape> Fillet(const std::shared_ptr<TopoShape>& shape, double radius);
	static std::shared_ptr<TopoShape> Chamfer(const std::shared_ptr<TopoShape>& shape, double dist);

	static std::shared_ptr<TopoShape> Prism(const std::shared_ptr<TopoFace>& face, double x, double y, double z);

	static std::shared_ptr<TopoShape> Cut(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2);
	static std::shared_ptr<TopoShape> Fuse(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2);
	static std::shared_ptr<TopoShape> Common(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2);
	static std::shared_ptr<TopoShape> Section(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2);

	static std::shared_ptr<TopoShape> Translate(const std::shared_ptr<TopoShape>& shape, double x, double y, double z);
	static std::shared_ptr<TopoShape> Mirror(const std::shared_ptr<TopoShape>& shape, const sm::vec3& pos, const sm::vec3& dir);

	static std::shared_ptr<TopoShape> Draft(const std::shared_ptr<TopoShape>& shape, const sm::vec3& dir, float angle, float len_max);
	static std::shared_ptr<TopoShape> ThickSolid(const std::shared_ptr<TopoShape>& shape, const std::shared_ptr<TopoFace>& face, float offset);

}; // TopoAlgo

}