#pragma once

#include <SM_Ray.h>

#include <memory>

namespace partgraph
{

class TopoShape;

class BRepSelector
{
public:
	static std::shared_ptr<TopoShape> SelectFace(const std::shared_ptr<TopoShape>& shape, int index);

	enum class FacePos
	{
		X_MIN,
		X_MAX,
		Y_MIN,
		Y_MAX,
		Z_MIN,
		Z_MAX
	};
	static std::shared_ptr<TopoShape> SelectFace(const std::shared_ptr<TopoShape>& shape, FacePos pos);

	static std::shared_ptr<TopoShape> SelectFace(const std::shared_ptr<TopoShape>& shape, const sm::Ray& ray);

	static std::shared_ptr<TopoShape> SelectEdge(const std::shared_ptr<TopoShape>& shape, const sm::Ray& ray);

}; // BRepSelector

}