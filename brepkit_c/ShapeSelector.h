#pragma once

#include <SM_Ray.h>

#include <memory>

namespace brepkit
{

class TopoShape;

class ShapeSelector
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

	// Geometric match: return the edge / face of `shape` that best matches a
	// reference sub-shape `ref` by bounding-box centre + size (length / area),
	// independent of TopoNaming. Lets a rebuilt dressup Selector re-find its
	// edge by the import's geo-resolved geometry when the TopoNaming uid has
	// drifted (rebuild op_id divergence on complex parts).
	static std::shared_ptr<TopoShape> MatchEdge(const std::shared_ptr<TopoShape>& shape, const std::shared_ptr<TopoShape>& ref);

	static std::shared_ptr<TopoShape> MatchFace(const std::shared_ptr<TopoShape>& shape, const std::shared_ptr<TopoShape>& ref);

}; // ShapeSelector

}