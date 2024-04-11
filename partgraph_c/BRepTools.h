#pragma once

#include <memory>

namespace partgraph
{

class TopoShape;

class BRepTools
{
public:
	static int FindEdgeIdx(const std::shared_ptr<TopoShape>& shape,
		const std::shared_ptr<TopoShape>& key);
	static std::shared_ptr<TopoShape> FindEdgeKey(const std::shared_ptr<TopoShape>& shape, int idx);

	static int FindFaceIdx(const std::shared_ptr<TopoShape>& shape,
		const std::shared_ptr<TopoShape>& face);
	static std::shared_ptr<TopoShape> FindFaceKey(const std::shared_ptr<TopoShape>& shape, int idx);

}; // BRepTools

}