#pragma once

#include <memory>

namespace partgraph
{

class TopoShape;
class TopoEdge;
class TopoFace;

class BRepTools
{
public:
	static int FindEdgeIdx(const std::shared_ptr<TopoShape>& shape,
		const std::shared_ptr<TopoEdge>& key);
	static std::shared_ptr<TopoEdge> FindEdgeKey(const std::shared_ptr<TopoShape>& shape, int idx);

	static int FindFaceIdx(const std::shared_ptr<TopoShape>& shape,
		const std::shared_ptr<TopoFace>& face);
	static std::shared_ptr<TopoFace> FindFaceKey(const std::shared_ptr<TopoShape>& shape, int idx);

}; // BRepTools

}