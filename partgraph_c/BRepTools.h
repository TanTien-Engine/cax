#pragma once

#include <memory>
#include <vector>

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

	static std::vector<std::shared_ptr<TopoShape>> MapShells(const std::shared_ptr<TopoShape>& shape);
	static std::vector<std::shared_ptr<TopoShape>> MapFaces(const std::shared_ptr<TopoShape>& shape);
	static std::vector<std::shared_ptr<TopoShape>> MapEdges(const std::shared_ptr<TopoShape>& shape);

}; // BRepTools

}