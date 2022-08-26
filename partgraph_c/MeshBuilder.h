#pragma once

#include <memory>

namespace ur { class VertexArray; }

namespace partgraph
{

class TopoShape;

class MeshBuilder
{
public:
	static std::shared_ptr<ur::VertexArray> Build(const TopoShape& topo);

}; // MeshBuilder

}