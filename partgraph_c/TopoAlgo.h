#pragma once

#include <memory>

namespace partgraph
{

class TopoShape;

class TopoAlgo
{
public:
	static std::shared_ptr<TopoShape> Fillet(const std::shared_ptr<TopoShape>& shape, double thickness);
	static std::shared_ptr<TopoShape> Chamfer(const std::shared_ptr<TopoShape>& shape, double dist);

}; // TopoAlgo

}