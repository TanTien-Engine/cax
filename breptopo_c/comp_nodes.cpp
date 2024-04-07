#include "comp_nodes.h"
#include "NodeComp.h"

#include "../partgraph_c/PrimMaker.h"
#include "../partgraph_c/TopoAlgo.h"

#include <graph/Graph.h>
#include <graph/Node.h>

namespace
{

bool is_pin_valid(int pin, const graph::Graph& G)
{
	if (pin < 0 || pin >= G.GetNodes().size())
		return false;
	else
		return true;
}

std::shared_ptr<breptopo::CompVariant> calc_output_val(int pin, const graph::Graph& G)
{
	if (!is_pin_valid(pin, G)) {
		return nullptr;
	}

	auto& c_comp = G.GetNodes()[pin]->GetComponent<breptopo::NodeComp>();
	return c_comp.GetCompNode()->Eval(G);
}

}

namespace breptopo
{

std::shared_ptr<CompVariant> NodeBox::Eval(const graph::Graph& G) const
{
	auto v_length = calc_output_val(m_length, G);
	auto v_width  = calc_output_val(m_width, G);
	auto v_height = calc_output_val(m_height, G);

	double x = 0, y = 0, z = 0;
	if (v_length)
	{
		assert(v_length->Type() == VAR_NUMBER);
		x = std::static_pointer_cast<VarNumber>(v_length)->val;
	}
	if (v_width)
	{
		assert(v_width->Type() == VAR_NUMBER);
		y = std::static_pointer_cast<VarNumber>(v_width)->val;
	}
	if (v_height)
	{
		assert(v_height->Type() == VAR_NUMBER);
		z = std::static_pointer_cast<VarNumber>(v_height)->val;
	}

	auto shape = partgraph::PrimMaker::Box(x, y, z, 0);
	return std::make_shared<VarShape>(shape);
}

std::shared_ptr<CompVariant> NodeOffset::Eval(const graph::Graph& G) const
{
	auto v_shape = calc_output_val(m_shape, G);
	auto v_offset = calc_output_val(m_offset, G);
	auto v_is_solid = calc_output_val(m_is_solid, G);

	std::shared_ptr<partgraph::TopoShape> src;
	if (v_shape)
	{
		assert(v_shape->Type() == VAR_SHAPE);
		src = std::static_pointer_cast<VarShape>(v_shape)->val;
	}

	float offset = 0;
	if (v_offset)
	{
		assert(v_offset->Type() == VAR_NUMBER);
		offset = static_cast<float>(std::static_pointer_cast<VarNumber>(v_offset)->val);
	}

	bool is_solid = 0;
	if (v_is_solid)
	{
		assert(v_is_solid->Type() == VAR_BOOLEAN);
		is_solid = std::static_pointer_cast<VarBoolean>(v_is_solid)->val;
	}

	auto dst = partgraph::TopoAlgo::OffsetShape(src, offset, is_solid, 0);
	return std::make_shared<VarShape>(dst);
}

std::shared_ptr<CompVariant> NodeMerge::Eval(const graph::Graph& G) const
{
	std::vector<std::shared_ptr<breptopo::CompVariant>> vals;
	for (auto node : m_nodes) {
		vals.push_back(calc_output_val(node, G));
	}
	return std::make_shared<VarArray>(vals);
}

}