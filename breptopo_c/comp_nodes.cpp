#include "comp_nodes.h"
#include "NodeComp.h"
#include "BrepTopo.h"
#include "HistGraph.h"
#include "NodeShape.h"

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

	auto shape = partgraph::PrimMaker::Box(x, y, z, m_op_id);
	return std::make_shared<VarShape>(shape);
}

std::shared_ptr<CompVariant> NodeTranslate::Eval(const graph::Graph& G) const
{
	auto v_shape = calc_output_val(m_shape, G);
	auto v_offset = calc_output_val(m_offset, G);

	std::shared_ptr<partgraph::TopoShape> src;
	if (v_shape)
	{
		assert(v_shape->Type() == VAR_SHAPE);
		src = std::static_pointer_cast<VarShape>(v_shape)->val;
	}

	sm::vec3 offset;
	if (v_offset)
	{
		assert(v_offset->Type() == VAR_NUMBER3);
		offset = std::static_pointer_cast<VarNumber3>(v_offset)->val;
	}

	auto dst = partgraph::TopoAlgo::Translate(src, offset.x, offset.y, offset.z, m_op_id);
	return std::make_shared<VarShape>(dst);
}

std::shared_ptr<CompVariant> NodeOffset::Eval(const graph::Graph& G) const
{
	auto v_shape = calc_output_val(m_shape, G);
	if (!v_shape) {
		return nullptr;
	}

	auto v_offset = calc_output_val(m_offset, G);
	auto v_is_solid = calc_output_val(m_is_solid, G);

	std::shared_ptr<partgraph::TopoShape> src;
	assert(v_shape->Type() == VAR_SHAPE);
	src = std::static_pointer_cast<VarShape>(v_shape)->val;

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

	auto dst = partgraph::TopoAlgo::OffsetShape(src, offset, is_solid, m_op_id);
	return std::make_shared<VarShape>(dst);
}

std::shared_ptr<CompVariant> NodeCut::Eval(const graph::Graph& G) const
{
	auto v_shp1 = calc_output_val(m_shp1, G);
	auto v_shp2 = calc_output_val(m_shp2, G);

	std::shared_ptr<partgraph::TopoShape> shp1, shp2;
	if (v_shp1)
	{
		assert(v_shp1->Type() == VAR_SHAPE);
		shp1 = std::static_pointer_cast<VarShape>(v_shp1)->val;
	}
	if (v_shp2)
	{
		assert(v_shp2->Type() == VAR_SHAPE);
		shp2 = std::static_pointer_cast<VarShape>(v_shp2)->val;
	}

	auto dst = partgraph::TopoAlgo::Cut(shp1, shp2, m_op_id);
	return std::make_shared<VarShape>(dst);
}

std::shared_ptr<CompVariant> NodeSelector::Eval(const graph::Graph& G) const
{
	auto v_uid = calc_output_val(m_uid, G);

	uint32_t uid = 0;
	if (v_uid)
	{
		assert(v_uid->Type() == VAR_INTEGER);
		uid = std::static_pointer_cast<VarInteger>(v_uid)->val;
	}
	else
	{
		return nullptr;
	}

	auto hist = Context::Instance()->GetHist();
	std::vector<std::shared_ptr<graph::Node>> nodes;
	if (!hist->QueryNodes(uid, nodes)) {
		return nullptr;
	}

	assert(!nodes.empty());
	if (nodes.size() == 1)
	{
		auto shp = nodes[0]->GetComponent<NodeShape>().GetShape();
		return std::make_shared<VarShape>(shp);
	}
	else
	{
		std::vector<std::shared_ptr<CompVariant>> array;
		for (auto node : nodes)
		{
			auto shp = node->GetComponent<NodeShape>().GetShape();
			array.push_back(std::make_shared<VarShape>(shp));
		}
		return std::make_shared<VarArray>(array);
	}
}

std::shared_ptr<CompVariant> NodeMerge::Eval(const graph::Graph& G) const
{
	std::vector<std::shared_ptr<breptopo::CompVariant>> vals;
	for (auto node : m_nodes) 
	{
		auto val = calc_output_val(node, G);
		if (val) {
			vals.push_back(val);
		}
	}
	return std::make_shared<VarArray>(vals);
}

}