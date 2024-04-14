#include "comp_nodes.h"
#include "NodeComp.h"
#include "HistGraph.h"
#include "CompGraph.h"
#include "NodeShape.h"

#include "../partgraph_c/PrimMaker.h"
#include "../partgraph_c/TopoAlgo.h"

#include <graph/Graph.h>
#include <graph/Node.h>

namespace
{

bool is_pin_valid(int pin, const graph::Graph& cg)
{
	if (pin < 0 || pin >= cg.GetNodes().size())
		return false;
	else
		return true;
}

std::shared_ptr<breptopo::CompVariant> 
calc_output_val(int pin, const breptopo::CompGraph& cg, breptopo::HistGraph& hg)
{
	auto g = cg.GetGraph();
	if (!is_pin_valid(pin, *g)) {
		return nullptr;
	}

	auto& c_comp = g->GetNodes()[pin]->GetComponent<breptopo::NodeComp>();
	return c_comp.GetCompNode()->Eval(cg, hg);
}

void flatten_shapes(const std::shared_ptr<breptopo::CompVariant>& src, std::vector<std::shared_ptr<partgraph::TopoShape>>& dst)
{
	if (src->Type() == breptopo::VAR_ARRAY)
	{
		auto& items = std::static_pointer_cast<breptopo::VarArray>(src)->val;
		for (auto item : items)
		{
			flatten_shapes(item, dst);
		}
	}
	else if (src->Type() == breptopo::VAR_SHAPE)
	{
		auto shp = std::static_pointer_cast<breptopo::VarShape>(src)->val;
		dst.push_back(shp);
	}
}

}

namespace breptopo
{

std::shared_ptr<CompVariant> NodeBox::Eval(const CompGraph& cg, HistGraph& hg) const
{
	auto v_length = calc_output_val(m_length, cg, hg);
	auto v_width  = calc_output_val(m_width, cg, hg);
	auto v_height = calc_output_val(m_height, cg, hg);

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

	const uint16_t op_id = cg.CalcOpId(GetOpId(), 0);
	auto shape = partgraph::PrimMaker::Box(x, y, z, op_id, &hg);
	return std::make_shared<VarShape>(shape);
}

std::shared_ptr<CompVariant> NodeTranslate::Eval(const CompGraph& cg, HistGraph& hg) const
{
	auto v_shape = calc_output_val(m_shape, cg, hg);
	auto v_offset = calc_output_val(m_offset, cg, hg);

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

	const uint16_t op_id = cg.CalcOpId(GetOpId(), 0);
	auto dst = partgraph::TopoAlgo::Translate(src, offset.x, offset.y, offset.z, op_id, &hg);
	return std::make_shared<VarShape>(dst);
}

std::shared_ptr<CompVariant> NodeOffset::Eval(const CompGraph& cg, HistGraph& hg) const
{
	auto v_shape = calc_output_val(m_shape, cg, hg);
	if (!v_shape) {
		return nullptr;
	}

	auto v_offset = calc_output_val(m_offset, cg, hg);
	auto v_is_solid = calc_output_val(m_is_solid, cg, hg);

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

	if (v_shape->Type() == VAR_ARRAY)
	{
		std::vector<std::shared_ptr<partgraph::TopoShape>> src_shapes;
		flatten_shapes(v_shape, src_shapes);

		std::vector<std::shared_ptr<CompVariant>> dst;
		for (int i = 0; i < src_shapes.size(); ++i)
		{
			const uint16_t op_id = cg.CalcOpId(GetOpId(), i);
			auto dst_shape = partgraph::TopoAlgo::OffsetShape(src_shapes[i], offset, is_solid, op_id, &hg);
			dst.push_back(std::make_shared<VarShape>(dst_shape));
		}
		return std::make_shared<VarArray>(dst);
	}
	else if (v_shape->Type() == VAR_SHAPE)
	{
		auto src = std::static_pointer_cast<VarShape>(v_shape)->val;
		const uint16_t op_id = cg.CalcOpId(GetOpId(), 0);
		auto dst = partgraph::TopoAlgo::OffsetShape(src, offset, is_solid, op_id, &hg);
		return std::make_shared<VarShape>(dst);
	}
	else
	{
		return nullptr;
	}
}

std::shared_ptr<CompVariant> NodeCut::Eval(const CompGraph& cg, HistGraph& hg) const
{
	auto v_shp1 = calc_output_val(m_shp1, cg, hg);
	auto v_shp2 = calc_output_val(m_shp2, cg, hg);

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

	const uint16_t op_id = cg.CalcOpId(GetOpId(), 0);
	auto dst = partgraph::TopoAlgo::Cut(shp1, shp2, op_id, &hg);
	return std::make_shared<VarShape>(dst);
}

std::shared_ptr<CompVariant> NodeSelector::Eval(const CompGraph& cg, HistGraph& hg) const
{
	auto v_uid = calc_output_val(m_uid, cg, hg);

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

	std::vector<std::shared_ptr<graph::Node>> nodes;
	if (!hg.QueryNodes(uid, nodes)) {
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

std::shared_ptr<CompVariant> NodeMerge::Eval(const CompGraph& cg, HistGraph& hg) const
{
	std::vector<std::shared_ptr<breptopo::CompVariant>> vals;
	for (auto node : m_nodes) 
	{
		auto val = calc_output_val(node, cg, hg);
		if (val) {
			vals.push_back(val);
		}
	}
	if (vals.size() == 0)
	{
		return nullptr;
	}
	else if (vals.size() == 1)
	{
		return vals[0];
	}
	else
	{
		return std::make_shared<VarArray>(vals);
	}
}

}