#include "comp_nodes.h"
#include "NodeComp.h"
#include "HistMgr.h"
#include "HistGraph.h"
#include "CompGraph.h"
#include "NodeShape.h"
#include "NodeInfo.h"

#include "../partgraph_c/PrimMaker.h"
#include "../partgraph_c/TopoAlgo.h"

#include <graph/Graph.h>
#include <graph/Node.h>

#include <set>

#define DYNAMIC_UPDATE
#define COPY_OUTPUT

namespace
{

bool is_pin_valid(int pin, const breptopo::CompGraph& cg)
{
	if (pin < 0 || pin >= cg.GetNodesNum())
		return false;
	else
		return true;
}

std::shared_ptr<breptopo::CompVariant> 
calc_output_val(int pin, breptopo::CompGraph& cg, const std::shared_ptr<breptopo::HistMgr>& hm)
{
	if (!is_pin_valid(pin, cg)) {
		return nullptr;
	}

	auto node = cg.GetNode(pin);
	auto& c_comp = node->GetComponent<breptopo::NodeComp>();
	return c_comp.GetCompNode()->Eval(cg, hm, node->GetId());
}

void flatten_vars(const std::shared_ptr<breptopo::CompVariant>& src, std::vector<std::shared_ptr<breptopo::CompVariant>>& dst, int type)
{
	if (src->Type() == breptopo::VAR_ARRAY)
	{
		auto& items = std::static_pointer_cast<breptopo::VarArray>(src)->val;
		for (auto item : items)
		{
			flatten_vars(item, dst, type);
		}
	}
	else if (src->Type() == type)
	{
		dst.push_back(src);
	}
}

}

namespace breptopo
{

std::shared_ptr<CompVariant> NodeBox::Eval(CompGraph& cg, const std::shared_ptr<HistMgr>& hm, int node_id) const
{
	auto v_length = calc_output_val(m_length, cg, hm);
	auto v_width  = calc_output_val(m_width, cg, hm);
	auto v_height = calc_output_val(m_height, cg, hm);

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
	auto shape = partgraph::PrimMaker::Box(x, y, z, op_id, hm);
	return std::make_shared<VarShape>(shape);
}

std::shared_ptr<CompVariant> NodeTranslate::Eval(CompGraph& cg, const std::shared_ptr<HistMgr>& hm, int node_id) const
{
	auto v_shape = calc_output_val(m_shape, cg, hm);
	auto v_offset = calc_output_val(m_offset, cg, hm);

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
	auto dst = partgraph::TopoAlgo::Translate(src, offset.x, offset.y, offset.z, op_id, hm);
	return std::make_shared<VarShape>(dst);
}

std::shared_ptr<CompVariant> NodeOffset::Eval(CompGraph& cg, const std::shared_ptr<HistMgr>& hm, int node_id) const
{
	auto v_shape = calc_output_val(m_shape, cg, hm);
	if (!v_shape) {
		return nullptr;
	}

	auto v_offset = calc_output_val(m_offset, cg, hm);
	auto v_is_solid = calc_output_val(m_is_solid, cg, hm);

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
//		// update graph
//#if 1
//		std::vector<std::shared_ptr<breptopo::CompVariant>> v_shape_array;
//		flatten_vars(v_shape, v_shape_array, VAR_SHAPE);
//
//		std::vector<size_t> output;
//
//		for (auto v_shape : v_shape_array)
//		{
//			auto shape = std::static_pointer_cast<breptopo::VarShape>(v_shape)->val;
//			auto shp_node = std::make_shared<NodeTopoShape>(shape);
//			size_t idx = cg.GetNodesNum();
//			cg.AddNode(shp_node, "shape");
//			cg.AddEdge(m_shape, idx);
//			output.push_back(idx);
//		}
//
//		size_t merge_idx = cg.GetNodesNum();
//		auto merge_node = std::make_shared<NodeMerge>(output);
//		cg.AddNode(merge_node, "merge");
//		for (auto i : output) 
//		{
//			cg.AddEdge(i, merge_idx);
//		}
//
//		int old_output_idx = -1;
//		auto conns = cg.GetNode(GetOpId())->GetConnects();
//		for (auto itr = conns.begin(); itr != conns.end(); )
//		{
//			if ((*itr)->GetId() == m_shape ||
//				(*itr)->GetId() == m_offset ||
//				(*itr)->GetId() == m_is_solid)
//			{
//				itr = conns.erase(itr);
//			}
//			else
//			{
//				++itr;
//			}
//		}
//		assert(conns.size() == 1);
//		cg.RemoveEdge(m_shape, GetOpId());
//		cg.AddEdge(merge_idx, GetOpId());
//
//		cg.Layout();
//#endif

		std::vector<std::shared_ptr<breptopo::CompVariant>> v_shape_array;
		flatten_vars(v_shape, v_shape_array, VAR_SHAPE);

		std::vector<std::shared_ptr<CompVariant>> dst;
		for (int i = 0; i < v_shape_array.size(); ++i)
		{
			auto src_shape = std::static_pointer_cast<breptopo::VarShape>(v_shape_array[i])->val;
			const uint16_t op_id = cg.CalcOpId(GetOpId(), i);
			auto dst_shape = partgraph::TopoAlgo::OffsetShape(src_shape, offset, is_solid, op_id, hm);
			dst.push_back(std::make_shared<VarShape>(dst_shape));
		}
		return std::make_shared<VarArray>(dst);
	}
	else if (v_shape->Type() == VAR_SHAPE)
	{
		auto src = std::static_pointer_cast<VarShape>(v_shape)->val;
		const uint16_t op_id = cg.CalcOpId(GetOpId(), 0);
		auto dst = partgraph::TopoAlgo::OffsetShape(src, offset, is_solid, op_id, hm);
		return std::make_shared<VarShape>(dst);
	}
	else
	{
		return nullptr;
	}
}

void NodeOffset::Update(const CompGraph& cg, int node_id)
{
	std::vector<size_t> nodes;

	auto& all_nodes = cg.GetGraph()->GetNodes();

	auto& edges = cg.GetGraph()->GetEdges();
	for (auto edge : edges)
	{
		if (edge.second == node_id && all_nodes[edge.first])
		{
			auto& cncomp = all_nodes[edge.first]->GetComponent<NodeComp>();
			auto shp_val = std::dynamic_pointer_cast<NodeShapeValue>(cncomp.GetCompNode());
			if (shp_val)
			{
				m_shape = static_cast<int>(edge.first);
				break;
			}
		}
	}
}

std::shared_ptr<CompVariant> NodeCut::Eval(CompGraph& cg, const std::shared_ptr<HistMgr>& hm, int node_id) const
{
	auto v_shp1 = calc_output_val(m_shp1, cg, hm);
	auto v_shp2 = calc_output_val(m_shp2, cg, hm);

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
	auto dst = partgraph::TopoAlgo::Cut(shp1, shp2, op_id, hm);
	return std::make_shared<VarShape>(dst);
}

std::shared_ptr<CompVariant> NodeSelector::Eval(CompGraph& cg, const std::shared_ptr<HistMgr>& hm, int node_id) const
{
	auto v_uid = calc_output_val(m_uid, cg, hm);

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
	if (!hm || !hm->GetFaceGraph()->QueryNodes(uid, nodes)) {
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
#ifdef DYNAMIC_UPDATE
		UpdateGraph(cg, node_id, nodes);
#endif // DYNAMIC_UPDATE

		std::vector<std::shared_ptr<CompVariant>> array;
		for (auto node : nodes)
		{
			auto shp = node->GetComponent<NodeShape>().GetShape();
			array.push_back(std::make_shared<VarShape>(shp));
		}
		return std::make_shared<VarArray>(array);
	}
}

void NodeSelector::UpdateGraph(CompGraph& cg, int node_id, const std::vector<std::shared_ptr<graph::Node>>& nodes) const
{
	auto G = cg.GetGraph();
	auto& edges = G->GetEdges();
	auto range = edges.equal_range(node_id);

	// already connected shapes
	std::set<std::shared_ptr<partgraph::TopoShape>> exists;
	auto& all_nodes = G->GetNodes();
	for (auto itr = range.first; itr != range.second; ++itr) 
	{
		assert(itr->second < all_nodes.size());
		auto node = all_nodes[itr->second];
		auto& cncomp = node->GetComponent<NodeComp>();
		auto shp_val = std::dynamic_pointer_cast<NodeShapeValue>(cncomp.GetCompNode());
		if (shp_val)
		{
			exists.insert(shp_val->GetShape());
		}
	}

	// if need update
	if (nodes.size() == exists.size())
	{
		bool need_update = false;
		for (auto node : nodes)
		{
			auto shp = node->GetComponent<NodeShape>().GetShape();
			if (exists.find(shp) == exists.end())
			{
				need_update = true;
				break;
			}
		}
		if (!need_update)
		{
			return;
		}
	}

	std::vector<int> out_nodes;
	for (auto itr = range.first; itr != range.second; ++itr) 
	{
		out_nodes.push_back(static_cast<int>(itr->second));
	}

	std::set<int> update_nodes;

	for (auto node : nodes)
	{
		auto shp = node->GetComponent<NodeShape>().GetShape();
		auto node_shp = std::make_shared<NodeShapeValue>(shp);
		int shp_node_id = cg.AddNode(node_shp, "shp_val");
		cg.AddEdge(node_id, shp_node_id);

#ifdef COPY_OUTPUT
		for (auto src_node_idx : out_nodes)
		{
			auto& src_node = all_nodes[src_node_idx];

			auto& src_c_comp = src_node->GetComponent<breptopo::NodeComp>();
			auto& src_c_info = src_node->GetComponent<breptopo::NodeInfo>();
	
			auto dst_c_comp = src_c_comp.GetCompNode()->Clone();
			auto dst_node = cg.AddNode(dst_c_comp, src_c_info.GetDesc());

			// copy conns
			std::set<int> out_conns;
			auto range = cg.GetGraph()->GetEdges().equal_range(src_node_idx);
			for (auto itr = range.first; itr != range.second; ++itr)
			{
				out_conns.insert(static_cast<int>(itr->second));
			}
			auto& conns = src_node->GetConnects();
			for (auto conn : conns)
			{
				int id = conn->GetId();
				if (id == node_id) {
					continue;
				}

				if (out_conns.find(id) != out_conns.end())
				{
					cg.AddEdge(dst_node, id);
					update_nodes.insert(id);
				}
				else
				{
					cg.AddEdge(id, dst_node);
				}
			}

			cg.AddEdge(shp_node_id, /*ori_out_node*/dst_node);

			dst_c_comp->Update(cg, dst_node);
		} 
#else
		for (auto src_node_idx : out_nodes)
		{
			cg.AddEdge(shp_node_id, src_node_idx);
		}
#endif // COPY_OUTPUT
	}

	for (auto out_node : out_nodes)
	{
		cg.RemoveEdge(node_id, out_node);
#ifdef COPY_OUTPUT
		cg.RemoveNode(out_node);
#endif // COPY_OUTPUT
	}

	for (auto node : update_nodes)
	{
		auto& src_node = all_nodes[node];
		auto& src_c_comp = src_node->GetComponent<breptopo::NodeComp>();
		src_c_comp.GetCompNode()->Update(cg, node);
	}
}

std::shared_ptr<CompVariant> NodeMerge::Eval(CompGraph& cg, const std::shared_ptr<HistMgr>& hm, int node_id) const
{
	auto& edges = cg.GetGraph()->GetEdges();
	auto range = edges.equal_range(node_id);

	std::vector<std::shared_ptr<breptopo::CompVariant>> vals;
	for (auto node : m_nodes) 
	{
		auto val = calc_output_val(static_cast<int>(node), cg, hm);
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

void NodeMerge::Update(const CompGraph& cg, int node_id)
{
	std::vector<size_t> nodes;

	auto& all_nodes = cg.GetGraph()->GetNodes();

	auto& edges = cg.GetGraph()->GetEdges();
	for (auto edge : edges)
	{
		if (edge.second == node_id)
		{
			if (all_nodes[edge.first])
			{
				nodes.push_back(edge.first);
			}
		}
	}

	m_nodes = nodes;
}

//std::shared_ptr<CompVariant> NodeSplit::Eval(CompGraph& cg, const std::shared_ptr<HistMgr>& hm, int node_id) const
//{
//	auto v_src = calc_output_val(m_src, cg, hm);
//	if (!v_src) {
//		return nullptr;
//	}
//
//	if (v_src->Type() != VAR_ARRAY) {
//		return v_src;
//	}
//
//	std::vector<std::shared_ptr<breptopo::CompVariant>> v_array;
//	flatten_vars(v_src, v_array, VAR_SHAPE);
//
//	if (v_array.size() == 0)
//	{
//		return nullptr;
//	}
//	else if (v_array.size() == 1)
//	{
//		return v_array[0];
//	}
//	else
//	{
//
//	}
//}

std::shared_ptr<CompVariant> NodeShapeValue::Eval(CompGraph& cg, const std::shared_ptr<HistMgr>& hm, int node_id) const
{
	return std::make_shared<VarShape>(m_shp);
}

}