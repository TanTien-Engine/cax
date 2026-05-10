#include "CompGraphBuilder.h"
#include "CompGraph.h"
#include "NodeInfo.h"

#include <graph/Graph.h>
#include <graph/Node.h>
#include <graph/GraphLayout.h>

#include <sstream>
#include <unordered_map>

namespace breptopo
{

static std::string FormatStepVal(const Val& v)
{
	std::string s;
	std::visit([&](auto&& val) {
		using T = std::decay_t<decltype(val)>;
		if constexpr (std::is_same_v<T, int>)         s = std::to_string(val);
		else if constexpr (std::is_same_v<T, double>) {
			std::ostringstream os; os << val; s = os.str();
		}
		else if constexpr (std::is_same_v<T, bool>)   s = val ? "true" : "false";
		else if constexpr (std::is_same_v<T, Vec3>) {
			std::ostringstream os;
			os << "(" << val[0] << ", " << val[1] << ", " << val[2] << ")";
			s = os.str();
		}
		else if constexpr (std::is_same_v<T, ShapeVal>) s = "[shape]";
	}, v);
	return s;
}

std::shared_ptr<graph::Graph>
CompGraphBuilder::BuildGraph(const CompGraph& cg)
{
	auto g = std::make_shared<graph::Graph>();

	auto& history = cg.GetHistory();
	auto& steps = history.Steps();

	std::unordered_map<int, size_t> step2gid;

	for (auto& step : steps)
	{
		size_t gid = g->GetNodesNum();
		step2gid[step.step_id] = gid;

		auto node = std::make_shared<graph::Node>();
		node->SetValue(static_cast<int>(gid));

		std::string name = step.op_name;
		if (!step.desc.empty()) {
			name = step.desc;
		}
		node->SetName(name);

		std::ostringstream desc;
		desc << step.op_name;
		auto val_str = FormatStepVal(step.imm);
		if (!val_str.empty()) {
			desc << " = " << val_str;
		}
		if (!step.desc.empty()) {
			desc << " (" << step.desc << ")";
		}
		node->AddComponent<NodeInfo>(desc.str());

		g->AddNode(node);
	}

	for (auto& step : steps)
	{
		auto it_to = step2gid.find(step.step_id);
		if (it_to == step2gid.end()) continue;
		size_t to_gid = it_to->second;

		for (auto inp : step.inputs)
		{
			auto it = step2gid.find(inp);
			if (it != step2gid.end()) {
				g->AddEdge(it->second, to_gid);
			}
		}
		for (auto inp : step.var_inputs)
		{
			auto it = step2gid.find(inp);
			if (it != step2gid.end()) {
				g->AddEdge(it->second, to_gid);
			}
		}
	}

	graph::GraphLayout::OptimalHierarchy(*g);
	return g;
}

}
