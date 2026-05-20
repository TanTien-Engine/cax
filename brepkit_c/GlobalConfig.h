#pragma once

#include <memory>

namespace brepgraph { class TopoNaming; class CalcGraph; }
namespace brepdb { class VersionTree; }

namespace brepkit
{

class GlobalConfig
{
public:
	auto GetTopoNaming() const { return m_topo_naming; }
	auto GetVersionTree() const { return m_version_tree; }
	auto GetCalcGraph() const { return m_calc_graph; }

	static GlobalConfig* Instance();

private:
	GlobalConfig();
	~GlobalConfig();

private:
	static GlobalConfig* m_instance;

	std::shared_ptr<brepgraph::TopoNaming> m_topo_naming = nullptr;
	std::shared_ptr<brepdb::VersionTree> m_version_tree = nullptr;
	std::shared_ptr<brepgraph::CalcGraph> m_calc_graph = nullptr;

}; // GlobalConfig

}