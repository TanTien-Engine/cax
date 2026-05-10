#pragma once

#include <memory>

namespace breptopo { class TopoNaming; class CompGraph; }
namespace brepdb { class VersionTree; }

namespace partgraph
{

class GlobalConfig
{
public:
	auto GetTopoNaming() const { return m_topo_naming; }
	auto GetVersionTree() const { return m_version_tree; }
	auto GetCompGraph() const { return m_comp_graph; }

	static GlobalConfig* Instance();

private:
	GlobalConfig();
	~GlobalConfig();

private:
	static GlobalConfig* m_instance;

	std::shared_ptr<breptopo::TopoNaming> m_topo_naming = nullptr;
	std::shared_ptr<brepdb::VersionTree> m_version_tree = nullptr;
	std::shared_ptr<breptopo::CompGraph> m_comp_graph = nullptr;

}; // GlobalConfig

}