#include "GlobalConfig.h"

#include <breptopo_c/TopoNaming.h>
#include <breptopo_c/CompGraph.h>
#include <brepdb_c/VersionTree.h>

namespace partgraph
{

GlobalConfig* GlobalConfig::m_instance = nullptr;

GlobalConfig* GlobalConfig::Instance()
{
	if (!m_instance) {
		m_instance = new GlobalConfig();
	}
	return m_instance;
}

GlobalConfig::GlobalConfig()
{
	m_topo_naming = std::make_shared<breptopo::TopoNaming>();
	m_version_tree = std::make_shared<brepdb::VersionTree>();
	m_comp_graph = std::make_shared<breptopo::CompGraph>();
}

GlobalConfig::~GlobalConfig()
{
}

}