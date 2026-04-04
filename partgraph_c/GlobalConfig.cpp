#include "GlobalConfig.h"

#include "breptopo_c/TopoNaming.h"

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
}

GlobalConfig::~GlobalConfig()
{
}

}