#pragma once

#include <memory>

namespace breptopo { class TopoNaming; }

namespace partgraph
{

class GlobalConfig
{
public:
	std::shared_ptr<breptopo::TopoNaming> GetTopoNaming() const { return m_topo_naming; }

	static GlobalConfig* Instance();

private:
	GlobalConfig();
	~GlobalConfig();

private:
	static GlobalConfig* m_instance;

	std::shared_ptr<breptopo::TopoNaming> m_topo_naming = nullptr;

}; // GlobalConfig

}