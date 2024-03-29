#pragma once

#include "core/macro.h"

#include <memory>

namespace breptopo
{

void init_cb();

class HistGraph;

class Context
{
public:

	auto GetHist() const { return m_hist; }

private:
	std::shared_ptr<HistGraph> m_hist;

	TT_SINGLETON_DECLARATION(Context)

}; // Context

}