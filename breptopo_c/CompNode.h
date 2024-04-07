#pragma once

#include "CompVariant.h"

#include <memory>

namespace graph { class Graph; }

namespace breptopo
{

class CompNode
{
public:
	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G) const = 0;

}; // CompNode

}