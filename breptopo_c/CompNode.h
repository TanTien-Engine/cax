#pragma once

#include "CompVariant.h"

#include <memory>

namespace graph { class Graph; }

namespace breptopo
{

class HistGraph;

class CompNode
{
public:
	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G,
		const std::shared_ptr<HistGraph>& hist) const = 0;

	void SetOpId(uint32_t op_id) { m_op_id = op_id; }

protected:
	uint32_t m_op_id = 0;

}; // CompNode

}