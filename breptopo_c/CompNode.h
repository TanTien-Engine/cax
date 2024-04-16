#pragma once

#include "CompVariant.h"

#include <memory>

namespace breptopo
{

class CompGraph;
class HistGraph;

class CompNode
{
public:
	virtual std::shared_ptr<CompVariant> Eval(CompGraph& cg, HistGraph& hg) const = 0;

	void SetOpId(int op_id) { m_op_id = op_id; }
	int GetOpId() const { return m_op_id; }

private:
	int m_op_id = -1;

}; // CompNode

}