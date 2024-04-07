#pragma once

#include <objcomp/Component.h>

namespace breptopo
{

class CompNode;

class NodeComp : public objcomp::Component
{
public:
	NodeComp(const std::shared_ptr<CompNode>& cnode)
		: m_cnode(cnode)
	{
	}

	virtual const char* Type() const override { return "node_comp"; }
	virtual objcomp::CompID TypeID() const override { return objcomp::GetCompTypeID<NodeComp>(); }
	virtual NodeComp* Clone() const override { return nullptr; }

	auto GetCompNode() const { return m_cnode; }

private:
	std::shared_ptr<CompNode> m_cnode;

}; // NodeId

}