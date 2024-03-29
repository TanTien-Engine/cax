#pragma once

#include <objcomp/Component.h>

namespace partgraph { class TopoFace; }

namespace breptopo
{

class NodeId : public objcomp::Component
{
public:
	NodeId(uint32_t uid, size_t graph_idx) 
		: m_uid(uid), m_graph_idx(graph_idx) 
	{
	}

	virtual const char* Type() const override { return "node_id"; }
	virtual objcomp::CompID TypeID() const override { return objcomp::GetCompTypeID<NodeId>(); }
	virtual NodeId* Clone() const override { return nullptr; }

private:
	uint32_t m_uid;
	size_t m_graph_idx;

}; // NodeId

}