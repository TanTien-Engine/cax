#pragma once

#include <objcomp/Component.h>

#include <string>

namespace breptopo
{

class NodeInfo : public objcomp::Component
{
public:
	NodeInfo(const std::string& desc)
		: m_desc(desc)
	{
	}

	virtual const char* Type() const override { return "node_desc"; }
	virtual objcomp::CompID TypeID() const override { return objcomp::GetCompTypeID<NodeInfo>(); }
	virtual NodeInfo* Clone() const override { return nullptr; }

	auto GetDesc() const { return m_desc; }

private:
	std::string m_desc;

}; // NodeInfo


}