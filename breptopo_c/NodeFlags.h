#pragma once

#include <objcomp/Component.h>

namespace breptopo
{

class NodeFlags : public objcomp::Component
{
public:
	NodeFlags()
	{
	}

	virtual const char* Type() const override { return "node_flags"; }
	virtual objcomp::CompID TypeID() const override { return objcomp::GetCompTypeID<NodeFlags>(); }
	virtual NodeFlags* Clone() const override { return nullptr; }

	void SetActive(bool active) { m_active = active; }
	bool IsActive() const { return m_active; }

private:
	bool m_active = true;

}; // NodeFlags

}