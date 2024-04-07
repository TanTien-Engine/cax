#pragma once

#include "CompNode.h"
#include "comp_variants.h"

namespace breptopo
{

class NodeNumber : public CompNode
{
public:
	NodeNumber(float val) : m_val(val) {}

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G) const
	{
		return std::make_shared<VarNumber>(m_val);
	}

private:
	float m_val = 0;

}; // NodeNumber

class NodeBoolean : public CompNode
{
public:
	NodeBoolean(bool val) : m_val(val) {}

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G) const
	{
		return std::make_shared<VarBoolean>(m_val);
	}

private:
	bool m_val = false;

}; // NodeBoolean 

class NodeTopoShape : public CompNode
{
public:
	NodeTopoShape(const std::shared_ptr<partgraph::TopoShape>& val) : m_val(val) {}

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G) const
	{
		return std::make_shared<VarShape>(m_val);
	}

private:
	std::shared_ptr<partgraph::TopoShape> m_val = nullptr;

}; // NodeTopoShape

class NodeBox : public CompNode
{
public:
	NodeBox(int length, int width, int height)
		: m_length(length), m_width(width), m_height(height) {}

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G) const;

private:
	int m_length = -1, m_width = -1, m_height = -1;

}; // NodeBox

class NodeOffset : public CompNode
{
public:
	NodeOffset(int shape, int offset, int is_solid)
		: m_shape(shape), m_offset(offset), m_is_solid(is_solid) {}

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G) const;

private:
	int m_shape = -1, m_offset = -1, m_is_solid = -1;

}; // NodeOffset

class NodeMerge : public CompNode
{
public:
	NodeMerge(const std::vector<int>& nodes)
		: m_nodes(nodes) {}

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G) const;

private:
	std::vector<int> m_nodes;

}; // NodeMerge

}