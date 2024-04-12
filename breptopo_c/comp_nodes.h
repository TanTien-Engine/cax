#pragma once

#include "CompNode.h"
#include "comp_variants.h"

namespace breptopo
{

class NodeInteger : public CompNode
{
public:
	NodeInteger(int val) : m_val(val) {}

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G, 
		const std::shared_ptr<HistGraph>& hist) const
	{
		return std::make_shared<VarInteger>(m_val);
	}

private:
	int m_val = 0;

}; // NodeInteger

class NodeNumber : public CompNode
{
public:
	NodeNumber(float val) : m_val(val) {}

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G, 
		const std::shared_ptr<HistGraph>& hist) const
	{
		return std::make_shared<VarNumber>(m_val);
	}

private:
	float m_val = 0;

}; // NodeNumber

class NodeNumber3 : public CompNode
{
public:
	NodeNumber3(const sm::vec3& val) : m_val(val) {}

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G,
		const std::shared_ptr<HistGraph>& hist) const
	{
		return std::make_shared<VarNumber3>(m_val);
	}

private:
	sm::vec3 m_val;

}; // NodeNumber3

class NodeBoolean : public CompNode
{
public:
	NodeBoolean(bool val) : m_val(val) {}

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G,
		const std::shared_ptr<HistGraph>& hist) const
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

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G,
		const std::shared_ptr<HistGraph>& hist) const
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

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G,
		const std::shared_ptr<HistGraph>& hist) const;

private:
	int m_length = -1, m_width = -1, m_height = -1;

}; // NodeBox

class NodeTranslate : public CompNode
{
public:
	NodeTranslate(int shape, int offset)
		: m_shape(shape), m_offset(offset) {}

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G,
		const std::shared_ptr<HistGraph>& hist) const;

private:
	int m_shape = -1, m_offset = -1;

}; // NodeTranslate

class NodeOffset : public CompNode
{
public:
	NodeOffset(int shape, int offset, int is_solid)
		: m_shape(shape), m_offset(offset), m_is_solid(is_solid) {}

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G,
		const std::shared_ptr<HistGraph>& hist) const;

private:
	int m_shape = -1, m_offset = -1, m_is_solid = -1;

}; // NodeOffset

class NodeCut : public CompNode
{
public:
	NodeCut(int shp1, int shp2)
		: m_shp1(shp1), m_shp2(shp2) {}

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G,
		const std::shared_ptr<HistGraph>& hist) const;

private:
	int m_shp1 = -1, m_shp2 = -1;

}; // NodeCut

class NodeSelector : public CompNode
{
public:
	NodeSelector(int uid) : m_uid(uid) {}

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G,
		const std::shared_ptr<HistGraph>& hist) const;

private:
	int m_uid;

}; // NodeSelector

class NodeMerge : public CompNode
{
public:
	NodeMerge(const std::vector<int>& nodes)
		: m_nodes(nodes) {}

	virtual std::shared_ptr<CompVariant> Eval(const graph::Graph& G,
		const std::shared_ptr<HistGraph>& hist) const;

private:
	std::vector<int> m_nodes;

}; // NodeMerge

}