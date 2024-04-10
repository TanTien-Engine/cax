#pragma once

#include "CompVariant.h"

#include <SM_Vector.h>

#include <memory>
#include <vector>

namespace partgraph { class TopoShape; }

namespace breptopo
{

enum VarType
{
	VAR_INTEGER,
	VAR_NUMBER,
	VAR_NUMBER3,
	VAR_BOOLEAN,
	VAR_SHAPE,
	VAR_ARRAY,
};

class VarInteger : public CompVariant
{
public:
	VarInteger(int _val) : val(_val) {}

	virtual int Type() const { return VAR_INTEGER; }

public:
	int val = 0;

}; // VarInteger

class VarNumber : public CompVariant
{
public:
	VarNumber(float _val) : val(_val) {}

	virtual int Type() const { return VAR_NUMBER; }

public:
	float val = 0;

}; // VarNumber

class VarNumber3 : public CompVariant
{
public:
	VarNumber3(const sm::vec3& _val) : val(_val) {}

	virtual int Type() const { return VAR_NUMBER3; }

public:
	sm::vec3 val;

}; // VarNumber3

class VarBoolean : public CompVariant
{
public:
	VarBoolean(bool _val) : val(_val) {}

	virtual int Type() const { return VAR_BOOLEAN; }

public:
	bool val = 0;

}; // VarBoolean

class VarShape : public CompVariant
{
public:
	VarShape(const std::shared_ptr<partgraph::TopoShape>& _val) : val(_val) {}

	virtual int Type() const { return VAR_SHAPE; }

public:
	std::shared_ptr<partgraph::TopoShape> val = nullptr;

}; // VarShape

class VarArray : public CompVariant
{
public:
	VarArray(const std::vector<std::shared_ptr<CompVariant>>& _val) : val(_val) {}

	virtual int Type() const { return VAR_ARRAY; }

public:
	std::vector<std::shared_ptr<CompVariant>> val;

}; // VarArray

}