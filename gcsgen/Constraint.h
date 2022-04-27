#pragma once

#include <memory>

namespace gs { class Shape2D; }

namespace gcsgen
{

enum class ConstraintType
{
	None,
	Distance,
	Horizontal,
	Vertical,
};

struct Constraint
{
	Constraint(ConstraintType type, const std::shared_ptr<gs::Shape2D>& geo1,
		const std::shared_ptr<gs::Shape2D>& geo2, double val) 
		: type(type), geo1(geo1), geo2(geo2), value(val) {}

	ConstraintType type = ConstraintType::None;

	std::shared_ptr<gs::Shape2D> geo1 = nullptr;
	std::shared_ptr<gs::Shape2D> geo2 = nullptr;

	mutable double value = 0.0;

}; // Constraint

}