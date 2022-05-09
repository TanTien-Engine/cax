#pragma once

#include "Util.h"

#include <utility>

namespace sketchlib
{

enum class ConsType
{
	None,

	// basic
	Distance,
	DistanceX,
	DistanceY,
	Angle,
	Parallel,
	Perpendicular,
	Coincident,
	Horizontal,
	Vertical,
	Equal,

	// point on
	PointOnLine,
	PointOnCircle,
	PointOnArc,
	PointOnEllipse,
	PointOnPerpBisector,
	MidpointOnLine,

	// tangent
	Tangent,
	TangentCircumf,

	// params
	CircleRadius,
	CircleDiameter,
	ArcRadius,
	ArcDiameter,
};

struct Constraint
{
	Constraint(ConsID id, ConsType type, const std::pair<GeoID, GeoType>& geo1, 
		const std::pair<GeoID, GeoType>& geo2, double val)
		: id(id), type(type), geo1(geo1), geo2(geo2), value(val) 
	{
	}

	ConsID id = -1;

	ConsType type = ConsType::None;

	std::pair<GeoID, GeoType> geo1, geo2;

	double value = 0.0;

}; // Constraint

}