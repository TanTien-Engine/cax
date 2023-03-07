#pragma once

#include "occt_adapter.h"

// OCCT
#include <Geom_CylindricalSurface.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_TrimmedCurve.hxx>

namespace partgraph
{

class CylindricalSurface
{
public:
	CylindricalSurface(const sm::vec3& pos, const sm::vec3& dir, float radius)
	{
		m_shape = new Geom_CylindricalSurface(gp_Ax2(trans_pnt(pos), trans_dir(dir)), radius);
	}

	auto GetShape() const { return m_shape; }

private:
	Handle(Geom_CylindricalSurface) m_shape;

}; // CylindricalSurface

class EllipseCurve
{
public:
	EllipseCurve(const sm::vec2& pos, const sm::vec2& dir, 
		const float major_radius, const float minor_radius)
	{
		m_shape = new Geom2d_Ellipse(trans_axis(pos, dir), major_radius, minor_radius);
	}

	auto GetShape() const { return m_shape; }

private:
	Handle(Geom2d_Ellipse) m_shape;

}; // EllipseCurve

class TrimmedCurve
{
public:
	TrimmedCurve(const EllipseCurve& curve, float U1, float U2)
	{
		m_shape = new Geom2d_TrimmedCurve(curve.GetShape(), U1, U2);
	}

	auto GetShape() const { return m_shape; }

private:
	Handle(Geom2d_TrimmedCurve) m_shape;

}; // TrimmedCurve

}