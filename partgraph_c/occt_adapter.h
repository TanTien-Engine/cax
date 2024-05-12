#pragma once

#include "../breptopo_c/HistGraph.h"

#include <SM_Vector.h>

// OCCT
#include <gp_Pnt.hxx>

namespace partgraph
{

static gp_Pnt trans_pnt(const sm::vec3& p)
{
	return gp_Pnt(p.x, p.y, p.z);
}

static gp_Pnt2d trans_pnt(const sm::vec2& p)
{
	return gp_Pnt2d(p.x, p.y);
}

static gp_Vec trans_vec(const sm::vec3& v)
{
	return gp_Vec(v.x, v.y, v.z);
}

static gp_Dir trans_dir(const sm::vec3& v)
{
	return gp_Dir(v.x, v.y, v.z);
}

static gp_Dir2d trans_dir(const sm::vec2& v)
{
	return gp_Dir2d(v.x, v.y);
}

static gp_Ax2d trans_axis(const sm::vec2& pnt, const sm::vec2& dir)
{
	return gp_Ax2d(trans_pnt(pnt), trans_dir(dir));
}

static TopAbs_ShapeEnum trans_type(const breptopo::HistGraph::Type src)
{
    TopAbs_ShapeEnum dst;
    switch (src)
    {
    case breptopo::HistGraph::Type::Edge:
        dst = TopAbs_EDGE;
        break;
    case breptopo::HistGraph::Type::Face:
        dst = TopAbs_FACE;
        break;
    case breptopo::HistGraph::Type::Solid:
        dst = TopAbs_SOLID;
        break;
    }
    return dst;
}

}