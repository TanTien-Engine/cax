#pragma once

#include <memory>
#include <assert.h>

namespace gs { class Shape2D; }

namespace gcsgen
{

enum class PointPos
{
    None,
    Start,
    End,
    Mid
};

struct Geometry
{
    int GetPointID(PointPos pos) const
    {
        switch (pos)
        {
        case PointPos::Start:
            return start_pt_idx;
        case PointPos::End:
            return end_pt_idx;
        case PointPos::Mid:
            return mid_pt_idx;
        default:
            assert(0);
            return -1;
        }
    }

    std::shared_ptr<gs::Shape2D> shape = nullptr;
    size_t index = 0;

    int start_pt_idx = -1;
    int mid_pt_idx = -1;
    int end_pt_idx = -1;
};

}