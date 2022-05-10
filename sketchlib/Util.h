#pragma once

#include <utility>

namespace sketchlib
{

using GeoID  = int;
using ConsID = int;

enum class GeoType
{
    None,
    Point,
    Line,
    Circle,
    Arc,
    Ellipse,

    GeoPtStart,
    GeoPtMid,
    GeoPtEnd,
};

using Geo = std::pair<GeoID, GeoType>;

}