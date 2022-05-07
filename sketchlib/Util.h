#pragma once

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

}