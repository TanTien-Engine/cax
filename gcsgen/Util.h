#pragma once

namespace gcsgen
{

using GeoID  = int;
using ConsID = int;

enum class GeoType
{
    None,
    Point,
    Line,
    Circle,
};

}