#pragma once

#include <cstdint>
#include <vector>

namespace brepdb
{

enum class Type : uint8_t
{
    Empty = 0,
    Line = 1, 
    Circle = 2, 
    BSplineCurve = 3,

    Plane = 10, 
    Cylinder = 11, 
    BSplineSurface = 12,

    Vertex = 20,
    Edge = 21,
    Wire = 22,
    Face = 23,
    Shell = 24,
    Solid = 25,
    Compound = 26
};

struct alignas(64) GeomHeader
{
    Type type;
    uint32_t persistent_id;
    uint32_t param_offset;
    uint32_t param_count;
    double min_pt[3];
    double max_pt[3];
};

struct GeometryPool 
{
    std::vector<GeomHeader> headers;
    std::vector<double> data_pool;
};

}