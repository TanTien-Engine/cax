#pragma once

#include <cstdint>

// ============================================================
// cadapp/ir/TopoRefIR.h
//
// CAD-kernel independent topology reference descriptor.
//
// A reader emits one of these per referenced edge / face / vertex,
// describing it by geometric properties (mid / center / position
// plus tangent / normal and a few tie-breakers). At replay time
// TopoRefResolver matches against the current OCCT shape and
// writes back a TopoNaming uid. From then on the editor follows
// the uid, not the geometric match.
//
// This is the data carrier for the "geo match + TopoNaming" two
// layer strategy.
// ============================================================

namespace cadapp
{

struct TopoRefIR
{
    enum class Kind : uint8_t
    {
        Vertex = 0,
        Edge   = 1,
        Face   = 2,
    };

    Kind kind = Kind::Edge;

    // Primary matching attributes:
    //   Edge   : point = midpoint,    normal = midpoint tangent (unit)
    //   Face   : point = UV center,   normal = UV center normal
    //   Vertex : point = vertex pos,  normal = 0
    double point [3] = { 0.0, 0.0, 0.0 };
    double normal[3] = { 0.0, 0.0, 0.0 };

    // Tie-breakers for symmetric models where geo match collides:
    //   Edge   : count of faces sharing this edge (usually 2)
    //   Face   : count of faces adjacent via edges
    //   Vertex : -1
    int32_t adj_count = -1;

    //   Edge   : arc length
    //   Face   : surface area
    //   Vertex : 0
    double measure = 0.0;

    // Filled in by TopoRefResolver. Persisted alongside, so a second
    // replay can skip geometric matching:
    //   uid = 0 means unresolved or resolution failed
    //   topo_index = 1-based TopExp::MapShapes index, used as
    //   fallback when uid is still 0.
    uint32_t resolved_uid        = 0;
    int32_t  resolved_topo_index = 0;
};

} // namespace cadapp
