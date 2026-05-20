#pragma once

#include <memory>
#include <string>

// ============================================================
// test/cadcvt_c/golden/GeoFingerprint.h
//
// Snapshot the replayed OCCT shape into a stable text line.
//
// This is deliberately a SEPARATE golden layer from the IR
// fingerprint. Topology counts and bounding boxes can shift when
// OCCT is upgraded (a fillet may tessellate into a different number
// of faces, a bbox may move by a tessellation epsilon). Isolating
// geometry here means an OCCT bump rewrites only the *.geo.golden
// files, and an IR-parsing regression never hides behind geometry
// noise (or vice-versa).
//
// What it records (all rounded):
//   - solids / shells / faces / edges / vertices counts
//   - axis-aligned bounding box (min / max)
//   - total surface area and volume
//
// Counts are exact (integers); the continuous quantities are
// rounded coarsely on purpose -- we want "the shape is the same
// size and shape", not "the mesh matches to 1e-9".
// ============================================================

namespace brepkit
{
class TopoShape;
}

namespace cadcvt_golden
{

struct GeoFingerprintOptions
{
    // Bounding box / area / volume rounding. Coarse by design: 2
    // places at metre scale = 10 mm, enough to catch a wrong
    // dimension while ignoring tessellation drift.
    int bbox_decimals   = 2;
    int volume_decimals = 2;
};

// Returns a one-block snapshot. shape may be null (e.g. a replay
// that produced nothing) -- in that case the output is the single
// line "geo NULL", which is itself a useful golden value.
std::string FingerprintShape(const std::shared_ptr<brepkit::TopoShape>& shape,
                             const GeoFingerprintOptions& opt = {});

} // namespace cadcvt_golden
