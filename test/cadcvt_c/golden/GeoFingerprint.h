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
    // Bounding box rounding. 2 decimal places at metre scale = 10 mm
    // -- coarse on purpose, enough to catch a wrong dimension while
    // ignoring tessellation drift.
    int bbox_decimals   = 2;

    // Area / volume rounding. Need finer than bbox because the
    // numbers are in m^2 / m^3 and small parts have tiny values:
    // a 20x10x5 mm pad has volume = 1e-6 m^3, which would round to
    // 0.00 at 2 places and lose all signal. 8 places = mm^3 / mm^2
    // resolution at metre scale, fine enough to discriminate while
    // staying above OCCT noise.
    int volume_decimals = 8;
};

// Returns a one-block snapshot. shape may be null (e.g. a replay
// that produced nothing) -- in that case the output is the single
// line "geo NULL", which is itself a useful golden value.
std::string FingerprintShape(const std::shared_ptr<brepkit::TopoShape>& shape,
                             const GeoFingerprintOptions& opt = {});

} // namespace cadcvt_golden
