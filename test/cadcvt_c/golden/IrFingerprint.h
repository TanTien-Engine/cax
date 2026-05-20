#pragma once

#include "cadapp_c/ir/FeatureIR.h"

#include <string>

// ============================================================
// test/cadcvt_c/golden/IrFingerprint.h
//
// Turn a cadapp::DocumentIR into a stable, line-oriented text
// snapshot suitable for golden-file diffing.
//
// Design rules that keep the snapshot diffable across runs:
//   - Deterministic order: features in document order, sketch geos
//     and constraints in id order. Never iterate an unordered_map
//     directly into the output.
//   - Rounded numbers: every double is printed via Round() at a
//     fixed precision so float jitter (last-bit differences between
//     compilers / math libs) does not churn the golden.
//   - One concern per section: the IR layer (counts, types, sketch
//     contents) lives here. Geometry (bbox / face / edge counts of
//     the replayed shape) lives in GeoFingerprint.h so an OCCT
//     version bump only rewrites that file's golden, not this one.
//
// The output is intentionally human-readable: a reviewer should be
// able to read a .golden file and understand the model without
// opening FreeCAD.
// ============================================================

namespace cadcvt_golden
{

struct FingerprintOptions
{
    // Decimal places used when printing coordinates / lengths.
    // Three places (micron at metre scale) is well above OCCT and
    // parser noise yet tight enough to catch real regressions.
    int coord_decimals = 3;

    // Decimal places for angles (radians).
    int angle_decimals = 4;

    // When true, ext_params / ext_strings are dumped (sorted by
    // key). Off by default because they are reader-experiment slots
    // and noisier than the typed payload.
    bool dump_ext = false;
};

// Build the snapshot string. The document's `source` and feature
// payloads are walked in a fixed order; see the .cpp for the exact
// line grammar.
std::string FingerprintDocument(const cadapp::DocumentIR& doc,
                                const FingerprintOptions& opt = {});

} // namespace cadcvt_golden
