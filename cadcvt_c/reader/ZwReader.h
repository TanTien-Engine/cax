#pragma once

#include "cadcvt_c/reader/Reader.h"

#include <cstdint>
#include <string>

// ============================================================
// cadcvt/reader/ZwReader.h
//
// ZW3D neutral intermediate (.cax.json) -> cadapp::DocumentIR.
//
// Data path:
//   Unlike SolidWorks (out-of-process COM via ISldWorks, driven from
//   this process by SwReader), ZW3D exposes only an in-process plugin
//   API. So the feature-tree walk runs inside ZW3D, in the zw_export
//   plugin (see plugins/zw_export/ZwCaxExport.cpp), which serializes a
//   neutral JSON snapshot. THIS reader is purely that JSON -> DocumentIR;
//   it never links a ZW3D SDK type and has no ZW3D build dependency. The
//   only external dependency is the header-only JSON parser (nlohmann),
//   which also gates the build: when it is absent the reader compiles to
//   a stub whose ReadFile returns a clean "not built" error (mirrors how
//   SwReader stubs out without the SolidWorks type libraries). The
//   wire-format contract is interop/CaxIntermediateSchema.h.
//
//   Practically this is a two-stage flow:
//     1. In ZW3D: run the export plugin -> "<part>.cax.json".
//     2. In cax:  ZwReader::ReadFile("<part>.cax.json", doc).
//   The Replayer downstream is reader-agnostic and unchanged.
//
// Coverage in v1 (mirrors SwReader's vertical slice):
//   - sketch  : Point / Line / Arc / Circle / Ellipse / Spline on its
//               world-space plane, plus constraints.
//   - extrude : boss -> FeatType::BossExtrude, cut -> FeatType::CutExtrude.
//   - anything else -> FeatPayloadOpaque (visible in the IR, carries
//     ext_strings["zw_type"], NOT wired into the body chain), unless
//     SetStrict(true), which fails on the first unknown kind.
//
// Units:
//   The intermediate carries "length_unit". By default ReadFile resolves
//   the scale from it (mm -> 0.001, cm -> 0.01, m -> 1, in -> 0.0254) to
//   reach the project's metre convention. SetUnitScale(s) with s > 0
//   FORCES that exact scale instead, overriding length_unit -- the
//   "set explicitly to force a scale" escape hatch. s <= 0 (the default)
//   leaves it on auto.
// ============================================================

namespace cadcvt
{

class ZwReader : public Reader
{
public:
    ZwReader();
    ~ZwReader() override;

    bool ReadFile(const std::string& path,
                  cadapp::DocumentIR& out,
                  std::string*       err_msg = nullptr) override;

    const char* Name() const override {
        return "zw";
    }

    // Force the length multiplier, overriding the per-file length_unit
    // derivation. s > 0 pins the scale to s; s <= 0 (the default) leaves
    // ReadFile to resolve it from the file's length_unit.
    void SetUnitScale(double s) {
        m_forced_scale = s;
    }

    // The scale actually in effect after the last ReadFile (the forced
    // value when one was set, else resolved from the file's length_unit).
    // The ZwLoader applies it to STEP-loaded authored geometry, which OCCT
    // reads in the file's native unit (mm), so it matches the scaled
    // parametric features.
    double UnitScale() const {
        return m_unit_scale;
    }

    // When true the reader fails on any unknown feature kind. When
    // false (default) unknown kinds become FeatPayloadOpaque and the
    // walk continues.
    void SetStrict(bool strict) {
        m_strict = strict;
    }

private:
    double m_unit_scale   = 0.001;  // resolved scale, valid after ReadFile
    double m_forced_scale = 0.0;    // > 0: caller forced this; <= 0: auto
    bool   m_strict       = false;

    uint32_t m_next_sketch_cons_id = 1;
};

} // namespace cadcvt
