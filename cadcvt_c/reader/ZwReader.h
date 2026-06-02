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
//   The intermediate carries "length_unit". ZW3D parts are commonly
//   millimetres, so the default scale is 0.001 (to reach the project's
//   metre convention); ReadFile overrides it from the file's
//   length_unit. SetUnitScale stays for symmetry / odd-unit overrides.
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

    // Multiplier applied to every length read from the intermediate.
    // Overridden per-file from "length_unit"; set explicitly only to
    // force a scale.
    void SetUnitScale(double s) {
        m_unit_scale = s;
    }

    // When true the reader fails on any unknown feature kind. When
    // false (default) unknown kinds become FeatPayloadOpaque and the
    // walk continues.
    void SetStrict(bool strict) {
        m_strict = strict;
    }

private:
    double m_unit_scale = 0.001;
    bool   m_strict     = false;

    uint32_t m_next_sketch_cons_id = 1;
};

} // namespace cadcvt
