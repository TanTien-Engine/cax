#pragma once

#include "cadcvt_c/reader/Reader.h"
#include "cadapp_c/ir/FeatureIR.h"

#include <cstdint>
#include <string>
#include <unordered_map>

// ============================================================
// cadcvt/reader/SwReader.h
//
// SolidWorks .SLDPRT / .SLDASM -> cadapp::DocumentIR.
//
// Data path:
//   Unlike FreeCAD (.FCStd = zip + XML, parsed offline), SolidWorks
//   files are a proprietary OLE compound binary. The ONLY way to get
//   the parametric feature tree is to drive the installed SolidWorks
//   via its COM automation API (ISldWorks). So this reader, when it
//   ReadFile()s, connects to / launches SLDWORKS.exe, OpenDoc6's the
//   part, walks the FeatureManager design tree, and reads each
//   feature's typed definition. Requires SolidWorks installed and
//   licensed on the machine; gated at build time by CAX_SW_OK (see
//   CMakeLists / the generated SwImports.h carrying the #import paths).
//
// Per the Reader.h contract, NO COM / SolidWorks SDK type appears in
// this header -- it is confined to SwReader.cpp behind a pimpl, so the
// rest of cax never transitively includes the SolidWorks type library
// headers.
//
// Coverage in v1 (vertical slice):
//   - ProfileFeature (sketch)  : geometry Line / Arc / Circle, on its
//                                world-space plane (read from
//                                ISketch::IModelToSketchTransform), plus
//                                relations + dimensions read from
//                                ISketchRelationManager -> SkConsIR.
//   - Extrusion (boss)         -> FeatPayloadExtrude (BossExtrude)
//   - Cut (cut-extrude)        -> FeatPayloadExtrude (CutExtrude)
//
// Everything else (RefPlane, OriginProfileFeature, the system folders
// CommentsFolder / SolidBodyFolder / ... ) is skipped. Unknown
// feature kinds become FeatPayloadOpaque unless SetStrict(true).
//
// Units:
//   The SolidWorks API reports lengths in SI metres already, so the
//   default unit scale is 1.0 (FreeCAD's reader uses 0.001 because
//   FreeCAD stores millimetres). The project convention is metres.
//
// Names:
//   SolidWorks feature names are LOCALIZED (e.g. a Chinese install
//   names a sketch "草图1"). The reader must therefore dispatch on
//   IFeature::GetTypeName2() (a stable, language-independent English
//   token such as "ProfileFeature" / "Extrusion") and never on the
//   display name. Within-document name matching (linking an Extrusion
//   to its profile sketch) is still fine because both sides see the
//   same localized string.
// ============================================================

namespace cadcvt
{

class SwReader : public Reader
{
public:
    SwReader();
    ~SwReader() override;

    bool ReadFile(const std::string& path,
                  cadapp::DocumentIR& out,
                  std::string*       err_msg = nullptr) override;

    const char* Name() const override {
        return "sw";
    }

    // Multiplier applied to every length read from SolidWorks. The
    // default (1.0) keeps the API's native SI metres; exposed only for
    // symmetry with FreeCadReader and odd-unit experiments.
    void SetUnitScale(double s) {
        m_unit_scale = s;
    }

    // When true the reader fails on any unknown feature type. When
    // false (default) unknown features become FeatPayloadOpaque and
    // the walk continues.
    void SetStrict(bool strict) {
        m_strict = strict;
    }

private:
    double m_unit_scale = 1.0;
    bool   m_strict     = false;

    // SolidWorks feature display-name -> our feature id. Used to wire
    // an Extrusion/Cut to the profile sketch it consumes (the sketch
    // is the feature's first ProfileFeature sub-feature; we look its
    // name up here). Names are localized but consistent within a doc.
    std::unordered_map<std::string, uint32_t> m_name_to_id;

    uint32_t m_next_feature_id     = 1;
    uint32_t m_next_sketch_geo_id  = 1;
    uint32_t m_next_sketch_cons_id = 1;
};

} // namespace cadcvt
