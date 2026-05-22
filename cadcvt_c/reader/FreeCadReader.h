#pragma once

#include "cadcvt_c/reader/Reader.h"

#include <cstdint>
#include <string>
#include <unordered_map>

// ============================================================
// cadcvt/reader/FreeCadReader.h
//
// FreeCAD .FCStd / Document.xml -> cadapp::DocumentIR.
//
// Data path:
//   .FCStd is a zip archive; we extract Document.xml via miniz
//   (header-only single-file lib). Document.xml is parsed with
//   pugixml.
//
// Coverage in v1:
//   - Sketcher::SketchObject (geometry: Line / Arc / Circle /
//     Ellipse / Spline / Point; construction flag; constraints:
//     Coincident / Horizontal / Vertical / Parallel / Perpendicular
//     / Tangent / Equal / Distance / DistanceX / DistanceY / Angle
//     / Radius / Diameter / PointOnObject / Symmetric)
//   - PartDesign::Pad        -> FeatPayloadExtrude (BossExtrude)
//   - PartDesign::Pocket     -> FeatPayloadExtrude (CutExtrude)
//   - PartDesign::Revolution -> FeatPayloadRevolve (BossRevolve)
//   - PartDesign::Groove     -> FeatPayloadRevolve (CutRevolve)
//   - PartDesign::Fillet     -> FeatPayloadFillet
//   - PartDesign::Chamfer    -> FeatPayloadChamfer
//   - PartDesign::Thickness  -> FeatPayloadShell
//   - Part::Box / Cylinder / Sphere / Cone / Torus -> Prim*
//
// Out of scope in v1 (logged into DocumentIR via Opaque):
//   - Loft, Pipe (Sweep)
//   - Mirrored / LinearPattern / PolarPattern
//   - Body / Part container hierarchy (we process features in
//     document order; assumes the file lists features in modeling
//     order, which is the FreeCAD default)
//
// Topo references:
//   FreeCAD names edges / faces by string ("Sketch.Edge3"). v1
//   stashes those names in FeatureIR::ext_strings keyed
//   "<edge|face>_ref_<i>_name" so a future FreeCAD-aware
//   TopoRefResolver can resolve them. The generic geo-match path
//   inside TopoRefResolver will not match (point/normal are 0).
//
// Units:
//   FreeCAD stores lengths in millimetres internally. We multiply
//   by m_unit_scale on the way out (default 0.001 = millimetres
//   -> metres, matching the project convention).
// ============================================================

namespace cadcvt
{

class FreeCadReader : public Reader
{
public:
    FreeCadReader();
    ~FreeCadReader() override;

    bool ReadFile(const std::string& path,
                  cadapp::DocumentIR& out,
                  std::string*       err_msg = nullptr) override;

    const char* Name() const override {
        return "freecad";
    }

    // Multiplier applied to every length read from the file. The
    // default (0.001) maps FreeCAD millimetres into project metres.
    void SetUnitScale(double s) {
        m_unit_scale = s;
    }

    // When true the reader fails on any unknown feature type.
    // When false (default) unknown features become FeatPayloadOpaque
    // and continue.
    void SetStrict(bool strict) {
        m_strict = strict;
    }

private:
    // Internal: parse Document.xml text into out. Both ReadFile
    // entry points (.FCStd via miniz and bare .xml) funnel here.
    bool ParseDocumentXml(const char*       xml_data,
                          size_t            xml_size,
                          cadapp::DocumentIR& out,
                          std::string*      err_msg);

    // Unzip path's Document.xml into a memory buffer. Returns true
    // on success and fills out_text + out_size. Caller frees with
    // FreeXmlBuffer.
    bool ExtractDocumentXml(const std::string& path,
                            char**             out_text,
                            size_t*            out_size,
                            std::string*       err_msg);

    static void FreeXmlBuffer(char* buf);

private:
    double m_unit_scale = 0.001;
    bool   m_strict     = false;

    // freecad object name -> our feature id, used to resolve
    // BaseFeature / Profile references between features.
    std::unordered_map<std::string, uint32_t> m_name_to_id;

    // Used by sketches to convert the FreeCAD "geometry index" into
    // the cadapp::SkGeoIR::id we assign at read time.
    std::unordered_map<int, uint32_t> m_sk_geo_idx_to_id;

    uint32_t m_next_feature_id = 1;
    uint32_t m_next_sketch_geo_id = 1;
    uint32_t m_next_sketch_cons_id = 1;
};

} // namespace cadcvt
