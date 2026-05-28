#pragma once

#include "cadcvt_c/reader/Reader.h"
#include "cadapp_c/ir/FeatureIR.h"

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
//   - PartDesign::Pad             -> FeatPayloadExtrude (BossExtrude)
//   - PartDesign::Pocket          -> FeatPayloadExtrude (CutExtrude)
//   - PartDesign::Revolution      -> FeatPayloadRevolve (BossRevolve)
//   - PartDesign::Groove          -> FeatPayloadRevolve (CutRevolve)
//   - PartDesign::AdditivePipe /
//     SubtractivePipe             -> FeatPayloadSweep
//   - PartDesign::AdditiveLoft /
//     SubtractiveLoft             -> FeatPayloadLoft
//   - PartDesign::Fillet          -> FeatPayloadFillet
//   - PartDesign::Chamfer         -> FeatPayloadChamfer
//   - PartDesign::Thickness       -> FeatPayloadShell
//   - Part::Thickness             -> FeatPayloadShell
//   - Part::Box / Cylinder / Sphere / Cone / Torus -> Prim*
//
// Out of scope (logged into DocumentIR via Opaque):
//   - Body / Part container hierarchy (we process features in
//     document order; assumes the file lists features in modeling
//     order, which is the FreeCAD default)
//
// Topo references:
//   FreeCAD names edges / faces by string ("Sketch.Edge3"). The
//   names get stashed in FeatureIR::ext_strings keyed
//   "<edge|face>_ref_<i>_name" for diagnostics.
//
//   For .FCStd inputs the reader additionally seeds TopoRefIR with
//   geometry (point / normal / measure) read from the referent
//   feature's authored BRep (.brp inside the zip): MapShapes the
//   target sub-shape and lift centroid / normal / area, scaled to
//   project metres. The generic geo-match path in TopoRefResolver
//   then finds the corresponding sub-shape on cax's replayed body
//   within tolerance. For raw .xml fixtures (no .brp), refs stay
//   zero and resolution returns 0 -- Fillet/Chamfer skip cleanly.
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
    // FreeXmlBuffer. Implementation reuses m_zip if already open.
    bool ExtractDocumentXml(const std::string& path,
                            char**             out_text,
                            size_t*            out_size,
                            std::string*       err_msg);

    // Same as ExtractDocumentXml but for GuiDocument.xml (the GUI /
    // view side of a .FCStd; carries ShapeMaterial / Transparency /
    // OverrideMaterial per ViewProvider). Optional input: returns
    // false silently when the entry is absent (some hand-edited /
    // headless-generated .FCStd files lack it). Caller frees with
    // FreeXmlBuffer.
    bool ExtractGuiDocumentXml(const std::string& path,
                               char**             out_text,
                               size_t*            out_size);

    // Populate m_name_to_material from GuiDocument.xml bytes. Parses
    // each ViewProvider's ShapeMaterial property (and App::Link's
    // OverrideMaterial bool when present) and stores the keyed
    // MaterialIR. Called once per .FCStd up-front; the main ParseDocumentXml
    // emission loop then looks up each emitted feature's name in
    // the resulting map.
    void ParseGuiDocumentXml(const char* xml_data, size_t xml_size);

    static void FreeXmlBuffer(char* buf);

    // Open / close the .FCStd zip backing m_zip. Open is a no-op
    // if path is not .FCStd; close is idempotent. Called by ReadFile
    // around ParseDocumentXml so StashRefNames can pull .brp entries
    // on demand.
    bool OpenArchive(const std::string& path, std::string* err_msg);
    void CloseArchive();

private:
    double m_unit_scale = 0.001;
    bool   m_strict     = false;

    // freecad object name -> our feature id, used to resolve
    // BaseFeature / Profile references between features.
    std::unordered_map<std::string, uint32_t> m_name_to_id;

    // Used by sketches to convert the FreeCAD "geometry index" into
    // the cadapp::SkGeoIR::id we assign at read time.
    std::unordered_map<int, uint32_t> m_sk_geo_idx_to_id;

    // freecad object name -> "PartShape3.brp" (filename inside the
    // .FCStd zip carrying that feature's serialized OCCT shape).
    // Populated from Part::PropertyPartShape properties during the
    // ObjectData scan; empty for raw .xml fixtures.
    std::unordered_map<std::string, std::string> m_feat_brep_path;

    // freecad object name -> MaterialIR. Populated up-front from
    // GuiDocument.xml's ViewProviderData by ParseGuiDocumentXml().
    // The main emission loop in ParseDocumentXml looks each emitted
    // feature's name up here and copies the material onto
    // FeatureIR::material. Empty for raw .xml fixtures or .FCStd
    // archives that omit GuiDocument.xml.
    std::unordered_map<std::string, cadapp::MaterialIR> m_name_to_material;

    // Opaque mz_zip_archive*; non-null only while a .FCStd is being
    // parsed. Owned by OpenArchive/CloseArchive.
    void* m_zip = nullptr;

    uint32_t m_next_feature_id = 1;
    uint32_t m_next_sketch_geo_id = 1;
    uint32_t m_next_sketch_cons_id = 1;
};

} // namespace cadcvt
