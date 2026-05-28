// Smoke test: read asm4-test.FCStd through FreeCadReader and dump
// the resulting features. No OGDF dependency.

#include "cadcvt_c/reader/FreeCadReader.h"
#include "cadapp_c/ir/FeatureIR.h"
#include "cadapp_c/ir/Enums.h"

#include <cstdio>
#include <cstring>
#include <string>

const char* TypeName(cadapp::FeatType t)
{
    using T = cadapp::FeatType;
    switch (t) {
        case T::Unknown:         return "Unknown";
        case T::Sketch:          return "Sketch";
        case T::BossExtrude:     return "BossExtrude";
        case T::CutExtrude:      return "CutExtrude";
        case T::BossRevolve:     return "BossRevolve";
        case T::CutRevolve:      return "CutRevolve";
        case T::Loft:            return "Loft";
        case T::Sweep:           return "Sweep";
        case T::Rib:             return "Rib";
        case T::Fillet:          return "Fillet";
        case T::Chamfer:         return "Chamfer";
        case T::Shell:           return "Shell";
        case T::Draft:           return "Draft";
        case T::Offset:          return "Offset";
        case T::Translate:       return "Translate";
        case T::Rotate:          return "Rotate";
        case T::Mirror:          return "Mirror";
        case T::Scale:           return "Scale";
        case T::LinearPattern:   return "LinearPattern";
        case T::CircularPattern: return "CircularPattern";
        case T::MultiTransform:  return "MultiTransform";
        case T::Fuse:            return "Fuse";
        case T::Cut:             return "Cut";
        case T::Common:          return "Common";
        case T::PrimBox:         return "PrimBox";
        case T::PrimCylinder:    return "PrimCylinder";
        case T::PrimCone:        return "PrimCone";
        case T::PrimSphere:      return "PrimSphere";
        case T::PrimTorus:       return "PrimTorus";
        case T::PrimEllipsoid:   return "PrimEllipsoid";
        case T::HoleWizard:      return "HoleWizard";
        case T::Link:            return "Link";
        case T::AsmConstraint:   return "AsmConstraint";
        case T::BakedShape:      return "BakedShape";
    }
    return "???";
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <path.FCStd>\n", argv[0]);
        return 1;
    }

    cadcvt::FreeCadReader reader;
    cadapp::DocumentIR doc;
    std::string err;

    bool ok = reader.ReadFile(argv[1], doc, &err);

    std::printf("=== %s ===\n", argv[1]);
    std::printf("ok=%d  err=%s\n", (int)ok, err.empty() ? "(none)" : err.c_str());
    std::printf("source=%s  sketches=%zu  features=%zu  authored_shapes=%zu\n",
                doc.source.c_str(),
                doc.sketches.size(),
                doc.features.size(),
                doc.authored_shapes.size());

    std::printf("\nfeatures:\n");
    for (const auto& f : doc.features)
    {
        std::printf("  #%-4u %-15s  name=%s",
                    f.id, TypeName(f.type), f.name.c_str());

        if (f.type == cadapp::FeatType::Link)
        {
            auto* pl = std::get_if<cadapp::FeatPayloadLink>(&f.data);
            if (pl)
            {
                std::printf("\n         linked_file=%s  linked_object=%s  sub_tip=#%u\n"
                            "         placement Px=%g Py=%g Pz=%g  axis=(%g,%g,%g) angle=%g",
                            pl->linked_file.c_str(),
                            pl->linked_object_name.c_str(),
                            pl->sub_tip_feature_id,
                            pl->placement_px, pl->placement_py, pl->placement_pz,
                            pl->placement_ox, pl->placement_oy, pl->placement_oz,
                            pl->placement_angle);
            }
        }
        else if (f.type == cadapp::FeatType::BossExtrude
              || f.type == cadapp::FeatType::CutExtrude)
        {
            auto* pl = std::get_if<cadapp::FeatPayloadExtrude>(&f.data);
            if (pl) std::printf("  sketch_id=#%u", pl->sketch_id);
        }
        else if (f.type == cadapp::FeatType::Sketch)
        {
            auto* pl = std::get_if<cadapp::FeatPayloadSketch>(&f.data);
            if (pl) std::printf("  sketch_id=#%u", pl->sketch_id);
        }
        else if (f.type == cadapp::FeatType::HoleWizard)
        {
            auto* pl = std::get_if<cadapp::FeatPayloadHoleWizard>(&f.data);
            if (pl)
            {
                std::printf("\n         sketch_id=#%u  diameter=%gm  depth=%gm  through_all=%d",
                            pl->sketch_id,
                            pl->diameter,
                            pl->depth,
                            (int)pl->through_all);
                // Spill a curated subset of the ext bag (the rest are
                // mostly thread / cosmetic knobs).
                auto dump = [&](const char* key, const char* label) {
                    auto it = f.ext_params.find(key);
                    if (it != f.ext_params.end())
                        std::printf("  %s=%g", label, it->second);
                };
                std::printf("\n        ");
                dump("threaded", "threaded");
                dump("hole_cut_type", "hole_cut_type");
                dump("hole_cut_diameter", "hole_cut_d");
                dump("hole_cut_depth", "hole_cut_h");
                dump("drill_point_kind", "drill_pt");
                dump("drill_point_angle_deg", "drill_ang");
                dump("thread_pitch", "pitch");
            }
        }
        else if (f.type == cadapp::FeatType::AsmConstraint)
        {
            auto* pl = std::get_if<cadapp::FeatPayloadAsmConstraint>(&f.data);
            if (pl)
            {
                std::printf("\n         linked_link=#%u  parent_link=#%u  attached=%s\n"
                            "         first_lcs=%s  second_lcs=%s\n"
                            "         offset Px=%g Py=%g Pz=%g  axis=(%g,%g,%g) angle=%g",
                            pl->linked_link_feature_id,
                            pl->parent_link_feature_id,
                            pl->is_attached_to.c_str(),
                            pl->first_lcs_name.c_str(),
                            pl->second_lcs_name.c_str(),
                            pl->offset_px, pl->offset_py, pl->offset_pz,
                            pl->offset_ox, pl->offset_oy, pl->offset_oz,
                            pl->offset_angle);
            }
        }

        // material (transparency + override flag)
        if (f.material.present)
        {
            std::printf("  mat[tr=%.2f override=%d diff=0x%08x]",
                        f.material.transparency,
                        (int)f.material.has_override,
                        f.material.diffuse_rgba);
        }

        // body / link tags
        auto bit = f.ext_strings.find("freecad_body");
        if (bit != f.ext_strings.end()) {
            std::printf("  body=%s", bit->second.c_str());
        }
        auto sit = f.ext_strings.find("link_source");
        if (sit != f.ext_strings.end()) {
            std::printf("  link_source=%s", sit->second.c_str());
        }
        if (!f.input_feature_ids.empty())
        {
            std::printf("  inputs=[");
            for (size_t i = 0; i < f.input_feature_ids.size(); ++i)
            {
                std::printf("%s%u/r%u",
                            i ? "," : "",
                            f.input_feature_ids[i],
                            (unsigned)f.input_roles[i]);
            }
            std::printf("]");
        }
        std::printf("\n");
    }
    return ok ? 0 : 2;
}
