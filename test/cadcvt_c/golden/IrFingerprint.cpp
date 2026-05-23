#include "IrFingerprint.h"

#include "cadapp_c/ir/SketchIR.h"
#include "cadapp_c/ir/TopoRefIR.h"

#include <cmath>
#include <map>
#include <sstream>
#include <variant>

// ============================================================
// test/cadcvt_c/golden/IrFingerprint.cpp
//
// Line grammar (one concept per line, stable order):
//
//   doc source=<reader>
//   sketch <feature_id> name=<name> plane_origin=(x,y,z) plane_normal=(x,y,z) plane_xdir=(x,y,z) geos=<n> cons=<m>
//     geo <id> type=<Line|Arc|...> constr=<0|1> params=[a,b,...]
//     con <id> type=<Coincident|...> a=<geo_id>:<pos> b=<geo_id>:<pos> value=<v> driving=<0|1>
//   feat <id> name=<name> type=<FeatType> suppressed=<0|1> payload=<...>
//
// Anything the reader cannot type lands as `payload=opaque` with
// its preserved freecad_type, so an Opaque feature is still visible
// in the diff (you can see coverage grow as Opaque lines turn into
// typed ones).
// ============================================================

namespace cadcvt_golden
{

namespace
{

// Round to `decimals` places and normalise -0 to 0 so the textual
// output never flips sign on a zero.
double Round(double v, int decimals)
{
    double scale = std::pow(10.0, decimals);
    double r = std::round(v * scale) / scale;
    if (r == 0.0) {
        return 0.0;
    }
    return r;
}

std::string Num(double v, int decimals)
{
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(decimals);
    os << Round(v, decimals);
    return os.str();
}

std::string Vec3(const double v[3], int decimals)
{
    std::ostringstream os;
    os << "(" << Num(v[0], decimals)
       << "," << Num(v[1], decimals)
       << "," << Num(v[2], decimals) << ")";
    return os.str();
}

const char* GeoTypeName(cadapp::SkGeoType t)
{
    switch (t)
    {
    case cadapp::SkGeoType::Point:   return "Point";
    case cadapp::SkGeoType::Line:    return "Line";
    case cadapp::SkGeoType::Arc:     return "Arc";
    case cadapp::SkGeoType::Circle:  return "Circle";
    case cadapp::SkGeoType::Ellipse: return "Ellipse";
    case cadapp::SkGeoType::Spline:  return "Spline";
    default:                         return "None";
    }
}

const char* ConsTypeName(cadapp::SkConsType t)
{
    switch (t)
    {
    case cadapp::SkConsType::Distance:            return "Distance";
    case cadapp::SkConsType::DistanceX:           return "DistanceX";
    case cadapp::SkConsType::DistanceY:           return "DistanceY";
    case cadapp::SkConsType::Angle:               return "Angle";
    case cadapp::SkConsType::Parallel:            return "Parallel";
    case cadapp::SkConsType::Perpendicular:       return "Perpendicular";
    case cadapp::SkConsType::Coincident:          return "Coincident";
    case cadapp::SkConsType::Horizontal:          return "Horizontal";
    case cadapp::SkConsType::Vertical:            return "Vertical";
    case cadapp::SkConsType::Equal:               return "Equal";
    case cadapp::SkConsType::PointOnLine:         return "PointOnLine";
    case cadapp::SkConsType::PointOnCircle:       return "PointOnCircle";
    case cadapp::SkConsType::PointOnArc:          return "PointOnArc";
    case cadapp::SkConsType::PointOnEllipse:      return "PointOnEllipse";
    case cadapp::SkConsType::PointOnPerpBisector: return "PointOnPerpBisector";
    case cadapp::SkConsType::MidpointOnLine:      return "MidpointOnLine";
    case cadapp::SkConsType::Tangent:             return "Tangent";
    case cadapp::SkConsType::TangentCircumf:      return "TangentCircumf";
    case cadapp::SkConsType::CircleRadius:        return "CircleRadius";
    case cadapp::SkConsType::CircleDiameter:      return "CircleDiameter";
    case cadapp::SkConsType::ArcRadius:           return "ArcRadius";
    case cadapp::SkConsType::ArcDiameter:         return "ArcDiameter";
    case cadapp::SkConsType::Symmetric:           return "Symmetric";
    case cadapp::SkConsType::Concentric:          return "Concentric";
    case cadapp::SkConsType::Colinear:            return "Colinear";
    case cadapp::SkConsType::Fix:                 return "Fix";
    default:                                      return "None";
    }
}

const char* FeatTypeName(cadapp::FeatType t)
{
    switch (t)
    {
    case cadapp::FeatType::Sketch:          return "Sketch";
    case cadapp::FeatType::BossExtrude:     return "BossExtrude";
    case cadapp::FeatType::CutExtrude:      return "CutExtrude";
    case cadapp::FeatType::BossRevolve:     return "BossRevolve";
    case cadapp::FeatType::CutRevolve:      return "CutRevolve";
    case cadapp::FeatType::Loft:            return "Loft";
    case cadapp::FeatType::Sweep:           return "Sweep";
    case cadapp::FeatType::Rib:             return "Rib";
    case cadapp::FeatType::Fillet:          return "Fillet";
    case cadapp::FeatType::Chamfer:         return "Chamfer";
    case cadapp::FeatType::Shell:           return "Shell";
    case cadapp::FeatType::Draft:           return "Draft";
    case cadapp::FeatType::Offset:          return "Offset";
    case cadapp::FeatType::Translate:       return "Translate";
    case cadapp::FeatType::Rotate:          return "Rotate";
    case cadapp::FeatType::Mirror:          return "Mirror";
    case cadapp::FeatType::Scale:           return "Scale";
    case cadapp::FeatType::LinearPattern:   return "LinearPattern";
    case cadapp::FeatType::CircularPattern: return "CircularPattern";
    case cadapp::FeatType::MultiTransform:  return "MultiTransform";
    case cadapp::FeatType::Fuse:            return "Fuse";
    case cadapp::FeatType::Cut:             return "Cut";
    case cadapp::FeatType::Common:          return "Common";
    case cadapp::FeatType::PrimBox:         return "PrimBox";
    case cadapp::FeatType::PrimCylinder:    return "PrimCylinder";
    case cadapp::FeatType::PrimCone:        return "PrimCone";
    case cadapp::FeatType::PrimSphere:      return "PrimSphere";
    case cadapp::FeatType::PrimTorus:       return "PrimTorus";
    case cadapp::FeatType::HoleWizard:      return "HoleWizard";
    default:                                return "Unknown";
    }
}

// Print a sketch and its geos / cons in id order.
void EmitSketch(std::ostringstream& os, const cadapp::SketchIR& sk, const FingerprintOptions& opt)
{
    os << "sketch " << sk.feature_id
       << " name=" << sk.name
       << " plane_origin=" << Vec3(sk.plane_origin, opt.coord_decimals)
       << " plane_normal=" << Vec3(sk.plane_normal, opt.coord_decimals)
       << " plane_xdir=" << Vec3(sk.plane_x_dir, opt.coord_decimals)
       << " geos=" << sk.geos.size()
       << " cons=" << sk.cons.size()
       << "\n";

    // Geos: copy ids into a sorted index so output order does not
    // depend on the reader's insertion order.
    std::map<uint32_t, const cadapp::SkGeoIR*> geo_by_id;
    for (const auto& g : sk.geos) {
        geo_by_id[g.id] = &g;
    }
    for (const auto& kv : geo_by_id)
    {
        const cadapp::SkGeoIR* g = kv.second;
        bool is_angle_geo = (g->type == cadapp::SkGeoType::Arc);
        os << "  geo " << g->id
           << " type=" << GeoTypeName(g->type)
           << " constr=" << (g->construction ? 1 : 0)
           << " params=[";
        for (size_t i = 0; i < g->params.size(); ++i)
        {
            if (i) {
                os << ",";
            }
            // Arc's last two params are angles; everything else is a
            // coordinate / radius. Splines store a leading count.
            bool angleField = is_angle_geo && (i >= 3);
            int dec = angleField ? opt.angle_decimals : opt.coord_decimals;
            os << Num(g->params[i], dec);
        }
        os << "]\n";
    }

    std::map<uint32_t, const cadapp::SkConsIR*> con_by_id;
    for (const auto& c : sk.cons) {
        con_by_id[c.id] = &c;
    }
    for (const auto& kv : con_by_id)
    {
        const cadapp::SkConsIR* c = kv.second;
        os << "  con " << c->id
           << " type=" << ConsTypeName(c->type)
           << " a=" << c->a.geo_id << ":" << (int)c->a.point_pos
           << " b=" << c->b.geo_id << ":" << (int)c->b.point_pos
           << " value=" << Num(c->value, opt.coord_decimals)
           << " driving=" << (c->driving ? 1 : 0)
           << "\n";
    }
}

// Dispatch on the payload variant and emit a one-line summary of
// the typed parameters. Each branch prints only the fields that
// matter for regression detection.
void EmitPayload(std::ostringstream& os, const cadapp::FeatureIR& feat, const FingerprintOptions& opt)
{
    const int dec = opt.coord_decimals;
    const int adec = opt.angle_decimals;

    std::visit([&](const auto& p)
    {
        using T = std::decay_t<decltype(p)>;

        if constexpr (std::is_same_v<T, cadapp::FeatPayloadSketch>) {
            os << "sketch sketch_id=" << p.sketch_id;
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadExtrude>) {
            os << "extrude sketch_id=" << p.sketch_id
               << " dir=" << Vec3(p.direction, dec)
               << " dist=" << Num(p.distance, dec)
               << " dist2=" << Num(p.distance2, dec)
               << " end=" << (int)p.end_type
               << " flip=" << (p.flip_direction ? 1 : 0);
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadRevolve>) {
            os << "revolve sketch_id=" << p.sketch_id
               << " axis_o=" << Vec3(p.axis_origin, dec)
               << " axis_d=" << Vec3(p.axis_dir, dec)
               << " angle=" << Num(p.angle, adec)
               << " flip=" << (p.flip_direction ? 1 : 0);
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadLoft>) {
            os << "loft profile_count=" << p.profile_sketch_ids.size()
               << " closed=" << (p.closed ? 1 : 0);
            for (size_t li = 0; li < p.profile_sketch_ids.size(); ++li) {
                os << (li == 0 ? " profiles=[" : ",")
                   << p.profile_sketch_ids[li];
            }
            if (!p.profile_sketch_ids.empty()) {
                os << "]";
            }
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadSweep>) {
            // Spine sketch id rides in ext_params (see reader notes);
            // surface it here so the golden line carries both the
            // profile and the path the sweep follows.
            uint32_t spine_id = 0xFFFFFFFF;
            auto sit = feat.ext_params.find("spine_sketch_id");
            if (sit != feat.ext_params.end()) {
                spine_id = (uint32_t)sit->second;
            }
            os << "sweep profile_sketch_id=" << p.profile_sketch_id
               << " spine_sketch_id=" << spine_id;
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadFillet>) {
            os << "fillet radius=" << Num(p.radius, dec)
               << " edges=" << p.edges.size();
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadChamfer>) {
            os << "chamfer d1=" << Num(p.distance1, dec)
               << " d2=" << Num(p.distance2, dec)
               << " edges=" << p.edges.size();
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadShell>) {
            os << "shell thickness=" << Num(p.thickness, dec)
               << " open_faces=" << p.faces_to_open.size();
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadMirror>) {
            os << "mirror plane_o=" << Vec3(p.plane_origin, dec)
               << " plane_n=" << Vec3(p.plane_normal, dec);
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadLinearPattern>) {
            os << "linpat dir1=" << Vec3(p.dir1, dec)
               << " count1=" << p.count1
               << " spacing1=" << Num(p.spacing1, dec)
               << " count2=" << p.count2
               << " spacing2=" << Num(p.spacing2, dec);
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadCircularPattern>) {
            os << "circpat axis_o=" << Vec3(p.axis_origin, dec)
               << " axis_d=" << Vec3(p.axis_dir, dec)
               << " count=" << p.count
               << " total_angle=" << Num(p.total_angle, adec);
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadMultiTransform>) {
            os << "multixform steps=" << p.steps.size();
            for (size_t si = 0; si < p.steps.size(); ++si) {
                const auto& s = p.steps[si];
                os << " [" << si << ":";
                if (s.kind == cadapp::MultiTransformStep::Kind::Mirror) {
                    os << "mirror plane_n=" << Vec3(s.plane_normal, dec);
                } else if (s.kind == cadapp::MultiTransformStep::Kind::LinearPattern) {
                    os << "linpat dir1=" << Vec3(s.dir1, dec)
                       << " count1=" << s.count1
                       << " spacing1=" << Num(s.spacing1, dec);
                } else {
                    os << "circpat axis_d=" << Vec3(s.axis_dir, dec)
                       << " count=" << s.count
                       << " total_angle=" << Num(s.total_angle, adec);
                }
                os << "]";
            }
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadPrimBox>) {
            os << "box l=" << Num(p.length, dec)
               << " w=" << Num(p.width, dec)
               << " h=" << Num(p.height, dec);
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadPrimCylinder>) {
            os << "cylinder r=" << Num(p.radius, dec)
               << " h=" << Num(p.height, dec);
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadPrimCone>) {
            os << "cone r1=" << Num(p.radius1, dec)
               << " r2=" << Num(p.radius2, dec)
               << " h=" << Num(p.height, dec);
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadPrimSphere>) {
            os << "sphere r=" << Num(p.radius, dec);
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadPrimTorus>) {
            os << "torus R=" << Num(p.major_radius, dec)
               << " r=" << Num(p.minor_radius, dec);
        }
        else if constexpr (std::is_same_v<T, cadapp::FeatPayloadOpaque>) {
            os << "opaque";
            auto it = p.strings.find("freecad_type");
            if (it != p.strings.end()) {
                os << " freecad_type=" << it->second;
            }
        }
        else {
            // Any payload type without a dedicated branch above. The
            // type name still appears on the feat line, so this only
            // hides the parameters, not the feature's existence.
            os << "other";
        }
    }, feat.data);
}

} // anonymous namespace

std::string FingerprintDocument(const cadapp::DocumentIR& doc, const FingerprintOptions& opt)
{
    std::ostringstream os;

    os << "doc source=" << doc.source << "\n";

    // Sketches first, in feature_id order, so the section is stable
    // regardless of how the reader queued them.
    std::map<uint32_t, const cadapp::SketchIR*> sk_by_id;
    for (const auto& sk : doc.sketches) {
        sk_by_id[sk.feature_id] = &sk;
    }
    for (const auto& kv : sk_by_id) {
        EmitSketch(os, *kv.second, opt);
    }

    // Features in document order (this order is meaningful: it is
    // the modeling order the Replayer consumes).
    for (const auto& feat : doc.features)
    {
        os << "feat " << feat.id
           << " name=" << feat.name
           << " type=" << FeatTypeName(feat.type)
           << " suppressed=" << (feat.suppressed ? 1 : 0)
           << " payload=";
        EmitPayload(os, feat, opt);
        os << "\n";

        if (opt.dump_ext)
        {
            // ext maps are already std::map (sorted), but copy
            // through std::map again to stay robust if the IR ever
            // switches to unordered_map.
            std::map<std::string, double> sorted_params(feat.ext_params.begin(),
                                                         feat.ext_params.end());
            for (const auto& kv : sorted_params) {
                os << "  ext_param " << kv.first << "=" << Num(kv.second, opt.coord_decimals) << "\n";
            }
            std::map<std::string, std::string> sorted_strings(feat.ext_strings.begin(),
                                                              feat.ext_strings.end());
            for (const auto& kv : sorted_strings) {
                os << "  ext_string " << kv.first << "=" << kv.second << "\n";
            }
        }
    }

    return os.str();
}

} // namespace cadcvt_golden
