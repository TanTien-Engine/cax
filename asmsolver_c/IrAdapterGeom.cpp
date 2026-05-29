#include "asmsolver_c/IrAdapterGeom.h"

#include "cadapp_c/ir/Enums.h"
#include "cadapp_c/ir/FeatureIR.h"
#include "brepkit_c/TopoShape.h"

#include <BRepAdaptor_Surface.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>

namespace asmsolver {
namespace {

std::string getstr(const cadapp::FeatureIR& f, const std::string& k)
{
    auto it = f.ext_strings.find(k);
    return it != f.ext_strings.end() ? it->second : std::string{};
}

// "Clone002.Face1" -> feature "Clone002", face index 1.
bool parseElem(const std::string& elem, std::string& feat, int& index)
{
    if (elem.empty()) return false;
    size_t first = elem.find('.');
    if (first == std::string::npos) return false;
    feat = elem.substr(0, first);
    size_t last = elem.rfind('.');
    std::string token = elem.substr(last + 1);            // e.g. "Face1"
    size_t i = 0;
    while (i < token.size() && !std::isdigit((unsigned char)token[i])) ++i;
    if (i >= token.size()) return false;
    index = std::atoi(token.c_str() + i);
    return index >= 1;
}

// Radius of feature.FaceN if it is a cylindrical face, else 0.
double faceCylRadius(const std::unordered_map<std::string, uint32_t>& name_to_id,
                     const cadapp::DocumentIR& doc,
                     const std::string& feat_name, int face_index)
{
    auto idit = name_to_id.find(feat_name);
    if (idit == name_to_id.end()) return 0.0;
    auto shit = doc.authored_shapes.find(idit->second);
    if (shit == doc.authored_shapes.end() || !shit->second) return 0.0;
    const TopoDS_Shape& s = shit->second->GetShape();
    if (s.IsNull()) return 0.0;
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(s, TopAbs_FACE, faces);
    if (std::getenv("ASM_DEBUG_FACES")) {
        std::printf("[geom] %s: %d faces\n", feat_name.c_str(), faces.Extent());
        for (int i = 1; i <= faces.Extent(); ++i) {
            BRepAdaptor_Surface su(TopoDS::Face(faces.FindKey(i)));
            if (su.GetType() == GeomAbs_Cylinder)
                std::printf("   Face%d cylinder r=%.6f\n", i, su.Cylinder().Radius());
            else
                std::printf("   Face%d type=%d\n", i, (int)su.GetType());
        }
    }
    if (face_index < 1 || face_index > faces.Extent()) return 0.0;
    TopoDS_Face f = TopoDS::Face(faces.FindKey(face_index));
    BRepAdaptor_Surface surf(f);
    if (surf.GetType() != GeomAbs_Cylinder) return 0.0;
    return surf.Cylinder().Radius();
}

} // namespace

int ResolveCylinderRadii(ImportResult& R, const cadapp::DocumentIR& doc)
{
    // name -> id, and name -> joint feature (for elem refs)
    std::unordered_map<std::string, uint32_t> name_to_id;
    std::unordered_map<std::string, const cadapp::FeatureIR*> joint_by_name;
    for (const auto& f : doc.features) {
        name_to_id[f.name] = f.id;
        if (f.type == cadapp::FeatType::Joint) joint_by_name[f.name] = &f;
    }

    int resolved = 0;
    for (auto& j : R.assembly.joints) {
        if (j.kind != JointKind::Distance || j.grounded) continue;
        auto jit = joint_by_name.find(j.name);
        if (jit == joint_by_name.end()) continue;
        const cadapp::FeatureIR& jf = *jit->second;

        // Convention: residual treats body_a as the plane and body_b as
        // the cylinder, so resolve the radius from ref2 (body_b). (ref1
        // as the cylinder is a flipped case left for a later pass.)
        std::string feat; int idx = 0;
        if (!parseElem(getstr(jf, "joint_ref2_elem"), feat, idx)) continue;
        double r = faceCylRadius(name_to_id, doc, feat, idx);
        if (r > 0.0) { j.radius = r; ++resolved; }
    }
    return resolved;
}

} // namespace asmsolver
