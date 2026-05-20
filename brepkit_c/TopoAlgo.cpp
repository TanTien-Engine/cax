#include "TopoAlgo.h"
#include "TopoShape.h"
#include "occt_adapter.h"

#include "ShapeBuilder.h"
#include "breptopo_c/TopoNaming.h"
#include "brepdb_c/WorldSender.h"
#include "brepdb_c/VersionTree.h"

// OCCT
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepOffsetAPI_MakeDraft.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>

namespace
{

// Single-body op: commit as child of input shape's version node.
// Stores the new version_id back into dst.
void commit_to_vt(const std::shared_ptr<breptopo::TopoNaming>& tn,
                  const std::shared_ptr<brepdb::VersionTree>& vt,
                  const std::shared_ptr<brepkit::TopoShape>& src,
                  const std::shared_ptr<brepkit::TopoShape>& dst,
                  const breptopo::TopoNaming::PidMap& pid_map,
                  const std::string& op_name)
{
    if (!tn || !vt) {
        return;
    }
    brepdb::WorldSender sender(tn);
    brepdb::BRepWorld world;
    sender.Serialize(dst->GetShape(), world);

    uint32_t parent_id = src->GetVersionId();
    uint32_t new_id;
    if (parent_id == brepkit::TopoShape::NO_VERSION) {
        uint32_t root_id = vt->AddRoot(world, op_name);
        new_id = root_id;
    } else {
        uint32_t root_id = vt->FindRootOf(parent_id);
        vt->Checkout(root_id, parent_id);
        new_id = vt->Commit(root_id, world, pid_map, op_name);
    }
    dst->SetVersionId(new_id);
}

// Boolean-op variant: creates a merge node.
// primary_parent comes from the main body's version_id,
// the tool body's version_id becomes an auxiliary parent.
//
// tool_world must be serialized BEFORE tn->Update(), because Update()
// unbinds the old shapes (including the tool body) from the HistGraph.
void merge_to_vt(const std::shared_ptr<breptopo::TopoNaming>& tn,
                 const std::shared_ptr<brepdb::VersionTree>& vt,
                 const std::shared_ptr<brepkit::TopoShape>& main_src,
                 const std::shared_ptr<brepkit::TopoShape>& tool_src,
                 const std::shared_ptr<brepkit::TopoShape>& dst,
                 brepdb::BRepWorld&& tool_world,
                 const breptopo::TopoNaming::PidMap& pid_map,
                 const std::string& op_name)
{
    if (!tn || !vt) {
        return;
    }

    uint32_t tool_vid = tool_src->GetVersionId();
    if (tool_vid == brepkit::TopoShape::NO_VERSION) {
        tool_vid = vt->AddRoot(tool_world, op_name + "_tool");
    }

    brepdb::WorldSender sender(tn);
    brepdb::BRepWorld world;
    sender.Serialize(dst->GetShape(), world);

    uint32_t primary_id = main_src->GetVersionId();
    uint32_t new_id = vt->Merge(primary_id, { tool_vid },
                                world, pid_map, op_name);
    dst->SetVersionId(new_id);
}

brepdb::BRepWorld serialize_world(const std::shared_ptr<breptopo::TopoNaming>& tn,
                                  const TopoDS_Shape& shape)
{
    brepdb::WorldSender sender(tn);
    brepdb::BRepWorld world;
    sender.Serialize(shape, world);
    return world;
}

}

namespace brepkit
{

std::shared_ptr<TopoShape> TopoAlgo::Fillet(const std::shared_ptr<TopoShape>& shape, double radius,
                                            const std::vector<std::shared_ptr<TopoShape>>& edges, uint32_t op_id,
                                            const std::shared_ptr<breptopo::TopoNaming>& tn,
                                            const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepFilletAPI_MakeFillet fillet(shape->GetShape());

    if (edges.empty())
    {
        for (TopExp_Explorer ex(shape->GetShape(), TopAbs_EDGE); ex.More(); ex.Next())
            fillet.Add(radius, TopoDS::Edge(ex.Current()));
    }
    else
    {
        for (auto& edge : edges) {
            if (edge->GetShape().ShapeType() == TopAbs_EDGE) {
                fillet.Add(radius, TopoDS::Edge(edge->GetShape()));
            } else {
                for (TopExp_Explorer ex(edge->GetShape(), TopAbs_EDGE); ex.More(); ex.Next())
                    fillet.Add(radius, TopoDS::Edge(ex.Current()));
            }
        }
    }

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(fillet, fillet.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(fillet.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "fillet");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Chamfer(const std::shared_ptr<TopoShape>& shape, double dist,
                                             const std::vector<std::shared_ptr<TopoShape>>& edges, uint32_t op_id,
                                             const std::shared_ptr<breptopo::TopoNaming>& tn,
                                             const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepFilletAPI_MakeChamfer chamfer(shape->GetShape());

    if (edges.empty())
    {
        for (TopExp_Explorer ex(shape->GetShape(), TopAbs_EDGE); ex.More(); ex.Next())
            chamfer.Add(dist, TopoDS::Edge(ex.Current()));
    }
    else
    {
        for (auto& edge : edges) {
            if (edge->GetShape().ShapeType() == TopAbs_EDGE) {
                chamfer.Add(dist, TopoDS::Edge(edge->GetShape()));
            } else {
                for (TopExp_Explorer ex(edge->GetShape(), TopAbs_EDGE); ex.More(); ex.Next())
                    chamfer.Add(dist, TopoDS::Edge(ex.Current()));
            }
        }
    }

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(chamfer, chamfer.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(chamfer.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "chamfer");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Prism(const std::shared_ptr<TopoShape>& face, double x, double y, double z,
                                           uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                           const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepPrimAPI_MakePrism prism(face->GetShape(), gp_Vec(x, y, z));

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(prism, prism.Shape(), face->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(prism.Shape());
    commit_to_vt(tn, vt, face, dst, pid_map, "prism");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Split(const std::shared_ptr<TopoShape>& base, const std::shared_ptr<TopoShape>& tool,
                                           uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                           const std::shared_ptr<brepdb::VersionTree>& vt)
{
    TopTools_ListOfShape bases, tools;
    bases.Append(base->GetShape());
    tools.Append(tool->GetShape());

    BRepAlgoAPI_Splitter algo;
    algo.SetArguments(bases);
    algo.SetTools(tools);
    algo.Build();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cout);
    }
    if (algo.HasWarnings()) {
        algo.DumpErrors(std::cout);
    }

    // Serialize tool BEFORE tn->Update() unbinds its shapes
    brepdb::BRepWorld tool_world;
    if (tn && vt) { tool_world = serialize_world(tn, tool->GetShape()); }

    breptopo::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = ShapeBuilder::MakeCompound({ base, tool });
        opencascade::handle<BRepTools_History> o_hist = algo.History();
        pid_map = tn->Update(o_hist, algo.Shape(), old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(algo.Shape());
    merge_to_vt(tn, vt, base, tool, dst, std::move(tool_world), pid_map, "split");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Cut(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
                                         uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                         const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepAlgoAPI_Cut algo(s1->GetShape(), s2->GetShape());
    algo.Build();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cout);
    }
    if (algo.HasWarnings()) {
        algo.DumpErrors(std::cout);
    }

    brepdb::BRepWorld tool_world;
    if (tn && vt) { tool_world = serialize_world(tn, s2->GetShape()); }

    breptopo::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = ShapeBuilder::MakeCompound({ s1, s2 });
        opencascade::handle<BRepTools_History> o_hist = algo.History();
        pid_map = tn->Update(o_hist, algo.Shape(), old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(algo.Shape());
    merge_to_vt(tn, vt, s1, s2, dst, std::move(tool_world), pid_map, "cut");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Fuse(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
                                          uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                          const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepAlgoAPI_Fuse algo(s1->GetShape(), s2->GetShape());
    algo.Build();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cout);
    }
    if (algo.HasWarnings()) {
        algo.DumpErrors(std::cout);
    }

    brepdb::BRepWorld tool_world;
    if (tn && vt) { tool_world = serialize_world(tn, s2->GetShape()); }

    breptopo::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = ShapeBuilder::MakeCompound({ s1, s2 });
        opencascade::handle<BRepTools_History> o_hist = algo.History();
        pid_map = tn->Update(o_hist, algo.Shape(), old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(algo.Shape());
    merge_to_vt(tn, vt, s1, s2, dst, std::move(tool_world), pid_map, "fuse");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Common(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
                                            uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                            const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepAlgoAPI_Common algo(s1->GetShape(), s2->GetShape());
    //algo.Build();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cout);
    }
    if (algo.HasWarnings()) {
        algo.DumpErrors(std::cout);
    }

    brepdb::BRepWorld tool_world;
    if (tn && vt) { tool_world = serialize_world(tn, s2->GetShape()); }

    breptopo::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = ShapeBuilder::MakeCompound({ s1, s2 });
        opencascade::handle<BRepTools_History> o_hist = algo.History();
        pid_map = tn->Update(o_hist, algo.Shape(), old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(algo.Shape());
    merge_to_vt(tn, vt, s1, s2, dst, std::move(tool_world), pid_map, "common");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Section(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
                                             uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                             const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepAlgoAPI_Section algo;
    algo.Init1(s1->GetShape());
    algo.Init2(s2->GetShape());
    algo.Approximation(Standard_False);
    algo.Build();

    if (!algo.IsDone()) {
        algo.DumpErrors(std::cout);
    }
    if (algo.HasWarnings()) {
        algo.DumpErrors(std::cout);
    }

    brepdb::BRepWorld tool_world;
    if (tn && vt) { tool_world = serialize_world(tn, s2->GetShape()); }

    breptopo::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = ShapeBuilder::MakeCompound({ s1, s2 });
        opencascade::handle<BRepTools_History> o_hist = algo.History();
        pid_map = tn->Update(o_hist, algo.Shape(), old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(algo.Shape());
    merge_to_vt(tn, vt, s1, s2, dst, std::move(tool_world), pid_map, "section");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Sew(const std::shared_ptr<TopoShape>& s1, const std::shared_ptr<TopoShape>& s2,
    uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
    const std::shared_ptr<brepdb::VersionTree>& vt)
{
    Standard_Real tolerance = 1e-6;
    BRepBuilderAPI_Sewing algo(tolerance);

    algo.Add(s1->GetShape());
    algo.Add(s2->GetShape());

    algo.Perform();

    //if (tn)
    //{
    //    auto old_shp = ShapeBuilder::MakeCompound({ s1, s2 });
    //    opencascade::handle<BRepTools_History> o_hist = algo.History();
    //    auto upd_hist_graph = [&](const std::shared_ptr<breptopo::HistGraph>& hg)
    //    {
    //        if (!hg) {
    //            return;
    //        }
    //        auto type = trans_type(hg->GetType());
    //        ShapeHistory hist(o_hist, type, algo.Shape(), s1->GetShape());
    //        hg->Update(hist, op_id);
    //    };
    //    upd_hist_graph(tn->GetEdgeGraph());
    //    upd_hist_graph(tn->GetFaceGraph());
    //    upd_hist_graph(tn->GetSolidGraph());
    //}

    auto shp = algo.SewedShape();
    auto type = shp.ShapeType();

    return std::make_shared<brepkit::TopoShape>(algo.SewedShape());
}

std::shared_ptr<TopoShape> TopoAlgo::UnifySameDomain(const std::shared_ptr<TopoShape>& shape,
                                                     uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                                     const std::shared_ptr<brepdb::VersionTree>& vt)
{
    ShapeUpgrade_UnifySameDomain algo;
    algo.Initialize(shape->GetShape(), Standard_True, Standard_True, Standard_False);
    algo.Build();

    breptopo::TopoNaming::PidMap pid_map;
    if (tn)
    {
        opencascade::handle<BRepTools_History> o_hist = algo.History();
        pid_map = tn->Update(o_hist, algo.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(algo.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "unify_same_domain");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Translate(const std::shared_ptr<TopoShape>& shape, double x, double y, double z,
                                               uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                               const std::shared_ptr<brepdb::VersionTree>& vt)
{
    gp_Trsf trsf;
    trsf.SetTranslation(gp_Vec(x, y, z));
    auto trans = BRepBuilderAPI_Transform(shape->GetShape(), trsf, Standard_True);

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(trans, trans.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(trans.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "translate");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Rotate(const std::shared_ptr<TopoShape>& shape,
                                            const sm::vec3& pos, const sm::vec3& dir, double angle,
                                            uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                            const std::shared_ptr<brepdb::VersionTree>& vt)
{
    gp_Ax1 axis(trans_pnt(pos), trans_dir(dir));

    gp_Trsf trsf;
    trsf.SetRotation(axis, angle);
    auto trans = BRepBuilderAPI_Transform(shape->GetShape(), trsf, Standard_True);

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(trans, trans.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(trans.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "rotate");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Scale(const std::shared_ptr<TopoShape>& shape,
                                           const sm::vec3& center, double factor,
                                           uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                           const std::shared_ptr<brepdb::VersionTree>& vt)
{
    gp_Trsf trsf;
    trsf.SetScale(trans_pnt(center), factor);
    auto trans = BRepBuilderAPI_Transform(shape->GetShape(), trsf, Standard_True);

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(trans, trans.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(trans.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "scale");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Transform(const std::shared_ptr<TopoShape>& shape,
                                                const double* mat4x4,
                                                uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                                const std::shared_ptr<brepdb::VersionTree>& vt)
{
    gp_Trsf trsf;
    // mat4x4 is row-major 4x4, OCCT SetValues takes row,col (1-based)
    trsf.SetValues(
        mat4x4[0],  mat4x4[1],  mat4x4[2],  mat4x4[3],
        mat4x4[4],  mat4x4[5],  mat4x4[6],  mat4x4[7],
        mat4x4[8],  mat4x4[9],  mat4x4[10], mat4x4[11]);

    auto trans = BRepBuilderAPI_Transform(shape->GetShape(), trsf, Standard_True);

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(trans, trans.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(trans.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "transform");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Mirror(const std::shared_ptr<TopoShape>& shape, const sm::vec3& pos, const sm::vec3& dir,
                                            uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                            const std::shared_ptr<brepdb::VersionTree>& vt)
{
    gp_Ax1 axis;
    axis.SetLocation(trans_pnt(pos));
    axis.SetDirection(trans_vec(dir));

    gp_Trsf trsf;
    trsf.SetMirror(axis);

    auto trans = BRepBuilderAPI_Transform(shape->GetShape(), trsf, Standard_True);

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(trans, trans.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(trans.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "mirror");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::Draft(const std::shared_ptr<TopoShape>& shape,
                                           const sm::vec3& dir, float angle, float len_max,
                                           uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                           const std::shared_ptr<brepdb::VersionTree>& vt)
{
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape->GetShape(), TopAbs_FACE, faces);

    const auto& face = TopoDS::Face(faces.FindKey(1));

    BRepOffsetAPI_MakeDraft draft(face, trans_vec(dir), angle);
    draft.Perform(len_max);

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(draft, draft.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(draft.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "draft");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::ThickSolid(const std::shared_ptr<TopoShape>& shape, const std::vector<std::shared_ptr<TopoShape>>& faces, float offset,
                                                uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                                const std::shared_ptr<brepdb::VersionTree>& vt)
{
    TopTools_ListOfShape faces_to_rm;
    for (auto& face : faces) {
        faces_to_rm.Append(face->GetShape());
    }

    BRepOffsetAPI_MakeThickSolid thick_solid;
    thick_solid.MakeThickSolidByJoin(shape->GetShape(), faces_to_rm, offset, 1.e-3);

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(thick_solid, thick_solid.Shape(), shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(thick_solid.Shape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "thick_solid");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::ThruSections(const std::vector<std::shared_ptr<TopoShape>>& wires,
                                                  uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                                  const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepOffsetAPI_ThruSections thru_sections(Standard_False);
    for (auto& wire : wires) {
        thru_sections.AddWire(wire->ToWire());
    }

    breptopo::TopoNaming::PidMap pid_map;
    if (tn)
    {
        auto old_shp = ShapeBuilder::MakeCompound(wires);
        pid_map = tn->Update(thru_sections, thru_sections.Shape(), old_shp->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(thru_sections.Shape());
    auto dummy_src = std::make_shared<brepkit::TopoShape>();
    commit_to_vt(tn, vt, dummy_src, dst, pid_map, "thru_sections");
    return dst;
}

std::shared_ptr<TopoShape> TopoAlgo::OffsetShape(const std::shared_ptr<TopoShape>& shape, float offset, bool is_solid,
                                                 uint32_t op_id, const std::shared_ptr<breptopo::TopoNaming>& tn,
                                                 const std::shared_ptr<brepdb::VersionTree>& vt)
{
    BRepOffset_MakeSimpleOffset builder;
    builder.Initialize(shape->GetShape(), offset);
    builder.SetBuildSolidFlag(is_solid);
    builder.Perform();

    breptopo::TopoNaming::PidMap pid_map;
    if (tn) {
        pid_map = tn->Update(builder, shape->GetShape(), op_id);
    }
    auto dst = std::make_shared<brepkit::TopoShape>(builder.GetResultShape());
    commit_to_vt(tn, vt, shape, dst, pid_map, "offset_shape");
    return dst;
}

}
