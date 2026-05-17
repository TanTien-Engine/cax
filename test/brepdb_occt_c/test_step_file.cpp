#include <catch2/catch_test_macros.hpp>

#include "brepdb_c/StepFile.h"
#include "brepdb_c/TypedPool.h"
#include "brepdb_c/WorldSender.h"
#include "brepdb_c/WorldReceiver.h"
#include "breptopo_c/TopoNaming.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <TopExp_Explorer.hxx>

#include <cstdio>
#include <filesystem>

using namespace brepdb;

namespace
{

std::string TmpPath(const std::string& name)
{
    auto p = std::filesystem::temp_directory_path() / name;
    return p.string();
}

// Build a BRepWorld containing a simple box
BRepWorld make_box_world(const std::shared_ptr<breptopo::TopoNaming>& tn)
{
    BRepWorld world;
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();

    WorldSender sender(tn);
    sender.Serialize(box, world);
    return world;
}

} // anonymous

TEST_CASE("StepFile export produces a file", "[step]")
{
    auto tn = std::make_shared<breptopo::TopoNaming>();
    auto world = make_box_world(tn);
    const auto filepath = TmpPath("test_export.step");

    bool ok = StepFile::Export(filepath, world);
    CHECK(ok);
    CHECK(std::filesystem::exists(filepath));
    CHECK(std::filesystem::file_size(filepath) > 0);

    std::remove(filepath.c_str());
}

TEST_CASE("StepFile import reads a STEP file", "[step]")
{
    auto tn = std::make_shared<breptopo::TopoNaming>();
    auto original = make_box_world(tn);
    const auto filepath = TmpPath("test_import.step");

    REQUIRE(StepFile::Export(filepath, original));

    BRepWorld imported;
    auto tn2 = std::make_shared<breptopo::TopoNaming>();
    bool ok = StepFile::Import(filepath, imported, tn2);
    CHECK(ok);
    CHECK(imported.EntityCount() > 0);

    std::remove(filepath.c_str());
}

TEST_CASE("StepFile roundtrip preserves topology", "[step]")
{
    auto tn = std::make_shared<breptopo::TopoNaming>();
    auto original = make_box_world(tn);
    const auto filepath = TmpPath("test_roundtrip.step");

    REQUIRE(StepFile::Export(filepath, original));

    BRepWorld imported;
    auto tn2 = std::make_shared<breptopo::TopoNaming>();
    REQUIRE(StepFile::Import(filepath, imported, tn2));

    // A box should have: 8 vertices, 12 edges, 6 faces, 1 solid
    int vertex_count = 0, edge_count = 0, face_count = 0, solid_count = 0;
    for (uint32_t id : imported.AliveEntities()) {
        const Type* t = imported.Types().Get(id);
        if (!t) continue;
        switch (*t) {
        case Type::Vertex: ++vertex_count; break;
        case Type::Edge:   ++edge_count;   break;
        case Type::Face:   ++face_count;   break;
        case Type::Solid:  ++solid_count;  break;
        default: break;
        }
    }

    CHECK(solid_count == 1);
    CHECK(face_count == 6);
    CHECK(edge_count == 12);
    CHECK(vertex_count == 8);

    // Reconstruct and verify geometry is valid
    WorldReceiver receiver(imported);
    TopoDS_Shape shape = receiver.GetAll();
    CHECK_FALSE(shape.IsNull());

    int rebuilt_solids = 0;
    for (TopExp_Explorer exp(shape, TopAbs_SOLID); exp.More(); exp.Next())
        ++rebuilt_solids;
    CHECK(rebuilt_solids == 1);

    std::remove(filepath.c_str());
}

TEST_CASE("StepFile import nonexistent file returns false", "[step]")
{
    BRepWorld world;
    auto tn = std::make_shared<breptopo::TopoNaming>();
    CHECK_FALSE(StepFile::Import(TmpPath("nonexistent_99999.step"), world, tn));
}

TEST_CASE("StepFile export empty world returns false", "[step]")
{
    BRepWorld world;
    const auto filepath = TmpPath("test_empty.step");
    CHECK_FALSE(StepFile::Export(filepath, world));
    std::remove(filepath.c_str());
}
