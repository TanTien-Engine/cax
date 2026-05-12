#include <catch2/catch_test_macros.hpp>

#include "brepdb_c/ShapeIndex.h"

#include <spatialdb/RTree.h>
#include <spatialdb/MemoryStorageManager.h>
#include <spatialdb/Region.h>

using namespace brepdb;

static spatialdb::Region MakeBox(double x, double y, double z, double size)
{
    double lo[3] = {x, y, z};
    double hi[3] = {x + size, y + size, z + size};
    return spatialdb::Region(lo, hi);
}

struct IndexFixture
{
    std::shared_ptr<spatialdb::IStorageManager> sm;
    std::unique_ptr<spatialdb::RTree> tree;
    std::unique_ptr<ShapeIndex> idx;

    IndexFixture()
    {
        sm = std::make_shared<spatialdb::MemoryStorageManager>();
        tree = std::make_unique<spatialdb::RTree>(sm, true);
        idx = std::make_unique<ShapeIndex>(*tree, sm);
    }

    void Insert(spatialdb::id_type id, double x, double y, double z)
    {
        auto box = MakeBox(x, y, z, 1.0);
        uint8_t dummy = 0;
        tree->InsertData(1, &dummy, box, id);
    }
};

// ---------------------------------------------------------------
//  Basic operations
// ---------------------------------------------------------------

TEST_CASE("ShapeIndex: empty index", "[shape_index]")
{
    IndexFixture f;
    CHECK(f.idx->Size() == 0);
    CHECK_FALSE(f.idx->Contains(1));

    ShapeSlot slot;
    CHECK_FALSE(f.idx->Lookup(1, slot));
}

TEST_CASE("ShapeIndex: insert updates index automatically", "[shape_index]")
{
    IndexFixture f;

    f.Insert(100, 0, 0, 0);
    CHECK(f.idx->Contains(100));
    CHECK(f.idx->Size() == 1);

    ShapeSlot slot;
    REQUIRE(f.idx->Lookup(100, slot));
}

TEST_CASE("ShapeIndex: multiple inserts", "[shape_index]")
{
    IndexFixture f;

    f.Insert(1, 0, 0, 0);
    f.Insert(2, 10, 0, 0);
    f.Insert(3, 20, 0, 0);

    CHECK(f.idx->Size() == 3);
    CHECK(f.idx->Contains(1));
    CHECK(f.idx->Contains(2));
    CHECK(f.idx->Contains(3));
    CHECK_FALSE(f.idx->Contains(4));
}

TEST_CASE("ShapeIndex: insert many triggers splits and index stays consistent", "[shape_index]")
{
    IndexFixture f;

    for (int i = 0; i < 50; ++i)
        f.Insert(i + 1, i * 2.0, 0, 0);

    CHECK(f.idx->Size() == 50);
    for (int i = 0; i < 50; ++i)
        CHECK(f.idx->Contains(i + 1));
}

// ---------------------------------------------------------------
//  Store / Load round-trip
// ---------------------------------------------------------------

TEST_CASE("ShapeIndex: Store and Load round-trip", "[shape_index]")
{
    auto sm = std::make_shared<spatialdb::MemoryStorageManager>();

    spatialdb::id_type page = -1;

    // Phase 1: build an index and store it
    {
        auto tree = std::make_unique<spatialdb::RTree>(sm, true);
        ShapeIndex idx(*tree, sm);

        auto box1 = MakeBox(0, 0, 0, 1.0);
        auto box2 = MakeBox(5, 5, 5, 1.0);
        auto box3 = MakeBox(10, 10, 10, 1.0);
        uint8_t d = 0;
        tree->InsertData(1, &d, box1, 10);
        tree->InsertData(1, &d, box2, 20);
        tree->InsertData(1, &d, box3, 30);

        CHECK(idx.Size() == 3);
        idx.Store(page);
        REQUIRE(page >= 0);
    }

    // Phase 2: create a fresh tree+index on the same storage, load the index
    {
        auto tree = std::make_unique<spatialdb::RTree>(sm, false);
        ShapeIndex idx(*tree, sm);

        idx.Load(page);

        CHECK(idx.Size() == 3);
        CHECK(idx.Contains(10));
        CHECK(idx.Contains(20));
        CHECK(idx.Contains(30));

        ShapeSlot slot;
        REQUIRE(idx.Lookup(20, slot));
    }
}

TEST_CASE("ShapeIndex: Store empty index", "[shape_index]")
{
    IndexFixture f;

    spatialdb::id_type page = -1;
    f.idx->Store(page);
    REQUIRE(page >= 0);

    ShapeIndex idx2(*f.tree, f.sm);
    idx2.Load(page);
    CHECK(idx2.Size() == 0);
}

// ---------------------------------------------------------------
//  Rebuild
// ---------------------------------------------------------------

TEST_CASE("ShapeIndex: Rebuild reconstructs index from R-tree", "[shape_index]")
{
    auto sm = std::make_shared<spatialdb::MemoryStorageManager>();
    auto tree = std::make_unique<spatialdb::RTree>(sm, true);

    auto box1 = MakeBox(0, 0, 0, 1.0);
    auto box2 = MakeBox(10, 10, 10, 1.0);
    uint8_t d1 = 1, d2 = 2;
    tree->InsertData(1, &d1, box1, 100);
    tree->InsertData(1, &d2, box2, 200);

    ShapeIndex idx(*tree, sm);
    idx.Rebuild();

    CHECK(idx.Size() == 2);
    CHECK(idx.Contains(100));
    CHECK(idx.Contains(200));
}

TEST_CASE("ShapeIndex: GetData returns false for unknown id", "[shape_index]")
{
    IndexFixture f;

    uint32_t len = 0;
    uint8_t* data = nullptr;
    CHECK_FALSE(f.idx->GetData(999, len, &data));
}
