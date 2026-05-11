#include <catch2/catch_test_macros.hpp>

#include "ShapeCache.h"

#include <string>

using namespace breptopo;

// Use int as a simple Val stand-in for LruCache tests.
using IntCache = LruCache<int>;

TEST_CASE("LruCache basic put/get", "[shape_cache]")
{
    IntCache cache(4);

    cache.Put(1, 10);
    cache.Put(2, 20);
    cache.Put(3, 30);

    REQUIRE(cache.Size() == 3);

    auto* v1 = cache.Get(1);
    REQUIRE(v1 != nullptr);
    REQUIRE(*v1 == 10);

    auto* v2 = cache.Get(2);
    REQUIRE(v2 != nullptr);
    REQUIRE(*v2 == 20);

    REQUIRE(cache.Get(99) == nullptr);
}

TEST_CASE("LruCache eviction", "[shape_cache]")
{
    IntCache cache(3);

    cache.Put(1, 10);
    cache.Put(2, 20);
    cache.Put(3, 30);
    REQUIRE(cache.Size() == 3);

    // inserting a 4th entry evicts the LRU (key=1)
    cache.Put(4, 40);
    REQUIRE(cache.Size() == 3);
    REQUIRE(cache.Get(1) == nullptr);
    REQUIRE(*cache.Get(2) == 20);
    REQUIRE(*cache.Get(4) == 40);
}

TEST_CASE("LruCache access refreshes entry", "[shape_cache]")
{
    IntCache cache(3);

    cache.Put(1, 10);
    cache.Put(2, 20);
    cache.Put(3, 30);

    // access key=1, making key=2 the new LRU
    cache.Get(1);

    cache.Put(4, 40);
    REQUIRE(cache.Get(2) == nullptr);
    REQUIRE(*cache.Get(1) == 10);
    REQUIRE(*cache.Get(3) == 30);
    REQUIRE(*cache.Get(4) == 40);
}

TEST_CASE("LruCache update existing key", "[shape_cache]")
{
    IntCache cache(4);

    cache.Put(1, 10);
    cache.Put(1, 99);
    REQUIRE(cache.Size() == 1);
    REQUIRE(*cache.Get(1) == 99);
}

TEST_CASE("LruCache remove", "[shape_cache]")
{
    IntCache cache(4);

    cache.Put(1, 10);
    cache.Put(2, 20);
    cache.Remove(1);

    REQUIRE(cache.Size() == 1);
    REQUIRE(cache.Get(1) == nullptr);
    REQUIRE(*cache.Get(2) == 20);

    // removing non-existent key is a no-op
    cache.Remove(99);
    REQUIRE(cache.Size() == 1);
}

TEST_CASE("LruCache clear", "[shape_cache]")
{
    IntCache cache(4);

    cache.Put(1, 10);
    cache.Put(2, 20);
    cache.Clear();

    REQUIRE(cache.Size() == 0);
    REQUIRE(cache.Get(1) == nullptr);
}

TEST_CASE("LruCache SetCapacity shrinks", "[shape_cache]")
{
    IntCache cache(8);

    for (uint32_t i = 1; i <= 6; ++i)
        cache.Put(i, static_cast<int>(i * 10));

    REQUIRE(cache.Size() == 6);

    cache.SetCapacity(3);
    REQUIRE(cache.Size() == 3);

    // oldest entries (1,2,3) should be evicted
    REQUIRE(cache.Get(1) == nullptr);
    REQUIRE(cache.Get(2) == nullptr);
    REQUIRE(cache.Get(3) == nullptr);
    REQUIRE(*cache.Get(4) == 40);
    REQUIRE(*cache.Get(5) == 50);
    REQUIRE(*cache.Get(6) == 60);
}

TEST_CASE("LruCache capacity 1", "[shape_cache]")
{
    IntCache cache(1);

    cache.Put(1, 10);
    REQUIRE(*cache.Get(1) == 10);

    cache.Put(2, 20);
    REQUIRE(cache.Get(1) == nullptr);
    REQUIRE(*cache.Get(2) == 20);
    REQUIRE(cache.Size() == 1);
}
