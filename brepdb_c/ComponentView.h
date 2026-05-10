#pragma once

#include "GeomPool.h"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <vector>

namespace brepdb
{

// SoA views over a GeometryPool.
// These do NOT own data — they reference a live GeometryPool.
// Rebuild after pool mutation.

struct AabbView
{
    std::vector<uint32_t> entity_ids;
    std::vector<double>   min_x, min_y, min_z;
    std::vector<double>   max_x, max_y, max_z;

    void Build(const GeometryPool& pool)
    {
        const size_t n = pool.headers.size();
        entity_ids.resize(n);
        min_x.resize(n); min_y.resize(n); min_z.resize(n);
        max_x.resize(n); max_y.resize(n); max_z.resize(n);

        for (size_t i = 0; i < n; ++i)
        {
            const auto& h = pool.headers[i];
            entity_ids[i] = h.persistent_id;
            min_x[i] = h.min_pt[0];
            min_y[i] = h.min_pt[1];
            min_z[i] = h.min_pt[2];
            max_x[i] = h.max_pt[0];
            max_y[i] = h.max_pt[1];
            max_z[i] = h.max_pt[2];
        }
    }

    // Query: find all entities whose AABB intersects the given box.
    std::vector<uint32_t> QueryBox(double qmin[3], double qmax[3]) const
    {
        std::vector<uint32_t> result;
        const size_t n = entity_ids.size();
        for (size_t i = 0; i < n; ++i)
        {
            if (max_x[i] < qmin[0] || min_x[i] > qmax[0]) continue;
            if (max_y[i] < qmin[1] || min_y[i] > qmax[1]) continue;
            if (max_z[i] < qmin[2] || min_z[i] > qmax[2]) continue;
            result.push_back(entity_ids[i]);
        }
        return result;
    }

    size_t Size() const { return entity_ids.size(); }
};

struct TypeView
{
    std::vector<uint32_t> entity_ids;
    std::vector<Type>     types;

    void Build(const GeometryPool& pool)
    {
        const size_t n = pool.headers.size();
        entity_ids.resize(n);
        types.resize(n);
        for (size_t i = 0; i < n; ++i)
        {
            entity_ids[i] = pool.headers[i].persistent_id;
            types[i]      = pool.headers[i].type;
        }
    }

    // Get all entity ids of a given type.
    std::vector<uint32_t> GetByType(Type t) const
    {
        std::vector<uint32_t> result;
        for (size_t i = 0; i < types.size(); ++i)
        {
            if (types[i] == t)
                result.push_back(entity_ids[i]);
        }
        return result;
    }

    size_t Size() const { return entity_ids.size(); }
};

struct ParamView
{
    std::vector<uint32_t> entity_ids;
    std::vector<uint32_t> offsets;
    std::vector<uint32_t> counts;
    const double*         data = nullptr;
    size_t                data_size = 0;

    void Build(const GeometryPool& pool)
    {
        const size_t n = pool.headers.size();
        entity_ids.resize(n);
        offsets.resize(n);
        counts.resize(n);
        for (size_t i = 0; i < n; ++i)
        {
            entity_ids[i] = pool.headers[i].persistent_id;
            offsets[i]    = pool.headers[i].param_offset;
            counts[i]     = pool.headers[i].param_count;
        }
        data      = pool.data_pool.data();
        data_size = pool.data_pool.size();
    }

    const double* GetParams(size_t index, uint32_t& out_count) const
    {
        assert(index < entity_ids.size());
        out_count = counts[index];
        return data + offsets[index];
    }

    // Find index by entity id (linear scan; for frequent lookups build a map).
    int32_t FindIndex(uint32_t pid) const
    {
        for (size_t i = 0; i < entity_ids.size(); ++i)
        {
            if (entity_ids[i] == pid)
                return static_cast<int32_t>(i);
        }
        return -1;
    }

    size_t Size() const { return entity_ids.size(); }
};

// Composite: holds all views for a pool, rebuilt together.
struct PoolViews
{
    AabbView  aabbs;
    TypeView  types;
    ParamView params;

    void Build(const GeometryPool& pool)
    {
        aabbs.Build(pool);
        types.Build(pool);
        params.Build(pool);
    }
};

} // namespace brepdb
