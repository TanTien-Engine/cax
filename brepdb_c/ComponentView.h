#pragma once

#include "TypedPool.h"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <vector>

namespace brepdb
{

// SoA views over a BRepWorld.
// These do NOT own data -- they snapshot a live BRepWorld.
// Rebuild after world mutation.

struct AabbView
{
    std::vector<uint32_t> entity_ids;
    std::vector<double>   min_x, min_y, min_z;
    std::vector<double>   max_x, max_y, max_z;

    void Build(const BRepWorld& world)
    {
        const auto& alive = world.AliveEntities();
        const size_t n = alive.size();
        entity_ids.resize(n);
        min_x.resize(n); min_y.resize(n); min_z.resize(n);
        max_x.resize(n); max_y.resize(n); max_z.resize(n);

        for (size_t i = 0; i < n; ++i)
        {
            uint32_t id = alive[i];
            entity_ids[i] = id;
            const AabbComp* a = world.Aabbs().Get(id);
            if (a) {
                min_x[i] = a->min_pt[0];
                min_y[i] = a->min_pt[1];
                min_z[i] = a->min_pt[2];
                max_x[i] = a->max_pt[0];
                max_y[i] = a->max_pt[1];
                max_z[i] = a->max_pt[2];
            } else {
                min_x[i] = min_y[i] = min_z[i] = 0.0;
                max_x[i] = max_y[i] = max_z[i] = 0.0;
            }
        }
    }

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

    void Build(const BRepWorld& world)
    {
        const auto& alive = world.AliveEntities();
        const size_t n = alive.size();
        entity_ids.resize(n);
        types.resize(n);
        for (size_t i = 0; i < n; ++i)
        {
            uint32_t id = alive[i];
            entity_ids[i] = id;
            const Type* t = world.Types().Get(id);
            types[i] = t ? *t : Type::Empty;
        }
    }

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
    std::vector<uint32_t>            entity_ids;
    std::vector<std::vector<double>> all_params;

    void Build(const BRepWorld& world)
    {
        const auto& alive = world.AliveEntities();
        const size_t n = alive.size();
        entity_ids.resize(n);
        all_params.resize(n);
        for (size_t i = 0; i < n; ++i)
        {
            uint32_t id = alive[i];
            entity_ids[i] = id;
            all_params[i] = world.ExportEntityParams(id);
        }
    }

    const std::vector<double>& GetParams(size_t index) const
    {
        assert(index < entity_ids.size());
        return all_params[index];
    }

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

struct WorldViews
{
    AabbView  aabbs;
    TypeView  types;
    ParamView params;

    void Build(const BRepWorld& world)
    {
        aabbs.Build(world);
        types.Build(world);
        params.Build(world);
    }
};

} // namespace brepdb
