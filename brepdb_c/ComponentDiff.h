#pragma once

#include "TypedPool.h"

#include <cstdint>
#include <cstring>
#include <set>
#include <unordered_map>
#include <vector>

namespace brepdb
{

enum class ComponentKind : uint8_t
{
    Type       = 0,
    Aabb       = 1,
    Params     = 2,
};

struct ComponentPatch
{
    uint32_t      entity_id;
    ComponentKind kind;
    std::vector<uint8_t> old_data;
    std::vector<uint8_t> new_data;
};

struct ComponentDiff
{
    std::vector<uint32_t> added_entities;
    std::vector<uint32_t> removed_entities;
    std::vector<ComponentPatch> patches;

    bool IsEmpty() const
    {
        return added_entities.empty() && removed_entities.empty() && patches.empty();
    }

    size_t PatchBytes() const
    {
        size_t total = 0;
        for (auto& p : patches)
            total += p.old_data.size() + p.new_data.size();
        return total;
    }

    static ComponentDiff Compute(const BRepWorld& old_world,
                                 const BRepWorld& new_world);

    static BRepWorld ApplyForward(const BRepWorld& base,
                                  const ComponentDiff& diff);

    static BRepWorld ApplyReverse(const BRepWorld& current,
                                  const ComponentDiff& diff);
};

// ============================================================
// Inline implementation
// ============================================================

namespace detail
{
    inline std::vector<uint8_t> PackAabb(const AabbComp& aabb)
    {
        std::vector<uint8_t> buf(48);
        std::memcpy(buf.data(),      aabb.min_pt, 24);
        std::memcpy(buf.data() + 24, aabb.max_pt, 24);
        return buf;
    }

    inline AabbComp UnpackAabb(const std::vector<uint8_t>& buf)
    {
        AabbComp aabb;
        std::memcpy(aabb.min_pt, buf.data(),      24);
        std::memcpy(aabb.max_pt, buf.data() + 24, 24);
        return aabb;
    }

    inline std::vector<uint8_t> PackParams(const ParamsComp& params)
    {
        size_t bytes = params.data.size() * sizeof(double);
        std::vector<uint8_t> buf(bytes);
        if (bytes > 0)
            std::memcpy(buf.data(), params.data.data(), bytes);
        return buf;
    }

    inline ParamsComp UnpackParams(const std::vector<uint8_t>& buf)
    {
        ParamsComp p;
        size_t count = buf.size() / sizeof(double);
        p.data.resize(count);
        if (count > 0)
            std::memcpy(p.data.data(), buf.data(), buf.size());
        return p;
    }
} // namespace detail

inline ComponentDiff ComponentDiff::Compute(const BRepWorld& old_world,
                                             const BRepWorld& new_world)
{
    ComponentDiff diff;

    std::set<uint32_t> old_set(old_world.AliveEntities().begin(),
                                old_world.AliveEntities().end());
    std::set<uint32_t> new_set(new_world.AliveEntities().begin(),
                                new_world.AliveEntities().end());

    for (uint32_t id : old_set)
    {
        if (new_set.find(id) == new_set.end())
            diff.removed_entities.push_back(id);
    }

    for (uint32_t id : new_set)
    {
        if (old_set.find(id) == old_set.end())
            diff.added_entities.push_back(id);
    }

    for (uint32_t id : new_set)
    {
        if (old_set.find(id) == old_set.end()) continue;

        auto* old_type = old_world.Types().Get(id);
        auto* new_type = new_world.Types().Get(id);
        if (old_type && new_type && *old_type != *new_type)
        {
            ComponentPatch p;
            p.entity_id = id;
            p.kind = ComponentKind::Type;
            p.old_data = { static_cast<uint8_t>(*old_type) };
            p.new_data = { static_cast<uint8_t>(*new_type) };
            diff.patches.push_back(std::move(p));
        }

        auto* old_aabb = old_world.Aabbs().Get(id);
        auto* new_aabb = new_world.Aabbs().Get(id);
        if (old_aabb && new_aabb &&
            (std::memcmp(old_aabb->min_pt, new_aabb->min_pt, 24) != 0 ||
             std::memcmp(old_aabb->max_pt, new_aabb->max_pt, 24) != 0))
        {
            ComponentPatch p;
            p.entity_id = id;
            p.kind = ComponentKind::Aabb;
            p.old_data = detail::PackAabb(*old_aabb);
            p.new_data = detail::PackAabb(*new_aabb);
            diff.patches.push_back(std::move(p));
        }

        auto* old_params = old_world.Params().Get(id);
        auto* new_params = new_world.Params().Get(id);
        if (old_params && new_params && old_params->data != new_params->data)
        {
            ComponentPatch p;
            p.entity_id = id;
            p.kind = ComponentKind::Params;
            p.old_data = detail::PackParams(*old_params);
            p.new_data = detail::PackParams(*new_params);
            diff.patches.push_back(std::move(p));
        }
    }

    return diff;
}

inline BRepWorld ComponentDiff::ApplyForward(const BRepWorld& base,
                                              const ComponentDiff& diff)
{
    std::set<uint32_t> removed_set(diff.removed_entities.begin(),
                                    diff.removed_entities.end());

    BRepWorld result;

    for (uint32_t id : base.AliveEntities())
    {
        if (removed_set.count(id)) continue;

        result.RegisterEntity(id);

        auto* t = base.Types().Get(id);
        if (t) result.Types().Set(id, *t);

        auto* a = base.Aabbs().Get(id);
        if (a) result.Aabbs().Set(id, *a);

        auto* pos = base.Positions().Get(id);
        if (pos) result.Positions().Set(id, *pos);

        auto* tol = base.Tolerances().Get(id);
        if (tol) result.Tolerances().Set(id, *tol);

        auto* p = base.Params().Get(id);
        if (p) result.Params().Set(id, *p);

        auto* c = base.Curves().Get(id);
        if (c) result.Curves().Set(id, *c);

        auto* s = base.Surfaces().Get(id);
        if (s) result.Surfaces().Set(id, *s);

        auto* et = base.EdgeTopos().Get(id);
        if (et) result.EdgeTopos().Set(id, *et);

        auto* ft = base.FaceTopos().Get(id);
        if (ft) result.FaceTopos().Set(id, *ft);

        auto* st = base.SolidTopos().Get(id);
        if (st) result.SolidTopos().Set(id, *st);
    }

    for (auto& patch : diff.patches)
    {
        if (!result.IsAlive(patch.entity_id)) continue;

        switch (patch.kind)
        {
        case ComponentKind::Type:
            result.Types().Set(patch.entity_id, static_cast<Type>(patch.new_data[0]));
            break;
        case ComponentKind::Aabb:
            result.Aabbs().Set(patch.entity_id, detail::UnpackAabb(patch.new_data));
            break;
        case ComponentKind::Params:
            result.Params().Set(patch.entity_id, detail::UnpackParams(patch.new_data));
            break;
        }
    }

    return result;
}

inline BRepWorld ComponentDiff::ApplyReverse(const BRepWorld& current,
                                              const ComponentDiff& diff)
{
    ComponentDiff reversed;
    reversed.added_entities = diff.removed_entities;
    reversed.removed_entities = diff.added_entities;
    reversed.patches.reserve(diff.patches.size());
    for (auto& p : diff.patches)
    {
        ComponentPatch rp;
        rp.entity_id = p.entity_id;
        rp.kind      = p.kind;
        rp.old_data  = p.new_data;
        rp.new_data  = p.old_data;
        reversed.patches.push_back(std::move(rp));
    }
    return ApplyForward(current, reversed);
}

} // namespace brepdb
