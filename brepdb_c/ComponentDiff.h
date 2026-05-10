#pragma once

#include "GeomPool.h"

#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

namespace brepdb
{

// Which aspect of an entity changed.
enum class ComponentKind : uint8_t
{
    Type       = 0,
    Aabb       = 1,
    Params     = 2,
};

// A single component change for one entity.
struct ComponentPatch
{
    uint32_t      entity_id;
    ComponentKind kind;
    std::vector<uint8_t> old_data;
    std::vector<uint8_t> new_data;
};

// Full diff expressed as component patches.
struct ComponentDiff
{
    std::vector<uint32_t> added_entities;
    std::vector<uint32_t> removed_entities;
    std::vector<ComponentPatch> patches;

    bool IsEmpty() const
    {
        return added_entities.empty() && removed_entities.empty() && patches.empty();
    }

    // Byte size of all patches (for measuring compactness).
    size_t PatchBytes() const
    {
        size_t total = 0;
        for (auto& p : patches)
            total += p.old_data.size() + p.new_data.size();
        return total;
    }

    // Build a ComponentDiff from two pools matched by persistent_id.
    static ComponentDiff Compute(const GeometryPool& old_pool,
                                  const GeometryPool& new_pool);

    // Apply forward: old_pool + diff -> new_pool state.
    static GeometryPool ApplyForward(const GeometryPool& base,
                                      const ComponentDiff& diff);

    // Apply reverse: new_pool + diff -> old_pool state.
    static GeometryPool ApplyReverse(const GeometryPool& current,
                                      const ComponentDiff& diff);
};

// ============================================================
// Inline implementation
// ============================================================

namespace detail
{
    inline std::vector<uint8_t> PackAabb(const GeomHeader& h)
    {
        std::vector<uint8_t> buf(48);
        std::memcpy(buf.data(),      h.min_pt, 24);
        std::memcpy(buf.data() + 24, h.max_pt, 24);
        return buf;
    }

    inline void UnpackAabb(const std::vector<uint8_t>& buf, GeomHeader& h)
    {
        std::memcpy(h.min_pt, buf.data(),      24);
        std::memcpy(h.max_pt, buf.data() + 24, 24);
    }

    inline std::vector<uint8_t> PackParams(const GeometryPool& pool, const GeomHeader& h)
    {
        size_t bytes = h.param_count * sizeof(double);
        std::vector<uint8_t> buf(bytes);
        if (bytes > 0)
            std::memcpy(buf.data(), pool.data_pool.data() + h.param_offset, bytes);
        return buf;
    }

    inline std::vector<double> UnpackParams(const std::vector<uint8_t>& buf)
    {
        size_t count = buf.size() / sizeof(double);
        std::vector<double> result(count);
        if (count > 0)
            std::memcpy(result.data(), buf.data(), buf.size());
        return result;
    }
} // namespace detail

inline ComponentDiff ComponentDiff::Compute(const GeometryPool& old_pool,
                                             const GeometryPool& new_pool)
{
    ComponentDiff diff;

    // Build id -> index maps
    std::unordered_map<uint32_t, uint32_t> old_map, new_map;
    for (uint32_t i = 0; i < old_pool.headers.size(); ++i)
        old_map[old_pool.headers[i].persistent_id] = i;
    for (uint32_t i = 0; i < new_pool.headers.size(); ++i)
        new_map[new_pool.headers[i].persistent_id] = i;

    // Removed: in old but not in new
    for (auto& [pid, idx] : old_map)
    {
        if (new_map.find(pid) == new_map.end())
            diff.removed_entities.push_back(pid);
    }

    // Added: in new but not in old
    for (auto& [pid, idx] : new_map)
    {
        if (old_map.find(pid) == old_map.end())
            diff.added_entities.push_back(pid);
    }

    // Modified: in both, check each component
    for (auto& [pid, new_idx] : new_map)
    {
        auto it = old_map.find(pid);
        if (it == old_map.end()) continue;

        uint32_t old_idx = it->second;
        const auto& oh = old_pool.headers[old_idx];
        const auto& nh = new_pool.headers[new_idx];

        // Type changed?
        if (oh.type != nh.type)
        {
            ComponentPatch p;
            p.entity_id = pid;
            p.kind = ComponentKind::Type;
            p.old_data = { static_cast<uint8_t>(oh.type) };
            p.new_data = { static_cast<uint8_t>(nh.type) };
            diff.patches.push_back(std::move(p));
        }

        // AABB changed?
        if (std::memcmp(oh.min_pt, nh.min_pt, 24) != 0 ||
            std::memcmp(oh.max_pt, nh.max_pt, 24) != 0)
        {
            ComponentPatch p;
            p.entity_id = pid;
            p.kind = ComponentKind::Aabb;
            p.old_data = detail::PackAabb(oh);
            p.new_data = detail::PackAabb(nh);
            diff.patches.push_back(std::move(p));
        }

        // Params changed?
        auto old_params = detail::PackParams(old_pool, oh);
        auto new_params = detail::PackParams(new_pool, nh);
        if (old_params != new_params)
        {
            ComponentPatch p;
            p.entity_id = pid;
            p.kind = ComponentKind::Params;
            p.old_data = std::move(old_params);
            p.new_data = std::move(new_params);
            diff.patches.push_back(std::move(p));
        }
    }

    return diff;
}

inline GeometryPool ComponentDiff::ApplyForward(const GeometryPool& base,
                                                 const ComponentDiff& diff)
{
    // Start with a copy, then apply patches
    std::unordered_map<uint32_t, uint32_t> id_map;
    for (uint32_t i = 0; i < base.headers.size(); ++i)
        id_map[base.headers[i].persistent_id] = i;

    // Collect entities to keep (not removed)
    std::vector<uint32_t> keep_order;
    for (auto& h : base.headers)
    {
        uint32_t pid = h.persistent_id;
        bool removed = false;
        for (auto r : diff.removed_entities)
            if (r == pid) { removed = true; break; }
        if (!removed)
            keep_order.push_back(pid);
    }

    // Build result headers + params from base for kept entities
    GeometryPool result;
    std::unordered_map<uint32_t, uint32_t> result_map;
    for (auto pid : keep_order)
    {
        uint32_t src_idx = id_map[pid];
        GeomHeader h = base.headers[src_idx];
        uint32_t new_offset = static_cast<uint32_t>(result.data_pool.size());
        const double* src = base.data_pool.data() + h.param_offset;
        result.data_pool.insert(result.data_pool.end(), src, src + h.param_count);
        h.param_offset = new_offset;
        result_map[pid] = static_cast<uint32_t>(result.headers.size());
        result.headers.push_back(h);
    }

    // Apply patches
    for (auto& patch : diff.patches)
    {
        auto it = result_map.find(patch.entity_id);
        if (it == result_map.end()) continue;
        uint32_t idx = it->second;
        auto& h = result.headers[idx];

        switch (patch.kind)
        {
        case ComponentKind::Type:
            h.type = static_cast<Type>(patch.new_data[0]);
            break;
        case ComponentKind::Aabb:
            detail::UnpackAabb(patch.new_data, h);
            break;
        case ComponentKind::Params:
        {
            auto new_params = detail::UnpackParams(patch.new_data);
            // Replace params in data_pool
            uint32_t old_offset = h.param_offset;
            uint32_t old_count  = h.param_count;
            uint32_t new_count  = static_cast<uint32_t>(new_params.size());

            if (new_count == old_count)
            {
                std::memcpy(result.data_pool.data() + old_offset,
                            new_params.data(), new_count * sizeof(double));
            }
            else
            {
                // Append at end, update offset
                h.param_offset = static_cast<uint32_t>(result.data_pool.size());
                h.param_count  = new_count;
                result.data_pool.insert(result.data_pool.end(),
                                        new_params.begin(), new_params.end());
            }
            break;
        }
        }
    }

    // Add new entities (with empty params for now — caller must supply full pool)
    // In practice, added entities come from the new_pool directly.
    // This simplified version doesn't carry full entity data for adds.
    // For a complete implementation, ComponentDiff would store EntityEntry for adds.

    return result;
}

inline GeometryPool ComponentDiff::ApplyReverse(const GeometryPool& current,
                                                 const ComponentDiff& diff)
{
    // Symmetric: swap old/new in patches, swap added/removed
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
