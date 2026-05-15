#pragma once

#include "TypedPool.h"

#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

namespace brepdb
{

using WorldPtr = std::shared_ptr<BRepWorld>;

// ============================================================
// Param hunk: stores only changed ranges of a double array
// ============================================================

struct ParamHunk
{
    uint32_t            offset;
    std::vector<double> data;
};

void ComputeParamHunks(const std::vector<double>& old_params,
                       const std::vector<double>& new_params,
                       std::vector<ParamHunk>&    forward_hunks,
                       std::vector<ParamHunk>&    reverse_hunks);

std::vector<double> ApplyParamHunks(const std::vector<double>& base,
                                    const std::vector<ParamHunk>& hunks,
                                    uint32_t target_size);

// ============================================================
// Component-level diff types
// ============================================================

enum class ComponentKind : uint8_t
{
    Type = 0,
    Aabb = 1,
    Params = 2,
};

struct EntitySnapshot
{
    uint32_t id = 0;
    Type     type = Type::Empty;
    double   min_pt[3] = {0,0,0};
    double   max_pt[3] = {0,0,0};
    std::vector<double> params;

    static EntitySnapshot Extract(const BRepWorld& world, uint32_t entity_id);
};

struct ComponentPatch
{
    uint32_t      entity_id = 0;
    ComponentKind kind = ComponentKind::Type;

    // For Type / Aabb
    std::vector<uint8_t> old_data;
    std::vector<uint8_t> new_data;

    // For Params (hunk-compressed; old_data/new_data left empty)
    uint32_t old_param_count = 0;
    uint32_t new_param_count = 0;
    std::vector<ParamHunk> forward_hunks;
    std::vector<ParamHunk> reverse_hunks;
};

struct ComponentDiff
{
    std::vector<EntitySnapshot> added;
    std::vector<EntitySnapshot> removed;

    std::vector<std::pair<uint32_t,uint32_t>> renamed; // old_pid -> new_pid

    std::vector<ComponentPatch> patches;

    std::vector<uint32_t> old_order;
    std::vector<uint32_t> new_order;

    bool IsEmpty() const
    {
        return added.empty() && removed.empty() && renamed.empty() && patches.empty();
    }

    size_t PatchBytes() const
    {
        size_t total = 0;
        for (auto& p : patches)
        {
            total += p.old_data.size() + p.new_data.size();
            for (auto& h : p.forward_hunks) total += h.data.size() * sizeof(double);
            for (auto& h : p.reverse_hunks) total += h.data.size() * sizeof(double);
        }
        return total;
    }

    using PidMapping = std::map<uint32_t, std::vector<uint32_t>>;

    static ComponentDiff Compute(const BRepWorld& old_world,
                                 const BRepWorld& new_world);

    static ComponentDiff ComputeWithPidMapping(const BRepWorld& old_world,
                                               const BRepWorld& new_world,
                                               const PidMapping& pid_map);

    static WorldPtr ApplyForward(const BRepWorld& base,
                                 const ComponentDiff& diff);

    static WorldPtr ApplyReverse(const BRepWorld& current,
                                 const ComponentDiff& diff);
};

// ============================================================
// Inline helpers: pack / unpack component data
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

} // namespace detail

} // namespace brepdb
