#include "ComponentDiff.h"

#include <algorithm>
#include <cassert>
#include <cstring>

namespace brepdb
{

// ============================================================
// ParamHunk computation
// ============================================================

void ComputeParamHunks(const std::vector<double>& old_params,
                      const std::vector<double>& new_params,
                      std::vector<ParamHunk>&    forward_hunks,
                      std::vector<ParamHunk>&    reverse_hunks)
{
    forward_hunks.clear();
    reverse_hunks.clear();

    const size_t   common       = std::min(old_params.size(), new_params.size());
    constexpr size_t COALESCE_GAP = 4;

    size_t i = 0;
    while (i < common)
    {
        if (old_params[i] == new_params[i]) { ++i; continue; }

        size_t start = i;
        while (i < common)
        {
            if (old_params[i] != new_params[i]) { ++i; continue; }
            size_t gap_end = std::min(i + COALESCE_GAP, common);
            bool   more    = false;
            for (size_t j = i; j < gap_end; ++j)
            {
                if (old_params[j] != new_params[j]) { more = true; break; }
            }
            if (more) { ++i; continue; }
            break;
        }

        ParamHunk fh;
        fh.offset = static_cast<uint32_t>(start);
        fh.data.assign(new_params.begin() + start, new_params.begin() + i);
        forward_hunks.push_back(std::move(fh));

        ParamHunk rh;
        rh.offset = static_cast<uint32_t>(start);
        rh.data.assign(old_params.begin() + start, old_params.begin() + i);
        reverse_hunks.push_back(std::move(rh));
    }

    if (new_params.size() > old_params.size())
    {
        ParamHunk fh;
        fh.offset = static_cast<uint32_t>(old_params.size());
        fh.data.assign(new_params.begin() + old_params.size(), new_params.end());
        forward_hunks.push_back(std::move(fh));
    }

    if (old_params.size() > new_params.size())
    {
        ParamHunk rh;
        rh.offset = static_cast<uint32_t>(new_params.size());
        rh.data.assign(old_params.begin() + new_params.size(), old_params.end());
        reverse_hunks.push_back(std::move(rh));
    }
}

std::vector<double> ApplyParamHunks(const std::vector<double>& base,
                                    const std::vector<ParamHunk>& hunks,
                                    uint32_t target_size)
{
    std::vector<double> result(target_size);

    size_t copy_len = std::min(base.size(), static_cast<size_t>(target_size));
    if (copy_len > 0)
        std::memcpy(result.data(), base.data(), copy_len * sizeof(double));

    for (const auto& h : hunks)
    {
        size_t end = h.offset + h.data.size();
        if (end > result.size()) result.resize(end);
        std::memcpy(result.data() + h.offset,
                    h.data.data(),
                    h.data.size() * sizeof(double));
    }

    result.resize(target_size);
    return result;
}

// ============================================================
// EntitySnapshot
// ============================================================

EntitySnapshot EntitySnapshot::Extract(const BRepWorld& world, uint32_t entity_id)
{
    EntitySnapshot e;
    e.id = entity_id;

    const Type* t = world.Types().Get(entity_id);
    if (t) e.type = *t;

    const AabbComp* aabb = world.Aabbs().Get(entity_id);
    if (aabb)
    {
        std::memcpy(e.min_pt, aabb->min_pt, sizeof(e.min_pt));
        std::memcpy(e.max_pt, aabb->max_pt, sizeof(e.max_pt));
    }

    e.params = world.ExportEntityParams(entity_id);
    return e;
}

// ============================================================
// Internal helpers: per-component patch building
// ============================================================

namespace
{

bool AabbEqual(const AabbComp& a, const AabbComp& b)
{
    return std::memcmp(a.min_pt, b.min_pt, 24) == 0 &&
           std::memcmp(a.max_pt, b.max_pt, 24) == 0;
}

// Build patches for an entity that exists in both worlds (with possibly different pids)
void EmitPatches(uint32_t pid_in_diff,
                 const EntitySnapshot& old_snap,
                 const EntitySnapshot& new_snap,
                 std::vector<ComponentPatch>& out)
{
    if (old_snap.type != new_snap.type)
    {
        ComponentPatch p;
        p.entity_id = pid_in_diff;
        p.kind = ComponentKind::Type;
        p.old_data = { static_cast<uint8_t>(old_snap.type) };
        p.new_data = { static_cast<uint8_t>(new_snap.type) };
        out.push_back(std::move(p));
    }

    if (std::memcmp(old_snap.min_pt, new_snap.min_pt, 24) != 0 ||
        std::memcmp(old_snap.max_pt, new_snap.max_pt, 24) != 0)
    {
        ComponentPatch p;
        p.entity_id = pid_in_diff;
        p.kind = ComponentKind::Aabb;
        AabbComp old_a, new_a;
        std::memcpy(old_a.min_pt, old_snap.min_pt, 24);
        std::memcpy(old_a.max_pt, old_snap.max_pt, 24);
        std::memcpy(new_a.min_pt, new_snap.min_pt, 24);
        std::memcpy(new_a.max_pt, new_snap.max_pt, 24);
        p.old_data = detail::PackAabb(old_a);
        p.new_data = detail::PackAabb(new_a);
        out.push_back(std::move(p));
    }

    if (old_snap.params != new_snap.params)
    {
        ComponentPatch p;
        p.entity_id = pid_in_diff;
        p.kind = ComponentKind::Params;
        p.old_param_count = static_cast<uint32_t>(old_snap.params.size());
        p.new_param_count = static_cast<uint32_t>(new_snap.params.size());
        ComputeParamHunks(old_snap.params, new_snap.params,
                          p.forward_hunks, p.reverse_hunks);
        out.push_back(std::move(p));
    }
}

void RegisterSnapshot(BRepWorld& w, const EntitySnapshot& s)
{
    w.RegisterEntity(s.id);
    w.Types().Set(s.id, s.type);
    AabbComp a;
    std::memcpy(a.min_pt, s.min_pt, 24);
    std::memcpy(a.max_pt, s.max_pt, 24);
    w.Aabbs().Set(s.id, a);
    if (!s.params.empty())
    {
        ParamsComp pc;
        pc.data = s.params;
        w.Params().Set(s.id, pc);
    }
}

// Copy entity components from src world to dst world (full clone of one entity).
void CloneEntity(const BRepWorld& src, BRepWorld& dst, uint32_t id)
{
    dst.RegisterEntity(id);
    if (auto* t = src.Types().Get(id))       dst.Types().Set(id, *t);
    if (auto* a = src.Aabbs().Get(id))       dst.Aabbs().Set(id, *a);
    if (auto* pos = src.Positions().Get(id)) dst.Positions().Set(id, *pos);
    if (auto* tol = src.Tolerances().Get(id))dst.Tolerances().Set(id, *tol);
    if (auto* p = src.Params().Get(id))      dst.Params().Set(id, *p);
    if (auto* c = src.Curves().Get(id))      dst.Curves().Set(id, *c);
    if (auto* s = src.Surfaces().Get(id))    dst.Surfaces().Set(id, *s);
    if (auto* et = src.EdgeTopos().Get(id))  dst.EdgeTopos().Set(id, *et);
    if (auto* ft = src.FaceTopos().Get(id))  dst.FaceTopos().Set(id, *ft);
    if (auto* st = src.SolidTopos().Get(id)) dst.SolidTopos().Set(id, *st);
}

} // namespace

// ============================================================
// Compute (no pid remap)
// ============================================================

ComponentDiff ComponentDiff::Compute(const BRepWorld& old_world,
                                     const BRepWorld& new_world)
{
    ComponentDiff diff;

    const auto& old_alive = old_world.AliveEntities();
    const auto& new_alive = new_world.AliveEntities();

    diff.old_order = old_alive;
    diff.new_order = new_alive;

    std::set<uint32_t> old_set(old_alive.begin(), old_alive.end());
    std::set<uint32_t> new_set(new_alive.begin(), new_alive.end());

    for (uint32_t id : new_alive)
    {
        if (!old_set.count(id))
            diff.added.push_back(EntitySnapshot::Extract(new_world, id));
    }

    for (uint32_t id : old_alive)
    {
        if (!new_set.count(id))
            diff.removed.push_back(EntitySnapshot::Extract(old_world, id));
    }

    for (uint32_t id : new_alive)
    {
        if (!old_set.count(id)) continue;
        auto old_snap = EntitySnapshot::Extract(old_world, id);
        auto new_snap = EntitySnapshot::Extract(new_world, id);
        EmitPatches(id, old_snap, new_snap, diff.patches);
    }

    return diff;
}

// ============================================================
// Compute (with pid remap)
// ============================================================

ComponentDiff ComponentDiff::ComputeWithPidMapping(const BRepWorld& old_world,
                                                   const BRepWorld& new_world,
                                                   const PidMapping& pid_map)
{
    ComponentDiff diff;

    const auto& old_alive = old_world.AliveEntities();
    const auto& new_alive = new_world.AliveEntities();

    diff.old_order = old_alive;
    diff.new_order = new_alive;

    std::set<uint32_t> old_set(old_alive.begin(), old_alive.end());
    std::set<uint32_t> new_set(new_alive.begin(), new_alive.end());

    std::set<uint32_t> accounted_new;
    std::set<uint32_t> accounted_old;

    for (const auto& [old_pid, new_pids] : pid_map)
    {
        accounted_old.insert(old_pid);
        bool has_old = old_set.count(old_pid) > 0;

        if (new_pids.empty())
        {
            if (has_old)
                diff.removed.push_back(EntitySnapshot::Extract(old_world, old_pid));
            continue;
        }

        for (size_t k = 0; k < new_pids.size(); ++k)
        {
            uint32_t new_pid = new_pids[k];
            accounted_new.insert(new_pid);
            if (!new_set.count(new_pid)) continue;

            if (k == 0 && has_old)
            {
                auto old_snap = EntitySnapshot::Extract(old_world, old_pid);
                auto new_snap = EntitySnapshot::Extract(new_world, new_pid);
                if (old_pid != new_pid)
                    diff.renamed.emplace_back(old_pid, new_pid);
                EmitPatches(new_pid, old_snap, new_snap, diff.patches);
            }
            else
            {
                diff.added.push_back(EntitySnapshot::Extract(new_world, new_pid));
            }
        }
    }

    for (uint32_t pid : new_alive)
    {
        if (accounted_new.count(pid)) continue;
        if (!old_set.count(pid))
        {
            diff.added.push_back(EntitySnapshot::Extract(new_world, pid));
        }
        else
        {
            auto old_snap = EntitySnapshot::Extract(old_world, pid);
            auto new_snap = EntitySnapshot::Extract(new_world, pid);
            EmitPatches(pid, old_snap, new_snap, diff.patches);
            accounted_old.insert(pid);
        }
    }

    for (uint32_t pid : old_alive)
    {
        if (accounted_old.count(pid)) continue;
        if (!new_set.count(pid))
            diff.removed.push_back(EntitySnapshot::Extract(old_world, pid));
    }

    return diff;
}

// ============================================================
// Apply forward / reverse
//
// Both build a working table of EntitySnapshots, mutate them, then
// emit a fresh BRepWorld in the requested order.
// ============================================================

namespace
{

// Apply a single patch to a snapshot table (in-place).
void ApplyPatchForward(std::unordered_map<uint32_t, EntitySnapshot>& ents,
                       const ComponentPatch& p)
{
    auto it = ents.find(p.entity_id);
    if (it == ents.end()) return;
    EntitySnapshot& s = it->second;

    switch (p.kind)
    {
    case ComponentKind::Type:
        if (!p.new_data.empty())
            s.type = static_cast<Type>(p.new_data[0]);
        break;
    case ComponentKind::Aabb:
    {
        AabbComp a = detail::UnpackAabb(p.new_data);
        std::memcpy(s.min_pt, a.min_pt, 24);
        std::memcpy(s.max_pt, a.max_pt, 24);
        break;
    }
    case ComponentKind::Params:
        s.params = ApplyParamHunks(s.params, p.forward_hunks, p.new_param_count);
        break;
    }
}

void ApplyPatchReverse(std::unordered_map<uint32_t, EntitySnapshot>& ents,
                       const ComponentPatch& p)
{
    auto it = ents.find(p.entity_id);
    if (it == ents.end()) return;
    EntitySnapshot& s = it->second;

    switch (p.kind)
    {
    case ComponentKind::Type:
        if (!p.old_data.empty())
            s.type = static_cast<Type>(p.old_data[0]);
        break;
    case ComponentKind::Aabb:
    {
        AabbComp a = detail::UnpackAabb(p.old_data);
        std::memcpy(s.min_pt, a.min_pt, 24);
        std::memcpy(s.max_pt, a.max_pt, 24);
        break;
    }
    case ComponentKind::Params:
        s.params = ApplyParamHunks(s.params, p.reverse_hunks, p.old_param_count);
        break;
    }
}

WorldPtr RebuildWorld(const std::unordered_map<uint32_t, EntitySnapshot>& ents,
                      const std::vector<uint32_t>& order)
{
    auto w = std::make_shared<BRepWorld>();
    for (uint32_t id : order)
    {
        auto it = ents.find(id);
        if (it == ents.end()) continue;
        RegisterSnapshot(*w, it->second);
    }
    // Include any entity not in `order` (defensive -- should not happen)
    for (auto& [id, s] : ents)
    {
        if (w->IsAlive(id)) continue;
        RegisterSnapshot(*w, s);
    }
    w->RebuildTypedFromParams();
    return w;
}

} // namespace

WorldPtr ComponentDiff::ApplyForward(const BRepWorld& base, const ComponentDiff& diff)
{
    std::unordered_map<uint32_t, EntitySnapshot> ents;
    for (uint32_t id : base.AliveEntities())
        ents[id] = EntitySnapshot::Extract(base, id);

    // 1. Remove
    for (const auto& s : diff.removed) ents.erase(s.id);

    // 2. Rename (old_pid -> new_pid). Use a temp pass to avoid collisions.
    for (const auto& [old_pid, new_pid] : diff.renamed)
    {
        auto it = ents.find(old_pid);
        if (it == ents.end()) continue;
        EntitySnapshot s = std::move(it->second);
        ents.erase(it);
        s.id = new_pid;
        ents[new_pid] = std::move(s);
    }

    // 3. Patch
    for (const auto& p : diff.patches) ApplyPatchForward(ents, p);

    // 4. Add
    for (const auto& s : diff.added) ents[s.id] = s;

    return RebuildWorld(ents, diff.new_order);
}

WorldPtr ComponentDiff::ApplyReverse(const BRepWorld& current, const ComponentDiff& diff)
{
    std::unordered_map<uint32_t, EntitySnapshot> ents;
    for (uint32_t id : current.AliveEntities())
        ents[id] = EntitySnapshot::Extract(current, id);

    // 1. Reverse-add (remove what was added)
    for (const auto& s : diff.added) ents.erase(s.id);

    // 2. Reverse-patch (under new_pid)
    for (const auto& p : diff.patches) ApplyPatchReverse(ents, p);

    // 3. Reverse-rename (new_pid -> old_pid)
    for (auto it = diff.renamed.rbegin(); it != diff.renamed.rend(); ++it)
    {
        auto eit = ents.find(it->second);
        if (eit == ents.end()) continue;
        EntitySnapshot s = std::move(eit->second);
        ents.erase(eit);
        s.id = it->first;
        ents[it->first] = std::move(s);
    }

    // 4. Reverse-remove (restore removed entities)
    for (const auto& s : diff.removed) ents[s.id] = s;

    return RebuildWorld(ents, diff.old_order);
}

} // namespace brepdb
