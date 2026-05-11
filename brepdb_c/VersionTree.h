#pragma once

#include "TypedPool.h"

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace brepdb
{

using WorldPtr = std::shared_ptr<BRepWorld>;

struct EntityEntry
{
    uint32_t persistent_id = 0;
    Type     type = Type::Empty;
    double   min_pt[3] = {0,0,0};
    double   max_pt[3] = {0,0,0};
    std::vector<double> params;

    uint32_t PersistentId() const { return persistent_id; }

    bool operator==(const EntityEntry& rhs) const;
    bool operator!=(const EntityEntry& rhs) const { return !(*this == rhs); }
};

struct ParamHunk
{
    uint32_t            offset;
    std::vector<double> data;
};

struct PoolDiff
{
    std::vector<EntityEntry> added;
    std::vector<EntityEntry> removed;

    struct ModifiedEntry
    {
        uint32_t old_persistent_id;
        uint32_t new_persistent_id;

        Type   old_type, new_type;
        double old_min_pt[3], old_max_pt[3];
        double new_min_pt[3], new_max_pt[3];

        uint32_t old_param_count;
        uint32_t new_param_count;

        std::vector<ParamHunk> forward_hunks;
        std::vector<ParamHunk> reverse_hunks;
    };

    std::vector<ModifiedEntry> modified;

    std::vector<uint32_t> new_order;
    std::vector<uint32_t> old_order;

    bool IsEmpty() const { return added.empty() && removed.empty() && modified.empty(); }
};

struct VersionNode
{
    uint32_t id        = 0;
    uint32_t parent_id = UINT32_MAX;

    std::vector<uint32_t> aux_parent_ids;
    std::vector<uint32_t> children;

    std::string op_desc;
    uint32_t    op_type   = 0;
    uint64_t    timestamp = 0;

    PoolDiff diff;
};

class VersionTree
{
public:
    using PidMapping = std::map<uint32_t, std::vector<uint32_t>>;

    VersionTree();

    uint32_t Commit(const BRepWorld&    new_world,
                    PoolDiff&&          diff,
                    const std::string&  op_desc,
                    uint32_t            op_type = 0);

    uint32_t Commit(const BRepWorld&    new_world,
                    const std::string&  op_desc,
                    uint32_t            op_type = 0);

    uint32_t Commit(const BRepWorld&    new_world,
                    const PidMapping&   pid_map,
                    const std::string&  op_desc,
                    uint32_t            op_type = 0);

    uint32_t AddRoot(const BRepWorld&   world,
                     const std::string& op_desc,
                     uint32_t           op_type = 0);

    uint32_t Branch(uint32_t            parent_id,
                    const BRepWorld&    new_world,
                    PoolDiff&&          diff,
                    const std::string&  op_desc,
                    uint32_t            op_type = 0);

    uint32_t Merge(uint32_t                       primary_parent_id,
                   const std::vector<uint32_t>&   aux_parent_ids,
                   const BRepWorld&               new_world,
                   PoolDiff&&                     diff,
                   const std::string&             op_desc,
                   uint32_t                       op_type = 0);

    uint32_t Merge(uint32_t                       primary_parent_id,
                   const std::vector<uint32_t>&   aux_parent_ids,
                   const BRepWorld&               new_world,
                   const PidMapping&              pid_map,
                   const std::string&             op_desc,
                   uint32_t                       op_type = 0);

    // ----- Navigation -----

    WorldPtr Checkout(uint32_t node_id);
    WorldPtr Undo();
    WorldPtr Redo(int child_index = -1);

    WorldPtr GetCurrentWorld() const { return m_current_world; }

    // ----- Query -----

    uint32_t GetCurrentId() const { return m_current_id; }
    uint32_t GetRootId()    const { return m_root_id; }

    std::vector<uint32_t> GetRoots() const;

    const VersionNode* GetNode(uint32_t id) const;
    size_t             GetNodeCount() const { return m_nodes.size(); }

    bool CanUndo() const;
    bool CanRedo() const;

    std::vector<uint32_t> GetPathFromRoot(uint32_t node_id) const;
    std::vector<uint32_t> GetLeaves() const;

    void TraverseAll(const std::function<void(const VersionNode&)>& visitor) const;

    // ----- Persistence -----

    bool SaveToFile(const std::string& filepath) const;
    bool LoadFromFile(const std::string& filepath);

    void StoreToByteArray(uint8_t** buf, uint32_t& len) const;
    void LoadFromByteArray(const uint8_t*  buf,
                           uint32_t        len,
                           const BRepWorld& current_world);

    void Clear();

    // ----- Static utilities -----

    static EntityEntry ExtractEntity(const BRepWorld& world, uint32_t entity_id);

    static void ComputeParamHunks(const std::vector<double>& old_params,
                                   const std::vector<double>& new_params,
                                   std::vector<ParamHunk>&    forward_hunks,
                                   std::vector<ParamHunk>&    reverse_hunks);

    static std::vector<double> ApplyParamHunks(const std::vector<double>& base,
                                                const std::vector<ParamHunk>& hunks,
                                                uint32_t target_size);

    static PoolDiff::ModifiedEntry BuildModifiedEntry(uint32_t           old_pid,
                                                       uint32_t           new_pid,
                                                       const EntityEntry& old_entry,
                                                       const EntityEntry& new_entry);

    static PoolDiff BuildDiffFromPidMapping(const BRepWorld& old_world,
                                             const BRepWorld& new_world,
                                             const PidMapping& pid_map);

    static WorldPtr ApplyForward(const BRepWorld& base,    const PoolDiff& diff);
    static WorldPtr ApplyReverse(const BRepWorld& current, const PoolDiff& diff);

    static PoolDiff ComputeDiff(const BRepWorld& old_world,
                                const BRepWorld& new_world);

private:
    uint32_t AllocNodeId();
    uint32_t InitRoot(const BRepWorld& world, const std::string& desc, uint32_t op_type);
    void     NavigateTo(uint32_t target_id);
    uint32_t FindLCA(uint32_t a, uint32_t b) const;

    static WorldPtr RebuildWorld(
        const std::unordered_map<uint32_t, EntityEntry>& entities,
        const std::vector<uint32_t>&                     order);

    std::unordered_map<uint32_t, VersionNode> m_nodes;

    uint32_t m_current_id = UINT32_MAX;
    uint32_t m_root_id    = UINT32_MAX;
    uint32_t m_next_id    = 0;

    WorldPtr m_current_world;

    std::unordered_map<uint32_t, WorldPtr> m_root_worlds;
};

} // namespace brepdb
