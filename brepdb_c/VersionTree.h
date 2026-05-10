#pragma once

#include "GeomPool.h"

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace brepdb
{

// Immutable pool snapshot, shared across multiple holders.
// NavigateTo creates a new object each time; anyone still holding
// an older PoolPtr sees a stable snapshot that won't be mutated.
using PoolPtr = std::shared_ptr<GeometryPool>;

// A self-contained entity extracted from GeometryPool.
// param_offset is normalized to 0, making the entry independent of pool layout.
struct EntityEntry
{
    GeomHeader          header;
    std::vector<double> params;

    uint32_t PersistentId() const { return header.persistent_id; }

    bool operator==(const EntityEntry& rhs) const;
    bool operator!=(const EntityEntry& rhs) const { return !(*this == rhs); }
};

// A contiguous changed region inside a double array.
struct ParamHunk
{
    uint32_t            offset;  // start index into the array
    std::vector<double> data;    // replacement values from offset onward
};

// Structural diff between two GeometryPool snapshots.
//
// Entities are matched by persistent_id:
//   new pool only            -> ADDED   (full EntityEntry stored)
//   old pool only            -> REMOVED (full EntityEntry stored)
//   both pools, data differs -> MODIFIED (headers full, params as hunks)
//
// persistent_id may change across ops (CalcUID encodes op_id),
// so ModifiedEntry stores both old and new pids explicitly.
struct PoolDiff
{
    std::vector<EntityEntry> added;
    std::vector<EntityEntry> removed;

    struct ModifiedEntry
    {
        uint32_t old_persistent_id;
        uint32_t new_persistent_id;

        GeomHeader old_header;
        GeomHeader new_header;

        uint32_t old_param_count;
        uint32_t new_param_count;

        std::vector<ParamHunk> forward_hunks;  // old params -> new params
        std::vector<ParamHunk> reverse_hunks;  // new params -> old params
    };

    std::vector<ModifiedEntry> modified;

    std::vector<uint32_t> new_order;  // pids in new pool order
    std::vector<uint32_t> old_order;  // pids in old pool order

    bool IsEmpty() const { return added.empty() && removed.empty() && modified.empty(); }
};

// One node in the version DAG.
// parent_id is the primary parent (diff base, undo target).
// aux_parent_ids holds secondary parents (e.g. tool body in a boolean cut).
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
    // Mapping from old persistent_id to new persistent_id(s):
    //   empty list  -> entity deleted
    //   one entry   -> entity modified (CalcUID assigned a new uid)
    //   multi entry -> entity split (first = modified, rest = added)
    // Entities in new_pool not referenced as targets are detected as ADDED.
    // Entities not present in the mapping fall back to pid-matching.
    using PidMapping = std::map<uint32_t, std::vector<uint32_t>>;

    VersionTree();

    // Primary: diff pre-built by caller from a PidMapping. O(changed).
    uint32_t Commit(const GeometryPool& new_pool,
                    PoolDiff&&          diff,
                    const std::string&  op_desc,
                    uint32_t            op_type = 0);

    // Fallback: compute diff by comparing both pools. O(total).
    uint32_t Commit(const GeometryPool& new_pool,
                    const std::string&  op_desc,
                    uint32_t            op_type = 0);

    // pid_map form: builds diff from BRepHistory-derived mapping. On the
    // very first call the tree has no root, so pid_map is irrelevant and
    // new_pool is installed as the root snapshot.
    uint32_t Commit(const GeometryPool& new_pool,
                    const PidMapping&   pid_map,
                    const std::string&  op_desc,
                    uint32_t            op_type = 0);

    // Create an additional root node (independent body, e.g. a tool shape).
    // Unlike InitRoot, this does NOT clear the tree.  The current pool and
    // current_id are left unchanged.
    uint32_t AddRoot(const GeometryPool& pool,
                     const std::string&  op_desc,
                     uint32_t            op_type = 0);

    uint32_t Branch(uint32_t            parent_id,
                    const GeometryPool& new_pool,
                    PoolDiff&&          diff,
                    const std::string&  op_desc,
                    uint32_t            op_type = 0);

    // Merge: create a node with one primary parent and one or more auxiliary
    // parents.  diff is computed against the primary parent's pool.
    // All parents (primary + aux) get the new node in their children list.
    uint32_t Merge(uint32_t                       primary_parent_id,
                   const std::vector<uint32_t>&   aux_parent_ids,
                   const GeometryPool&            new_pool,
                   PoolDiff&&                     diff,
                   const std::string&             op_desc,
                   uint32_t                       op_type = 0);

    uint32_t Merge(uint32_t                       primary_parent_id,
                   const std::vector<uint32_t>&   aux_parent_ids,
                   const GeometryPool&            new_pool,
                   const PidMapping&              pid_map,
                   const std::string&             op_desc,
                   uint32_t                       op_type = 0);

    // ----- Navigation -----
    // Returns a shared_ptr to an immutable pool snapshot.
    // The returned pointer remains valid even after subsequent navigation.

    PoolPtr Checkout(uint32_t node_id);
    PoolPtr Undo();
    PoolPtr Redo(int child_index = -1);

    PoolPtr GetCurrentPool() const { return m_current_pool; }

    // ----- Query -----

    uint32_t GetCurrentId() const { return m_current_id; }
    uint32_t GetRootId()    const { return m_root_id; }

    // All root node IDs (nodes with parent_id == UINT32_MAX).
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

    // Compact byte-array form for embedding in a BrepDB meta page.
    // Does NOT contain the root pool snapshot; entity data lives in the RTree.
    // LoadFromByteArray requires the caller to pass the current pool
    // (exported from the RTree) as the navigation anchor.
    void StoreToByteArray(uint8_t** buf, uint32_t& len) const;
    void LoadFromByteArray(const uint8_t*      buf,
                           uint32_t            len,
                           const GeometryPool& current_pool);

    void Clear();

    // ----- Static utilities (public for diff construction and testing) -----

    static EntityEntry ExtractEntity(const GeometryPool& pool, uint32_t header_idx);
    static std::unordered_map<uint32_t, uint32_t> BuildIdIndex(const GeometryPool& pool);

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

    // Build diff from a pid mapping. OCCT-free; directly testable.
    // Producers (e.g. breptopo::TopoNaming::Update) emit a PidMapping
    // covering all sub-shape types, which plugs in directly.
    static PoolDiff BuildDiffFromPidMapping(const GeometryPool& old_pool,
                                             const GeometryPool& new_pool,
                                             const PidMapping&   pid_map);

    static GeometryPool ApplyForward(const GeometryPool& base,    const PoolDiff& diff);
    static GeometryPool ApplyReverse(const GeometryPool& current, const PoolDiff& diff);

    static PoolDiff ComputeDiff(const GeometryPool& old_pool,
                                const GeometryPool& new_pool);

private:
    uint32_t AllocNodeId();
    uint32_t InitRoot(const GeometryPool& pool, const std::string& desc, uint32_t op_type);
    void     NavigateTo(uint32_t target_id);
    uint32_t FindLCA(uint32_t a, uint32_t b) const;

    static GeometryPool RebuildPool(
        const std::unordered_map<uint32_t, EntityEntry>& entities,
        const std::vector<uint32_t>&                     order);

    std::unordered_map<uint32_t, VersionNode> m_nodes;

    uint32_t m_current_id = UINT32_MAX;
    uint32_t m_root_id    = UINT32_MAX;   // primary root
    uint32_t m_next_id    = 0;

    PoolPtr  m_current_pool;  // shared immutable snapshot of the current version

    // Pool snapshots for every root node (needed to navigate across root chains).
    std::unordered_map<uint32_t, PoolPtr> m_root_pools;
};

} // namespace brepdb
