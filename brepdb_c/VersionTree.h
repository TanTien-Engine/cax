#pragma once

#include "ComponentDiff.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace brepdb
{

struct VersionNode
{
    uint32_t id        = 0;
    uint32_t parent_id = UINT32_MAX;

    std::vector<uint32_t> aux_parent_ids;
    std::vector<uint32_t> children;

    std::string op_desc;
    uint32_t    op_type   = 0;
    uint64_t    timestamp = 0;

    ComponentDiff diff;
};

struct RootCursor
{
    uint32_t current_id = UINT32_MAX;
    WorldPtr current_world;
};

class VersionTree
{
public:
    using PidMapping = ComponentDiff::PidMapping;

    VersionTree();

    uint32_t AddRoot(const BRepWorld&   world,
                     const std::string& op_desc,
                     uint32_t           op_type = 0);

    uint32_t Commit(uint32_t            root_id,
                    const BRepWorld&    new_world,
                    ComponentDiff&&     diff,
                    const std::string&  op_desc,
                    uint32_t            op_type = 0);

    uint32_t Commit(uint32_t            root_id,
                    const BRepWorld&    new_world,
                    const std::string&  op_desc,
                    uint32_t            op_type = 0);

    uint32_t Commit(uint32_t            root_id,
                    const BRepWorld&    new_world,
                    const PidMapping&   pid_map,
                    const std::string&  op_desc,
                    uint32_t            op_type = 0);

    uint32_t Branch(uint32_t            parent_id,
                    const BRepWorld&    new_world,
                    ComponentDiff&&     diff,
                    const std::string&  op_desc,
                    uint32_t            op_type = 0);

    uint32_t Merge(uint32_t                       primary_parent_id,
                   const std::vector<uint32_t>&   aux_parent_ids,
                   const BRepWorld&               new_world,
                   ComponentDiff&&                diff,
                   const std::string&             op_desc,
                   uint32_t                       op_type = 0);

    uint32_t Merge(uint32_t                       primary_parent_id,
                   const std::vector<uint32_t>&   aux_parent_ids,
                   const BRepWorld&               new_world,
                   const PidMapping&              pid_map,
                   const std::string&             op_desc,
                   uint32_t                       op_type = 0);

    // ----- Navigation (per root) -----

    WorldPtr Checkout(uint32_t root_id, uint32_t node_id);
    WorldPtr Undo(uint32_t root_id);
    WorldPtr Redo(uint32_t root_id, int child_index = -1);

    WorldPtr GetCurrentWorld(uint32_t root_id) const;
    uint32_t GetCurrentId(uint32_t root_id) const;
    bool     CanUndo(uint32_t root_id) const;
    bool     CanRedo(uint32_t root_id) const;

    // ----- Query -----

    uint32_t GetRootId() const { return m_root_id; }

    std::vector<uint32_t> GetRoots() const;
    uint32_t FindRootOf(uint32_t node_id) const;
    std::vector<uint32_t> GetAllCurrentIds() const;

    const RootCursor* GetCursor(uint32_t root_id) const;

    const VersionNode* GetNode(uint32_t id) const;
    size_t             GetNodeCount() const { return m_nodes.size(); }

    std::vector<uint32_t> GetPathFromRoot(uint32_t node_id) const;
    std::vector<uint32_t> GetLeaves() const;

    void TraverseAll(const std::function<void(const VersionNode&)>& visitor) const;

    // ----- Persistence -----

    bool SaveToFile(const std::string& filepath) const;
    bool LoadFromFile(const std::string& filepath);

    void StoreToByteArray(uint8_t** buf, uint32_t& len) const;
    void LoadFromByteArray(const uint8_t* buf, uint32_t len);

    void Clear();

private:
    uint32_t AllocNodeId();
    void     NavigateTo(uint32_t root_id, uint32_t target_id);
    uint32_t FindLCA(uint32_t a, uint32_t b) const;

    std::unordered_map<uint32_t, VersionNode> m_nodes;
    std::unordered_map<uint32_t, RootCursor>  m_cursors;

    uint32_t m_root_id = UINT32_MAX;
    uint32_t m_next_id = 0;

    std::unordered_map<uint32_t, WorldPtr> m_root_worlds;
};

} // namespace brepdb
