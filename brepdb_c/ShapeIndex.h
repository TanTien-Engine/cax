#pragma once

#include <spatialdb/RTree.h>

#include <unordered_map>
#include <vector>

namespace caxdb
{

struct ShapeSlot
{
    spatialdb::id_type  page_id; // R-tree leaf node's storage page
    uint32_t child_idx;       // index within that leaf's children array
};

// Maintains a persistent_id ¡ú R-tree leaf location mapping.
// Hooks into RTree's WriteNode/DeleteNode commands for automatic maintenance.
// Serializes itself into the spatial database's IStorageManager.
class ShapeIndex
{
public:
    ShapeIndex(spatialdb::RTree& tree, const std::shared_ptr<spatialdb::IStorageManager>& sm);
    ~ShapeIndex();

    // Query
    bool     Lookup(spatialdb::id_type persistent_id, ShapeSlot& slot) const;
    bool     Contains(spatialdb::id_type persistent_id) const;
    size_t   Size() const { return m_id_to_slot.size(); }

    // Read data directly from R-tree by persistent_id
    bool GetData(spatialdb::id_type persistent_id, uint32_t& len, uint8_t** data);

    // Persistence: store/load the index itself into the storage manager
    void Store();
    void Load();

    // Full rebuild by scanning all R-tree leaves
    void Rebuild();

private:
    // ICommand implementations for RTree hooks
    class WriteCmd : public spatialdb::ICommand
    {
    public:
        explicit WriteCmd(ShapeIndex& idx) : m_idx(idx) {}
        void Execute(const spatialdb::INode& n) override;
    private:
        ShapeIndex& m_idx;
    };

    class DeleteCmd : public spatialdb::ICommand
    {
    public:
        explicit DeleteCmd(ShapeIndex& idx) : m_idx(idx) {}
        void Execute(const spatialdb::INode& n) override;
    private:
        ShapeIndex& m_idx;
    };

    void OnLeafWritten(const spatialdb::INode& n);
    void OnLeafDeleted(const spatialdb::INode& n);
    void RemoveByPage(spatialdb::id_type page);

private:
    spatialdb::RTree& m_tree;
    std::shared_ptr<spatialdb::IStorageManager> m_sm;
    spatialdb::id_type m_index_page;  // page id where the index is stored

    // Forward: persistent_id ¡ú location
    std::unordered_map<spatialdb::id_type, ShapeSlot> m_id_to_slot;

    // Reverse: page ¡ú list of persistent_ids (for efficient cleanup on split)
    std::unordered_map<spatialdb::id_type, std::vector<spatialdb::id_type>> m_page_to_ids;

    std::shared_ptr<WriteCmd>  m_write_cmd;
    std::shared_ptr<DeleteCmd> m_delete_cmd;

}; // ShapeIndex

} // namespace spatialdb
