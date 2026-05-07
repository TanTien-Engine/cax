#include "brepdb_c/ShapeIndex.h"

#include <spatialdb/Node.h>
#include <spatialdb/Leaf.h>
#include <spatialdb/Exception.h>

#include <cstring>
#include <assert.h>

namespace brepdb
{

//
// Serialization format (stored as a single byte array in IStorageManager):
//
//   [magic:   uint32_t]  0x53494458  ("SIDX")
//   [version: uint32_t]  1
//   [count:   uint32_t]  number of entries
//   [entries: count * Entry]
//
//   Each Entry:
//     [persistent_id: spatialdb::id_type  ]
//     [page_id:       spatialdb::id_type  ]
//     [child_idx:     uint32_t ]
//

static const uint32_t SIDX_MAGIC   = 0x53494458;
static const uint32_t SIDX_VERSION = 1;

struct SerializedEntry
{
    spatialdb::id_type  persistent_id;
    spatialdb::id_type  page_id;
    uint32_t child_idx;
};

// ============================================================
// Construction / Destruction
// ============================================================

ShapeIndex::ShapeIndex(spatialdb::RTree& tree, const std::shared_ptr<spatialdb::IStorageManager>& sm)
    : m_tree(tree)
    , m_sm(sm)
{
    m_write_cmd  = std::make_shared<WriteCmd>(*this);
    m_delete_cmd = std::make_shared<DeleteCmd>(*this);

    tree.AddCommand(m_write_cmd, spatialdb::CommandType::Write);
    tree.AddCommand(m_delete_cmd, spatialdb::CommandType::Delete);
}

ShapeIndex::~ShapeIndex()
{
}

// ============================================================
// Query
// ============================================================

bool ShapeIndex::Lookup(spatialdb::id_type persistent_id, ShapeSlot& slot) const
{
    auto it = m_id_to_slot.find(persistent_id);
    if (it == m_id_to_slot.end()) {
        return false;
    }
    slot = it->second;
    return true;
}

bool ShapeIndex::Contains(spatialdb::id_type persistent_id) const
{
    return m_id_to_slot.count(persistent_id) > 0;
}

//// fix
//bool ShapeIndex::GetData(spatialdb::id_type persistent_id, uint32_t& len, uint8_t** data)
//{
//    ShapeSlot slot;
//    if (!Lookup(persistent_id, slot)) {
//        return false;
//    }
//
//    std::shared_ptr<spatialdb::Node> node = m_tree.ReadNode(slot.page_id);
//    if (!node->IsLeaf()) {
//        return false;
//    }
//
//    // Validate: the child at this index should match our persistent_id
//    if (slot.child_idx < node->GetChildrenCount() &&
//        node->GetChildIdentifier(slot.child_idx) == persistent_id)
//    {
//        node->GetChildData(slot.child_idx, len, data);
//        return true;
//    }
//
//    // Index is stale for this entry, linear search the node
//    for (uint32_t i = 0; i < node->GetChildrenCount(); ++i)
//    {
//        if (node->GetChildIdentifier(i) == persistent_id)
//        {
//            node->GetChildData(i, len, data);
//            // Fix the stale entry
//            m_id_to_slot[persistent_id] = ShapeSlot{ slot.page_id, i };
//            return true;
//        }
//    }
//
//    // Entry no longer in this node ¡ª remove stale mapping
//    m_id_to_slot.erase(persistent_id);
//    return false;
//}

bool ShapeIndex::GetData(spatialdb::id_type persistent_id, uint32_t& len, uint8_t** data)
{
    ShapeSlot slot;
    if (!Lookup(persistent_id, slot)) {
        return false;
    }

    std::shared_ptr<spatialdb::Node> node = m_tree.ReadNode(slot.page_id);
    assert(node->IsLeaf());
    assert(slot.child_idx < node->GetChildrenCount());
    assert(node->GetChildIdentifier(slot.child_idx) == persistent_id);

    node->GetChildData(slot.child_idx, len, data);
    return true;
}

// ============================================================
// Persistence
// ============================================================

void ShapeIndex::Store(spatialdb::id_type& page)
{
    const uint32_t count = static_cast<uint32_t>(m_id_to_slot.size());

    // Header: magic + version + count
    const uint32_t header_size = sizeof(uint32_t) * 3;
    const uint32_t entry_size  = sizeof(SerializedEntry);
    const uint32_t total       = header_size + count * entry_size;

    uint8_t* buf = new uint8_t[total];
    uint8_t* ptr = buf;

    // Write header
    uint32_t magic = SIDX_MAGIC;
    memcpy(ptr, &magic, sizeof(uint32_t));
    ptr += sizeof(uint32_t);

    uint32_t version = SIDX_VERSION;
    memcpy(ptr, &version, sizeof(uint32_t));
    ptr += sizeof(uint32_t);

    memcpy(ptr, &count, sizeof(uint32_t));
    ptr += sizeof(uint32_t);

    // Write entries
    for (const auto& pair : m_id_to_slot)
    {
        SerializedEntry entry;
        entry.persistent_id = pair.first;
        entry.page_id       = pair.second.page_id;
        entry.child_idx     = pair.second.child_idx;
        memcpy(ptr, &entry, entry_size);
        ptr += entry_size;
    }

    assert(ptr - buf == total);

    try
    {
        m_sm->StoreByteArray(page, total, buf);
    }
    catch (...)
    {
        delete[] buf;
        throw;
    }

    delete[] buf;
}

void ShapeIndex::Load(spatialdb::id_type page)
{
    uint32_t len = 0;
    uint8_t* buf = nullptr;

    try {
        m_sm->LoadByteArray(page, len, &buf);
    } catch (spatialdb::InvalidPageException&) {
        return;  // No index stored yet
    }

    if (len < sizeof(uint32_t) * 3)
    {
        delete[] buf;
        return;
    }

    uint8_t* ptr = buf;

    // Read header
    uint32_t magic;
    memcpy(&magic, ptr, sizeof(uint32_t));
    ptr += sizeof(uint32_t);

    if (magic != SIDX_MAGIC)
    {
        delete[] buf;
        return;
    }

    uint32_t version;
    memcpy(&version, ptr, sizeof(uint32_t));
    ptr += sizeof(uint32_t);

    uint32_t count;
    memcpy(&count, ptr, sizeof(uint32_t));
    ptr += sizeof(uint32_t);

    const uint32_t entry_size = sizeof(SerializedEntry);
    const uint32_t expected = sizeof(uint32_t) * 3 + count * entry_size;
    if (len < expected)
    {
        delete[] buf;
        return;
    }

    // Read entries
    m_id_to_slot.clear();
    m_page_to_ids.clear();
    m_id_to_slot.reserve(count);

    for (uint32_t i = 0; i < count; ++i)
    {
        SerializedEntry entry;
        memcpy(&entry, ptr, entry_size);
        ptr += entry_size;

        ShapeSlot slot{ entry.page_id, entry.child_idx };
        m_id_to_slot[entry.persistent_id] = slot;
        m_page_to_ids[entry.page_id].push_back(entry.persistent_id);
    }

    delete[] buf;
}

// ============================================================
// Full rebuild by scanning R-tree leaves
// ============================================================

void ShapeIndex::Rebuild()
{
    m_id_to_slot.clear();
    m_page_to_ids.clear();

    class LeafScanner : public spatialdb::IVisitor
    {
    public:
        ShapeIndex& idx;
        LeafScanner(ShapeIndex& i) : idx(i) {}

        spatialdb::VisitorStatus VisitNode(const spatialdb::INode& n) override
        {
            if (n.IsLeaf())
            {
                spatialdb::id_type page = n.GetIdentifier();
                auto& ids = idx.m_page_to_ids[page];

                for (uint32_t i = 0; i < n.GetChildrenCount(); ++i)
                {
                    spatialdb::id_type sid = n.GetChildIdentifier(i);
                    idx.m_id_to_slot[sid] = ShapeSlot{ page, i };
                    ids.push_back(sid);
                }
            }
            return spatialdb::VisitorStatus::Continue;
        }

        void VisitData(const spatialdb::IData& d) override {}
        void VisitData(std::vector<const spatialdb::IData*>& v) override {}
    };

    LeafScanner scanner(*this);
    m_tree.LevelTraversal(scanner);
}

// ============================================================
// Hook implementations
// ============================================================

void ShapeIndex::WriteCmd::Execute(const spatialdb::INode& n)
{
    m_idx.OnLeafWritten(n);
}

void ShapeIndex::DeleteCmd::Execute(const spatialdb::INode& n)
{
    m_idx.OnLeafDeleted(n);
}

void ShapeIndex::OnLeafWritten(const spatialdb::INode& n)
{
    if (!n.IsLeaf()) 
        return;

    spatialdb::id_type page = n.GetIdentifier();
    if (page < 0) 
        return;

    // Clear old mappings for this page
    RemoveByPage(page);

    // Re-register all children
    auto& ids = m_page_to_ids[page];
    for (uint32_t i = 0; i < n.GetChildrenCount(); ++i)
    {
        spatialdb::id_type sid = n.GetChildIdentifier(i);
        m_id_to_slot[sid] = ShapeSlot{ page, i };
        ids.push_back(sid);
    }
}

void ShapeIndex::OnLeafDeleted(const spatialdb::INode& n)
{
    if (!n.IsLeaf()) 
        return;
    RemoveByPage(n.GetIdentifier());
}

void ShapeIndex::RemoveByPage(spatialdb::id_type page)
{
    auto it = m_page_to_ids.find(page);
    if (it == m_page_to_ids.end()) 
        return;

    for (spatialdb::id_type sid : it->second) {
        m_id_to_slot.erase(sid);
    }
    it->second.clear();
}

}
