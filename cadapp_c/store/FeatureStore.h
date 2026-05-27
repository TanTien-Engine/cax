#pragma once

#include "cadapp_c/ir/FeatureIR.h"

#include <cstdint>
#include <string>
#include <vector>

// ============================================================
// cadapp/store/FeatureStore.h
//
// Binary feature chain store. Persisted as the "feature_store"
// meta page of BrepDB.
//
// Layout v2:
//
//   [FeatFileHeader]
//   [FeatureEntry[N]]      fixed-size record per feature
//   [byte payload_pool[]]  variant payload bytes (variable)
//   [char name_pool[]]
//
// Each payload bytes block is a per-type encoding written by
// EncodePayload<X> / read by DecodePayload<X> in the cpp. Any
// variable-sized data (TopoRefIR vectors, ext_params, sketch_id
// lists) is inlined as [u32 count][records...] within those bytes
// so a feature's payload is one contiguous slice.
//
// Cross-store relationships:
//   payload's sketch_id field (Extrude / Revolve / ...) matches
//     SkSketchHeader::feature_id.
//   FeatureEntry::feature_id <-> VersionTree node_id once committed.
//
// Versioning:
//   FEAT_VERSION = 1. The variant tag (payload_tag) is the index
//   into FeaturePayload, so appending to that variant is fine but
//   reordering is not. Bump FEAT_VERSION on layout break.
//
//   Bumps during the multi-last_node refactor (P3.1 added typed
//   input_feature_ids, P3.3.A added input_roles, P3.3.B emptied
//   FeatPayloadBoolean) were rolled back to V1 here because no
//   saved store ever existed in the intermediate formats -- the
//   only consumer regenerates DocumentIR from .FCStd on every
//   load, so there was nothing to migrate.
// ============================================================

namespace cadapp
{

#pragma pack(push, 1)

struct FeatFileHeader
{
    uint32_t magic;             // 'FEAT' (0x54414546 little-endian)
    uint32_t version;
    uint32_t entry_count;
    uint32_t payload_bytes;
    uint32_t name_pool_len;
    uint32_t reserved0;
    uint32_t reserved1;
    uint32_t reserved2;
};

struct FeatureEntry
{
    uint32_t feature_id;        // global id, kept in sync with VersionTree
    uint8_t  feat_type;         // FeatType enum
    uint8_t  payload_tag;       // FeaturePayload::index()
    uint8_t  suppressed;
    uint8_t  reserved;

    uint32_t name_offset;       // into name_pool
    uint32_t name_length;

    uint32_t payload_offset;    // into payload_pool (bytes)
    uint32_t payload_length;
};

#pragma pack(pop)

static constexpr uint32_t FEAT_MAGIC   = 0x54414546;   // 'F','E','A','T' little-endian
static constexpr uint32_t FEAT_VERSION = 1;


// ============================================================
// FeatureStore
// ============================================================

class FeatureStore

{
public:
    FeatureStore();

    // ---- Build ----
    // Append a FeatureIR; returns the entry index.
    uint32_t Append(const FeatureIR& feat);

    // ---- Query ----
    uint32_t FeatureCount() const {
        return (uint32_t)m_entries.size();
    }

    const FeatureEntry& GetEntry(uint32_t entry_idx) const {
        return m_entries[entry_idx];
    }

    std::string GetName(uint32_t entry_idx) const;

    int FindByFeatureId(uint32_t feature_id) const;

    // Decode the full FeatureIR including payload + ext bag.
    bool ExportToIR(uint32_t entry_idx, FeatureIR& out) const;

    // ---- Write-back ----
    // After Replayer resolves TopoRefIRs inside a payload, push
    // the mutated FeatureIR back into the store. Cheaper than
    // re-Append + re-link because feature_id / index stay stable.
    bool ReplaceFeature(uint32_t entry_idx, const FeatureIR& feat);

    // ---- Persistence ----
    void StoreToByteArray(uint8_t** data, uint32_t& len) const;
    bool LoadFromByteArray(const uint8_t* data, uint32_t len);

    void Clear();

private:
    std::vector<FeatureEntry> m_entries;
    std::vector<uint8_t>      m_payload_pool;
    std::string               m_name_pool;
};

} // namespace cadapp
