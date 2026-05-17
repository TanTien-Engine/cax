#pragma once

#include "cadcvt/ir/SketchIR.h"

#include <cstdint>
#include <string>
#include <vector>

// ============================================================
// cadcvt/store/SketchStore.h
//
// Binary sketch store, persisted as the "sketch_store" meta page
// of BrepDB.
//
// Design points:
//   1. Kernel independent (only depends on cadcvt/ir/, no OCCT,
//      no sketchlib).
//   2. POD arrays + string pool + double pool, packed, host
//      endian.
//   3. Two build paths: Append(SketchIR) and a streaming
//      BeginSketch / AddGeo / AddCons / EndSketch.
//   4. Round-trip through ImportFromIR / ExportToIR; the editor
//      reaches sketchlib::Scene via SketchBridge.
//
// File layout:
//   [SkFileHeader]
//   [SkSketchHeader[N]]
//   [SkGeoEntry[]]                   flattened across all sketches, offsets
//   [SkConsEntry[]]                  same flattening scheme
//   [double param_pool[]]            SkGeoEntry::param_offset indexes here
//   [char   name_pool[]]             SkSketchHeader::name_offset indexes here
// ============================================================

namespace cadcvt
{

// ---- on-disk PODs; bump version when adding fields ----

#pragma pack(push, 1)

struct SkFileHeader
{
    uint32_t magic;          // 'SKCH' (0x48434B53 little-endian)
    uint32_t version;
    uint32_t sketch_count;
    uint32_t geo_count;
    uint32_t cons_count;
    uint32_t param_pool_len; // count of doubles
    uint32_t name_pool_len;  // count of bytes
    uint32_t reserved;
};

struct SkSketchHeader
{
    uint32_t feature_id;     // matches FeatureStore::FeatureEntry::feature_id
    uint32_t name_offset;    // into name_pool
    uint32_t name_length;
    double   plane_origin[3];
    double   plane_normal[3];
    double   plane_x_dir [3];
    uint32_t geo_offset;     // into geo array
    uint32_t geo_count;
    uint32_t cons_offset;    // into cons array
    uint32_t cons_count;
};

struct SkGeoEntry
{
    uint32_t id;
    uint8_t  type;           // SkGeoType
    uint8_t  construction;   // 0 / 1
    uint16_t reserved;
    uint32_t param_offset;   // into param_pool (doubles)
    uint32_t param_count;    // number of doubles
};

struct SkConsEntry
{
    uint32_t id;
    uint8_t  type;           // SkConsType
    uint8_t  driving;        // 0 / 1
    uint8_t  a_point_pos;    // SkPointPos
    uint8_t  b_point_pos;    // SkPointPos
    uint32_t a_geo_id;       // 0xFFFFFFFF = none
    uint32_t b_geo_id;       // 0xFFFFFFFF = none
    double   value;
};

#pragma pack(pop)

static constexpr uint32_t SKCH_MAGIC   = 0x48434B53; // 'S','K','C','H' little-endian
static constexpr uint32_t SKCH_VERSION = 1;


// ============================================================
// SketchStore
// ============================================================

class SketchStore
{
public:
    SketchStore();

    // ---- Build (high-level IR path, preferred) ----
    // Append a sketch built from a SketchIR; returns the new
    // sketch index.
    uint32_t Append(const SketchIR& sketch);

    // ---- Build (streaming path, mirrors temp SwSketchConverter) ----
    void BeginSketch(uint32_t      feature_id,
                     const char*   name,
                     const double  origin[3],
                     const double  normal[3],
                     const double  x_dir[3]);

    void AddGeometry(uint32_t      geo_id,
                     SkGeoType     type,
                     bool          construction,
                     const double* params,
                     uint32_t      param_count);

    void AddConstraint(uint32_t        cons_id,
                       SkConsType      type,
                       const SkGeoRef& a,
                       const SkGeoRef& b,
                       double          value,
                       bool            driving);

    // Returns the just-committed sketch index.
    uint32_t EndSketch();

    // ---- Query ----
    uint32_t SketchCount() const {
        return (uint32_t)m_sketches.size();
    }

    // sketch_idx in [0, SketchCount()).
    const SkSketchHeader& GetSketch(uint32_t sketch_idx) const {
        return m_sketches[sketch_idx];
    }

    std::string GetSketchName(uint32_t sketch_idx) const;

    int FindByFeatureId(uint32_t feature_id) const;

    // Geometry / constraint slices for a sketch.
    const SkGeoEntry*  GetGeos(uint32_t sketch_idx, uint32_t& count) const;
    const SkConsEntry* GetCons(uint32_t sketch_idx, uint32_t& count) const;
    const double* GetParamPool() const {
        return m_param_pool.data();
    }

    // Reverse direction: rebuild a SketchIR for SketchBridge / VesEmitter.
    bool ExportToIR(uint32_t sketch_idx, SketchIR& out) const;

    // ---- Persistence ----
    void StoreToByteArray(uint8_t** data, uint32_t& len) const;
    bool LoadFromByteArray(const uint8_t* data, uint32_t len);

    void Clear();

private:
    std::vector<SkSketchHeader> m_sketches;
    std::vector<SkGeoEntry>     m_geos;
    std::vector<SkConsEntry>    m_cons;
    std::vector<double>         m_param_pool;
    std::string                 m_name_pool;

    // Streaming builder transient state.
    bool           m_building = false;
    SkSketchHeader m_cur{};
};

} // namespace cadcvt
