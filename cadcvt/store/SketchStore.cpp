#include "cadcvt/store/SketchStore.h"

#include <cassert>
#include <cstring>

namespace cadcvt
{

SketchStore::SketchStore() = default;

// ============================================================
// Build: high-level append
// ============================================================

uint32_t SketchStore::Append(const SketchIR& sketch)
{
    BeginSketch(sketch.feature_id,
                sketch.name.c_str(),
                sketch.plane_origin,
                sketch.plane_normal,
                sketch.plane_x_dir);

    for (const auto& g : sketch.geos)
    {
        AddGeometry(g.id,
                    g.type,
                    g.construction,
                    g.params.data(),
                    (uint32_t)g.params.size());
    }

    for (const auto& c : sketch.cons) {
        AddConstraint(c.id, c.type, c.a, c.b, c.value, c.driving);
    }

    return EndSketch();
}

// ============================================================
// Build: streaming
// ============================================================

void SketchStore::BeginSketch(uint32_t     feature_id,
                              const char*  name,
                              const double origin[3],
                              const double normal[3],
                              const double x_dir[3])
{
    assert(!m_building);
    m_building = true;

    m_cur = SkSketchHeader{};
    m_cur.feature_id  = feature_id;
    m_cur.name_offset = (uint32_t)m_name_pool.size();
    m_cur.name_length = name ? (uint32_t)std::strlen(name) : 0;
    if (m_cur.name_length > 0) {
        m_name_pool.append(name, m_cur.name_length);
    }

    for (int i = 0; i < 3; ++i)
    {
        m_cur.plane_origin[i] = origin ? origin[i] : 0.0;
        m_cur.plane_normal[i] = normal ? normal[i] : (i == 2 ? 1.0 : 0.0);
        m_cur.plane_x_dir [i] = x_dir  ? x_dir [i] : (i == 0 ? 1.0 : 0.0);
    }

    m_cur.geo_offset  = (uint32_t)m_geos.size();
    m_cur.geo_count   = 0;
    m_cur.cons_offset = (uint32_t)m_cons.size();
    m_cur.cons_count  = 0;
}

void SketchStore::AddGeometry(uint32_t      geo_id,
                              SkGeoType     type,
                              bool          construction,
                              const double* params,
                              uint32_t      param_count)
{
    assert(m_building);

    SkGeoEntry e{};
    e.id           = geo_id;
    e.type         = (uint8_t)type;
    e.construction = construction ? 1u : 0u;
    e.param_offset = (uint32_t)m_param_pool.size();
    e.param_count  = param_count;
    if (param_count > 0 && params) {
        m_param_pool.insert(m_param_pool.end(), params, params + param_count);
    }

    m_geos.push_back(e);
    m_cur.geo_count++;
}

void SketchStore::AddConstraint(uint32_t        cons_id,
                                SkConsType      type,
                                const SkGeoRef& a,
                                const SkGeoRef& b,
                                double          value,
                                bool            driving)
{
    assert(m_building);

    SkConsEntry e{};
    e.id          = cons_id;
    e.type        = (uint8_t)type;
    e.driving     = driving ? 1u : 0u;
    e.a_point_pos = (uint8_t)a.point_pos;
    e.b_point_pos = (uint8_t)b.point_pos;
    e.a_geo_id    = a.geo_id;
    e.b_geo_id    = b.geo_id;
    e.value       = value;

    m_cons.push_back(e);
    m_cur.cons_count++;
}

uint32_t SketchStore::EndSketch()
{
    assert(m_building);
    m_building = false;

    uint32_t idx = (uint32_t)m_sketches.size();
    m_sketches.push_back(m_cur);
    return idx;
}

// ============================================================
// Query
// ============================================================

std::string SketchStore::GetSketchName(uint32_t sketch_idx) const
{
    const auto& h = m_sketches[sketch_idx];
    if (h.name_length == 0) {
        return {};
    }
    return m_name_pool.substr(h.name_offset, h.name_length);
}

int SketchStore::FindByFeatureId(uint32_t feature_id) const
{
    for (uint32_t i = 0; i < m_sketches.size(); ++i)
    {
        if (m_sketches[i].feature_id == feature_id) {
            return (int)i;
        }
    }
    return -1;
}

const SkGeoEntry* SketchStore::GetGeos(uint32_t sketch_idx, uint32_t& count) const
{
    const auto& h = m_sketches[sketch_idx];
    count = h.geo_count;
    return h.geo_count > 0 ? &m_geos[h.geo_offset] : nullptr;
}

const SkConsEntry* SketchStore::GetCons(uint32_t sketch_idx, uint32_t& count) const
{
    const auto& h = m_sketches[sketch_idx];
    count = h.cons_count;
    return h.cons_count > 0 ? &m_cons[h.cons_offset] : nullptr;
}

bool SketchStore::ExportToIR(uint32_t sketch_idx, SketchIR& out) const
{
    if (sketch_idx >= m_sketches.size()) {
        return false;
    }
    const auto& h = m_sketches[sketch_idx];

    out.feature_id = h.feature_id;
    out.name       = GetSketchName(sketch_idx);

    for (int i = 0; i < 3; ++i)
    {
        out.plane_origin[i] = h.plane_origin[i];
        out.plane_normal[i] = h.plane_normal[i];
        out.plane_x_dir [i] = h.plane_x_dir [i];
    }

    out.geos.clear();
    out.geos.reserve(h.geo_count);
    for (uint32_t i = 0; i < h.geo_count; ++i)
    {
        const auto& g = m_geos[h.geo_offset + i];
        SkGeoIR gi;
        gi.id           = g.id;
        gi.type         = (SkGeoType)g.type;
        gi.construction = (g.construction != 0);
        if (g.param_count > 0)
        {
            gi.params.assign(
                m_param_pool.data() + g.param_offset,
                m_param_pool.data() + g.param_offset + g.param_count);
        }
        out.geos.push_back(std::move(gi));
    }

    out.cons.clear();
    out.cons.reserve(h.cons_count);
    for (uint32_t i = 0; i < h.cons_count; ++i)
    {
        const auto& c = m_cons[h.cons_offset + i];
        SkConsIR ci;
        ci.id           = c.id;
        ci.type         = (SkConsType)c.type;
        ci.a.geo_id     = c.a_geo_id;
        ci.a.point_pos  = (SkPointPos)c.a_point_pos;
        ci.b.geo_id     = c.b_geo_id;
        ci.b.point_pos  = (SkPointPos)c.b_point_pos;
        ci.value        = c.value;
        ci.driving      = (c.driving != 0);
        out.cons.push_back(ci);
    }

    return true;
}

// ============================================================
// Serialization
// ============================================================

void SketchStore::StoreToByteArray(uint8_t** data, uint32_t& len) const
{
    SkFileHeader hdr{};
    hdr.magic          = SKCH_MAGIC;
    hdr.version        = SKCH_VERSION;
    hdr.sketch_count   = (uint32_t)m_sketches.size();
    hdr.geo_count      = (uint32_t)m_geos.size();
    hdr.cons_count     = (uint32_t)m_cons.size();
    hdr.param_pool_len = (uint32_t)m_param_pool.size();
    hdr.name_pool_len  = (uint32_t)m_name_pool.size();

    uint32_t total = sizeof(SkFileHeader)
                   + hdr.sketch_count   * sizeof(SkSketchHeader)
                   + hdr.geo_count      * sizeof(SkGeoEntry)
                   + hdr.cons_count     * sizeof(SkConsEntry)
                   + hdr.param_pool_len * (uint32_t)sizeof(double)
                   + hdr.name_pool_len;

    *data = new uint8_t[total];
    len   = total;

    uint8_t* p = *data;
    auto W = [&](const void* src, size_t sz)
    {
        std::memcpy(p, src, sz);
        p += sz;
    };

    W(&hdr, sizeof(hdr));
    if (hdr.sketch_count) {
        W(m_sketches.data(), hdr.sketch_count * sizeof(SkSketchHeader));
    }
    if (hdr.geo_count) {
        W(m_geos.data(), hdr.geo_count * sizeof(SkGeoEntry));
    }
    if (hdr.cons_count) {
        W(m_cons.data(), hdr.cons_count * sizeof(SkConsEntry));
    }
    if (hdr.param_pool_len) {
        W(m_param_pool.data(), hdr.param_pool_len * sizeof(double));
    }
    if (hdr.name_pool_len) {
        W(m_name_pool.data(), hdr.name_pool_len);
    }
}

bool SketchStore::LoadFromByteArray(const uint8_t* data, uint32_t len)
{
    Clear();
    if (len < sizeof(SkFileHeader) || !data) {
        return false;
    }

    const uint8_t* p   = data;
    const uint8_t* end = data + len;

    auto R = [&](void* dst, size_t sz) -> bool
    {
        if (p + sz > end) {
            return false;
        }
        std::memcpy(dst, p, sz);
        p += sz;
        return true;
    };

    SkFileHeader hdr{};
    if (!R(&hdr, sizeof(hdr))) {
        return false;
    }
    if (hdr.magic != SKCH_MAGIC || hdr.version != SKCH_VERSION) {
        return false;
    }

    m_sketches.resize(hdr.sketch_count);
    if (hdr.sketch_count &&
        !R(m_sketches.data(), hdr.sketch_count * sizeof(SkSketchHeader)))
    {
        return false;
    }

    m_geos.resize(hdr.geo_count);
    if (hdr.geo_count &&
        !R(m_geos.data(), hdr.geo_count * sizeof(SkGeoEntry)))
    {
        return false;
    }

    m_cons.resize(hdr.cons_count);
    if (hdr.cons_count &&
        !R(m_cons.data(), hdr.cons_count * sizeof(SkConsEntry)))
    {
        return false;
    }

    m_param_pool.resize(hdr.param_pool_len);
    if (hdr.param_pool_len &&
        !R(m_param_pool.data(), hdr.param_pool_len * sizeof(double)))
    {
        return false;
    }

    m_name_pool.resize(hdr.name_pool_len);
    if (hdr.name_pool_len &&
        !R(m_name_pool.data(), hdr.name_pool_len))
    {
        return false;
    }

    return true;
}

void SketchStore::Clear()
{
    m_sketches.clear();
    m_geos.clear();
    m_cons.clear();
    m_param_pool.clear();
    m_name_pool.clear();
    m_building = false;
    m_cur = SkSketchHeader{};
}

} // namespace cadcvt
