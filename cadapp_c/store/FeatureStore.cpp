#include "cadapp_c/store/FeatureStore.h"

#include <cassert>
#include <cstring>

namespace cadapp
{

// ============================================================
// Binary writer / reader primitives
// ============================================================
//
// The encoders write straight into a std::vector<uint8_t>; the
// decoders read from a (ptr, end) pair. All multibyte integers are
// host-endian. The format is internal to FeatureStore so we don't
// bother with portability flags.

namespace
{

class BlobWriter
{
public:
    explicit BlobWriter(std::vector<uint8_t>& sink)
        : m_sink(sink)
    {
    }

    size_t Begin() const {
        return m_sink.size();
    }

    void Bytes(const void* data, size_t n)
    {
        const auto* p = static_cast<const uint8_t*>(data);
        m_sink.insert(m_sink.end(), p, p + n);
    }

    void U8(uint8_t v) {
        m_sink.push_back(v);
    }

    void Bool(bool v) {
        m_sink.push_back(v ? 1 : 0);
    }

    void U32(uint32_t v) {
        Bytes(&v, sizeof(v));
    }

    void I32(int32_t v) {
        Bytes(&v, sizeof(v));
    }

    void F64(double v) {
        Bytes(&v, sizeof(v));
    }

    void Vec3(const double v[3])
    {
        F64(v[0]);
        F64(v[1]);
        F64(v[2]);
    }

    void TopoRef(const TopoRefIR& r)
    {
        U8((uint8_t)r.kind);
        Vec3(r.point);
        Vec3(r.normal);
        I32(r.adj_count);
        F64(r.measure);
        U32(r.resolved_uid);
        I32(r.resolved_topo_index);
    }

    void TopoRefVec(const std::vector<TopoRefIR>& v)
    {
        U32((uint32_t)v.size());
        for (const auto& r : v)
        {
            TopoRef(r);
        }
    }

    void U32Vec(const std::vector<uint32_t>& v)
    {
        U32((uint32_t)v.size());
        for (uint32_t x : v)
        {
            U32(x);
        }
    }

    void StringField(const std::string& s)
    {
        U32((uint32_t)s.size());
        Bytes(s.data(), s.size());
    }

private:
    std::vector<uint8_t>& m_sink;
};


class BlobReader
{
public:
    BlobReader(const uint8_t* begin, const uint8_t* end)
        : m_p(begin)
        , m_end(end)
        , m_ok(true)
    {
    }

    bool ok() const {
        return m_ok;
    }

    bool Bytes(void* dst, size_t n)
    {
        if (m_p + n > m_end)
        {
            m_ok = false;
            return false;
        }
        std::memcpy(dst, m_p, n);
        m_p += n;
        return true;
    }

    uint8_t U8()
    {
        uint8_t v = 0;
        Bytes(&v, 1);
        return v;
    }

    bool Bool()
    {
        return U8() != 0;
    }

    uint32_t U32()
    {
        uint32_t v = 0;
        Bytes(&v, sizeof(v));
        return v;
    }

    int32_t I32()
    {
        int32_t v = 0;
        Bytes(&v, sizeof(v));
        return v;
    }

    double F64()
    {
        double v = 0.0;
        Bytes(&v, sizeof(v));
        return v;
    }

    void Vec3(double out[3])
    {
        out[0] = F64();
        out[1] = F64();
        out[2] = F64();
    }

    TopoRefIR TopoRef()
    {
        TopoRefIR r;
        r.kind = (TopoRefIR::Kind)U8();
        Vec3(r.point);
        Vec3(r.normal);
        r.adj_count            = I32();
        r.measure              = F64();
        r.resolved_uid         = U32();
        r.resolved_topo_index  = I32();
        return r;
    }

    std::vector<TopoRefIR> TopoRefVec()
    {
        std::vector<TopoRefIR> v;
        uint32_t               n = U32();
        v.reserve(n);
        for (uint32_t i = 0; i < n; ++i)
        {
            v.push_back(TopoRef());
        }
        return v;
    }

    std::vector<uint32_t> U32Vec()
    {
        std::vector<uint32_t> v;
        uint32_t              n = U32();
        v.reserve(n);
        for (uint32_t i = 0; i < n; ++i)
        {
            v.push_back(U32());
        }
        return v;
    }

    std::string StringField()
    {
        uint32_t n = U32();
        std::string s;
        s.resize(n);
        if (n > 0) {
            Bytes(s.data(), n);
        }
        return s;
    }

private:
    const uint8_t* m_p;
    const uint8_t* m_end;
    bool           m_ok;
};


// ============================================================
// Encode dispatch: payload tag -> Encode(payload, writer)
// ============================================================

void Encode(const FeatPayloadSketch& p, BlobWriter& w)
{
    w.U32(p.sketch_id);
}

void Encode(const FeatPayloadExtrude& p, BlobWriter& w)
{
    w.U32(p.sketch_id);
    w.Vec3(p.direction);
    w.F64(p.distance);
    w.F64(p.distance2);
    w.U8((uint8_t)p.end_type);
    w.U8((uint8_t)p.end_type2);
    w.Bool(p.flip_direction);
    w.Bool(p.is_thin);
    w.F64(p.thin_thickness);
    w.Bool(p.has_end1_target);
    w.TopoRef(p.end1_target);
    w.Bool(p.has_end2_target);
    w.TopoRef(p.end2_target);
}

void Encode(const FeatPayloadRevolve& p, BlobWriter& w)
{
    w.U32(p.sketch_id);
    w.Vec3(p.axis_origin);
    w.Vec3(p.axis_dir);
    w.F64(p.angle);
    w.F64(p.angle2);
    w.Bool(p.flip_direction);
    w.Bool(p.is_thin);
    w.F64(p.thin_thickness);
}

void Encode(const FeatPayloadLoft& p, BlobWriter& w)
{
    w.U32Vec(p.profile_sketch_ids);
    w.TopoRefVec(p.guide_refs);
    w.Bool(p.closed);
}

void Encode(const FeatPayloadSweep& p, BlobWriter& w)
{
    w.U32(p.profile_sketch_id);
    w.TopoRef(p.path_ref);
    w.Bool(p.twist_along_path);
}

void Encode(const FeatPayloadFillet& p, BlobWriter& w)
{
    w.F64(p.radius);
    w.TopoRefVec(p.edges);
}

void Encode(const FeatPayloadChamfer& p, BlobWriter& w)
{
    w.F64(p.distance1);
    w.F64(p.distance2);
    w.TopoRefVec(p.edges);
}

void Encode(const FeatPayloadShell& p, BlobWriter& w)
{
    w.F64(p.thickness);
    w.TopoRefVec(p.faces_to_open);
    w.Bool(p.shell_outward);
}

void Encode(const FeatPayloadDraft& p, BlobWriter& w)
{
    w.F64(p.angle);
    w.Vec3(p.pull_dir);
    w.TopoRefVec(p.faces);
    w.Bool(p.has_neutral_plane);
    w.TopoRef(p.neutral_plane);
}

void Encode(const FeatPayloadOffset& p, BlobWriter& w)
{
    w.F64(p.distance);
    w.TopoRefVec(p.faces);
}

void Encode(const FeatPayloadTransform& p, BlobWriter& w)
{
    w.Vec3(p.translation);
    w.Vec3(p.axis_origin);
    w.Vec3(p.axis_dir);
    w.F64(p.angle);
    w.Vec3(p.scale);
}

void Encode(const FeatPayloadMirror& p, BlobWriter& w)
{
    w.Vec3(p.plane_origin);
    w.Vec3(p.plane_normal);
}

void Encode(const FeatPayloadLinearPattern& p, BlobWriter& w)
{
    w.Vec3(p.dir1);
    w.I32(p.count1);
    w.F64(p.spacing1);
    w.Vec3(p.dir2);
    w.I32(p.count2);
    w.F64(p.spacing2);
}

void Encode(const FeatPayloadCircularPattern& p, BlobWriter& w)
{
    w.Vec3(p.axis_origin);
    w.Vec3(p.axis_dir);
    w.I32(p.count);
    w.F64(p.total_angle);
}

void Encode(const FeatPayloadBoolean& p, BlobWriter& w)
{
    w.U32Vec(p.operand_feature_ids);
}

void Encode(const FeatPayloadPrimBox& p, BlobWriter& w)
{
    w.F64(p.length);
    w.F64(p.width);
    w.F64(p.height);
}

void Encode(const FeatPayloadPrimCylinder& p, BlobWriter& w)
{
    w.F64(p.radius);
    w.F64(p.height);
}

void Encode(const FeatPayloadPrimCone& p, BlobWriter& w)
{
    w.F64(p.radius1);
    w.F64(p.radius2);
    w.F64(p.height);
}

void Encode(const FeatPayloadPrimSphere& p, BlobWriter& w)
{
    w.F64(p.radius);
}

void Encode(const FeatPayloadPrimTorus& p, BlobWriter& w)
{
    w.F64(p.major_radius);
    w.F64(p.minor_radius);
}

void Encode(const FeatPayloadPrimEllipsoid& p, BlobWriter& w)
{
    w.F64(p.radius1);
    w.F64(p.radius2);
    w.F64(p.radius3);
}

void Encode(const FeatPayloadHoleWizard& p, BlobWriter& w)
{
    w.U32(p.sketch_id);
    w.F64(p.diameter);
    w.F64(p.depth);
    w.Bool(p.through_all);
    w.Bool(p.has_placement_face);
    w.TopoRef(p.placement_face);
}

void Encode(const FeatPayloadRib& p, BlobWriter& w)
{
    w.U32(p.sketch_id);
    w.F64(p.thickness);
    w.Vec3(p.direction);
    w.Bool(p.flip);
}

void Encode(const FeatPayloadOpaque& p, BlobWriter& w)
{
    w.U32((uint32_t)p.params.size());
    for (const auto& kv : p.params)
    {
        w.StringField(kv.first);
        w.F64(kv.second);
    }

    w.U32((uint32_t)p.strings.size());
    for (const auto& kv : p.strings)
    {
        w.StringField(kv.first);
        w.StringField(kv.second);
    }

    w.TopoRefVec(p.edge_refs);
    w.TopoRefVec(p.face_refs);
}

void Encode(const FeatPayloadMultiTransform& p, BlobWriter& w)
{
    w.U32((uint32_t)p.steps.size());
    for (const auto& s : p.steps)
    {
        w.U8((uint8_t)s.kind);
        w.Vec3(s.plane_origin);
        w.Vec3(s.plane_normal);
        w.Vec3(s.dir1);
        w.I32(s.count1);
        w.F64(s.spacing1);
        w.Vec3(s.dir2);
        w.I32(s.count2);
        w.F64(s.spacing2);
        w.Vec3(s.axis_origin);
        w.Vec3(s.axis_dir);
        w.I32(s.count);
        w.F64(s.total_angle);
    }
}


// ============================================================
// Decode dispatch: payload tag -> Decode(reader, payload)
// ============================================================

void Decode(BlobReader& r, FeatPayloadSketch& p)
{
    p.sketch_id = r.U32();
}

void Decode(BlobReader& r, FeatPayloadExtrude& p)
{
    p.sketch_id = r.U32();
    r.Vec3(p.direction);
    p.distance       = r.F64();
    p.distance2      = r.F64();
    p.end_type       = (ExtrudeEndType)r.U8();
    p.end_type2      = (ExtrudeEndType)r.U8();
    p.flip_direction = r.Bool();
    p.is_thin        = r.Bool();
    p.thin_thickness = r.F64();
    p.has_end1_target = r.Bool();
    p.end1_target     = r.TopoRef();
    p.has_end2_target = r.Bool();
    p.end2_target     = r.TopoRef();
}

void Decode(BlobReader& r, FeatPayloadRevolve& p)
{
    p.sketch_id = r.U32();
    r.Vec3(p.axis_origin);
    r.Vec3(p.axis_dir);
    p.angle          = r.F64();
    p.angle2         = r.F64();
    p.flip_direction = r.Bool();
    p.is_thin        = r.Bool();
    p.thin_thickness = r.F64();
}

void Decode(BlobReader& r, FeatPayloadLoft& p)
{
    p.profile_sketch_ids = r.U32Vec();
    p.guide_refs         = r.TopoRefVec();
    p.closed             = r.Bool();
}

void Decode(BlobReader& r, FeatPayloadSweep& p)
{
    p.profile_sketch_id = r.U32();
    p.path_ref          = r.TopoRef();
    p.twist_along_path  = r.Bool();
}

void Decode(BlobReader& r, FeatPayloadFillet& p)
{
    p.radius = r.F64();
    p.edges  = r.TopoRefVec();
}

void Decode(BlobReader& r, FeatPayloadChamfer& p)
{
    p.distance1 = r.F64();
    p.distance2 = r.F64();
    p.edges     = r.TopoRefVec();
}

void Decode(BlobReader& r, FeatPayloadShell& p)
{
    p.thickness     = r.F64();
    p.faces_to_open = r.TopoRefVec();
    p.shell_outward = r.Bool();
}

void Decode(BlobReader& r, FeatPayloadDraft& p)
{
    p.angle = r.F64();
    r.Vec3(p.pull_dir);
    p.faces             = r.TopoRefVec();
    p.has_neutral_plane = r.Bool();
    p.neutral_plane     = r.TopoRef();
}

void Decode(BlobReader& r, FeatPayloadOffset& p)
{
    p.distance = r.F64();
    p.faces    = r.TopoRefVec();
}

void Decode(BlobReader& r, FeatPayloadTransform& p)
{
    r.Vec3(p.translation);
    r.Vec3(p.axis_origin);
    r.Vec3(p.axis_dir);
    p.angle = r.F64();
    r.Vec3(p.scale);
}

void Decode(BlobReader& r, FeatPayloadMirror& p)
{
    r.Vec3(p.plane_origin);
    r.Vec3(p.plane_normal);
}

void Decode(BlobReader& r, FeatPayloadLinearPattern& p)
{
    r.Vec3(p.dir1);
    p.count1   = r.I32();
    p.spacing1 = r.F64();
    r.Vec3(p.dir2);
    p.count2   = r.I32();
    p.spacing2 = r.F64();
}

void Decode(BlobReader& r, FeatPayloadCircularPattern& p)
{
    r.Vec3(p.axis_origin);
    r.Vec3(p.axis_dir);
    p.count       = r.I32();
    p.total_angle = r.F64();
}

void Decode(BlobReader& r, FeatPayloadBoolean& p)
{
    p.operand_feature_ids = r.U32Vec();
}

void Decode(BlobReader& r, FeatPayloadPrimBox& p)
{
    p.length = r.F64();
    p.width  = r.F64();
    p.height = r.F64();
}

void Decode(BlobReader& r, FeatPayloadPrimCylinder& p)
{
    p.radius = r.F64();
    p.height = r.F64();
}

void Decode(BlobReader& r, FeatPayloadPrimCone& p)
{
    p.radius1 = r.F64();
    p.radius2 = r.F64();
    p.height  = r.F64();
}

void Decode(BlobReader& r, FeatPayloadPrimSphere& p)
{
    p.radius = r.F64();
}

void Decode(BlobReader& r, FeatPayloadPrimTorus& p)
{
    p.major_radius = r.F64();
    p.minor_radius = r.F64();
}

void Decode(BlobReader& r, FeatPayloadPrimEllipsoid& p)
{
    p.radius1 = r.F64();
    p.radius2 = r.F64();
    p.radius3 = r.F64();
}

void Decode(BlobReader& r, FeatPayloadHoleWizard& p)
{
    p.sketch_id          = r.U32();
    p.diameter           = r.F64();
    p.depth              = r.F64();
    p.through_all        = r.Bool();
    p.has_placement_face = r.Bool();
    p.placement_face     = r.TopoRef();
}

void Decode(BlobReader& r, FeatPayloadRib& p)
{
    p.sketch_id = r.U32();
    p.thickness = r.F64();
    r.Vec3(p.direction);
    p.flip = r.Bool();
}

void Decode(BlobReader& r, FeatPayloadOpaque& p)
{
    uint32_t n_params = r.U32();
    for (uint32_t i = 0; i < n_params; ++i)
    {
        std::string key = r.StringField();
        double      v   = r.F64();
        p.params.emplace(std::move(key), v);
    }

    uint32_t n_strs = r.U32();
    for (uint32_t i = 0; i < n_strs; ++i)
    {
        std::string key = r.StringField();
        std::string val = r.StringField();
        p.strings.emplace(std::move(key), std::move(val));
    }

    p.edge_refs = r.TopoRefVec();
    p.face_refs = r.TopoRefVec();
}

void Decode(BlobReader& r, FeatPayloadMultiTransform& p)
{
    uint32_t n = r.U32();
    p.steps.resize(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        auto& s = p.steps[i];
        s.kind = (MultiTransformStep::Kind)r.U8();
        r.Vec3(s.plane_origin);
        r.Vec3(s.plane_normal);
        r.Vec3(s.dir1);
        s.count1   = r.I32();
        s.spacing1 = r.F64();
        r.Vec3(s.dir2);
        s.count2   = r.I32();
        s.spacing2 = r.F64();
        r.Vec3(s.axis_origin);
        r.Vec3(s.axis_dir);
        s.count       = r.I32();
        s.total_angle = r.F64();
    }
}


// ============================================================
// Ext bag (FeatureIR::ext_params / ext_strings) packed inside the
// payload byte slice, right after the typed payload. Used by every
// feature kind (Opaque uses its own internal bag, separately).
// ============================================================

void EncodeExt(const FeatureIR& feat, BlobWriter& w)
{
    w.U32((uint32_t)feat.ext_params.size());
    for (const auto& kv : feat.ext_params)
    {
        w.StringField(kv.first);
        w.F64(kv.second);
    }

    w.U32((uint32_t)feat.ext_strings.size());
    for (const auto& kv : feat.ext_strings)
    {
        w.StringField(kv.first);
        w.StringField(kv.second);
    }
}

void DecodeExt(BlobReader& r, FeatureIR& feat)
{
    uint32_t n_params = r.U32();
    for (uint32_t i = 0; i < n_params; ++i)
    {
        std::string key = r.StringField();
        double      v   = r.F64();
        feat.ext_params.emplace(std::move(key), v);
    }

    uint32_t n_strs = r.U32();
    for (uint32_t i = 0; i < n_strs; ++i)
    {
        std::string key = r.StringField();
        std::string val = r.StringField();
        feat.ext_strings.emplace(std::move(key), std::move(val));
    }
}

} // anonymous namespace


// ============================================================
// FeatureStore methods
// ============================================================

FeatureStore::FeatureStore() = default;

uint32_t FeatureStore::Append(const FeatureIR& feat)
{
    FeatureEntry e{};
    e.feature_id  = feat.id;
    e.feat_type   = (uint8_t)feat.type;
    e.payload_tag = (uint8_t)feat.data.index();
    e.suppressed  = feat.suppressed ? 1u : 0u;
    e.name_offset = (uint32_t)m_name_pool.size();
    e.name_length = (uint32_t)feat.name.size();
    if (e.name_length > 0) {
        m_name_pool.append(feat.name);
    }

    e.payload_offset = (uint32_t)m_payload_pool.size();

    BlobWriter w(m_payload_pool);
    std::visit([&](const auto& pl) { Encode(pl, w); }, feat.data);
    EncodeExt(feat, w);

    e.payload_length = (uint32_t)m_payload_pool.size() - e.payload_offset;

    uint32_t idx = (uint32_t)m_entries.size();
    m_entries.push_back(e);
    return idx;
}

std::string FeatureStore::GetName(uint32_t entry_idx) const
{
    const auto& e = m_entries[entry_idx];
    if (e.name_length == 0) {
        return {};
    }
    return m_name_pool.substr(e.name_offset, e.name_length);
}

int FeatureStore::FindByFeatureId(uint32_t feature_id) const
{
    for (uint32_t i = 0; i < m_entries.size(); ++i)
    {
        if (m_entries[i].feature_id == feature_id) {
            return (int)i;
        }
    }
    return -1;
}

bool FeatureStore::ExportToIR(uint32_t entry_idx, FeatureIR& out) const
{
    if (entry_idx >= m_entries.size()) {
        return false;
    }

    const auto& e = m_entries[entry_idx];

    out            = FeatureIR{};
    out.id         = e.feature_id;
    out.type       = (FeatType)e.feat_type;
    out.suppressed = (e.suppressed != 0);
    out.name       = GetName(entry_idx);

    const uint8_t* base  = m_payload_pool.data() + e.payload_offset;
    const uint8_t* end   = base + e.payload_length;
    BlobReader     r(base, end);

    // The variant alternative ordering MUST match FeaturePayload
    // in FeatureIR.h. switch on payload_tag and decode into the
    // proper alternative.
    switch (e.payload_tag)
    {
    case 0:
    {
        FeatPayloadSketch p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 1:
    {
        FeatPayloadExtrude p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 2:
    {
        FeatPayloadRevolve p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 3:
    {
        FeatPayloadLoft p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 4:
    {
        FeatPayloadSweep p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 5:
    {
        FeatPayloadFillet p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 6:
    {
        FeatPayloadChamfer p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 7:
    {
        FeatPayloadShell p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 8:
    {
        FeatPayloadDraft p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 9:
    {
        FeatPayloadOffset p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 10:
    {
        FeatPayloadTransform p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 11:
    {
        FeatPayloadMirror p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 12:
    {
        FeatPayloadLinearPattern p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 13:
    {
        FeatPayloadCircularPattern p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 14:
    {
        FeatPayloadBoolean p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 15:
    {
        FeatPayloadPrimBox p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 16:
    {
        FeatPayloadPrimCylinder p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 17:
    {
        FeatPayloadPrimCone p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 18:
    {
        FeatPayloadPrimSphere p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 19:
    {
        FeatPayloadPrimTorus p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 20:
    {
        FeatPayloadHoleWizard p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 21:
    {
        FeatPayloadRib p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 22:
    {
        FeatPayloadOpaque p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 23:
    {
        FeatPayloadMultiTransform p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    case 24:
    {
        FeatPayloadPrimEllipsoid p;
        Decode(r, p);
        out.data = std::move(p);
        break;
    }
    default:
        return false;
    }

    DecodeExt(r, out);
    return r.ok();
}

bool FeatureStore::ReplaceFeature(uint32_t entry_idx, const FeatureIR& feat)
{
    // Rebuild the payload bytes; entries layout itself is rewritten
    // by rewriting m_payload_pool from scratch, which is O(N) but
    // safe (TopoRef write-back happens once at end of replay).
    if (entry_idx >= m_entries.size()) {
        return false;
    }

    std::vector<FeatureIR> all;
    all.reserve(m_entries.size());
    for (uint32_t i = 0; i < m_entries.size(); ++i)
    {
        FeatureIR f;
        if (i == entry_idx)
        {
            f = feat;
        }
        else
        {
            if (!ExportToIR(i, f)) {
                return false;
            }
        }
        all.push_back(std::move(f));
    }

    Clear();
    for (const auto& f : all)
    {
        Append(f);
    }
    return true;
}

// ============================================================
// Serialization
// ============================================================

void FeatureStore::StoreToByteArray(uint8_t** data, uint32_t& len) const
{
    FeatFileHeader hdr{};
    hdr.magic         = FEAT_MAGIC;
    hdr.version       = FEAT_VERSION;
    hdr.entry_count   = (uint32_t)m_entries.size();
    hdr.payload_bytes = (uint32_t)m_payload_pool.size();
    hdr.name_pool_len = (uint32_t)m_name_pool.size();

    uint32_t total = sizeof(FeatFileHeader)
                   + hdr.entry_count * sizeof(FeatureEntry)
                   + hdr.payload_bytes
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
    if (hdr.entry_count) {
        W(m_entries.data(), hdr.entry_count * sizeof(FeatureEntry));
    }
    if (hdr.payload_bytes) {
        W(m_payload_pool.data(), hdr.payload_bytes);
    }
    if (hdr.name_pool_len) {
        W(m_name_pool.data(), hdr.name_pool_len);
    }
}

bool FeatureStore::LoadFromByteArray(const uint8_t* data, uint32_t len)
{
    Clear();
    if (len < sizeof(FeatFileHeader) || !data) {
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

    FeatFileHeader hdr{};
    if (!R(&hdr, sizeof(hdr))) {
        return false;
    }
    if (hdr.magic != FEAT_MAGIC || hdr.version != FEAT_VERSION) {
        return false;
    }

    m_entries.resize(hdr.entry_count);
    if (hdr.entry_count &&
        !R(m_entries.data(), hdr.entry_count * sizeof(FeatureEntry)))
    {
        return false;
    }

    m_payload_pool.resize(hdr.payload_bytes);
    if (hdr.payload_bytes &&
        !R(m_payload_pool.data(), hdr.payload_bytes))
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

void FeatureStore::Clear()
{
    m_entries.clear();
    m_payload_pool.clear();
    m_name_pool.clear();
}

} // namespace cadapp
