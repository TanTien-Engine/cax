#include "VersionTree.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstring>
#include <fstream>
#include <queue>
#include <set>

// ============================================================
// File-local helpers
// ============================================================
namespace
{

constexpr uint32_t FILE_MAGIC   = 0x56544244; // "VTBD"
constexpr uint32_t FILE_VERSION = 1;

struct FileHeader
{
    uint32_t magic;
    uint32_t version;
    uint32_t node_count;
    uint32_t root_id;
    uint32_t cursor_count;
    uint32_t reserved0;
    uint32_t reserved1;
};

uint64_t NowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(
        system_clock::now().time_since_epoch()).count();
}

// ---- stream write helpers ----

void Wr32(std::ofstream& os, uint32_t v) {
    os.write(reinterpret_cast<const char*>(&v), sizeof(v));
}

void Wr64(std::ofstream& os, uint64_t v) {
    os.write(reinterpret_cast<const char*>(&v), sizeof(v));
}

void WrF64(std::ofstream& os, const double* data, size_t count) {
    if (count > 0)
        os.write(reinterpret_cast<const char*>(data),
                 static_cast<std::streamsize>(count * sizeof(double)));
}

void WrStr(std::ofstream& os, const std::string& s)
{
    Wr32(os, static_cast<uint32_t>(s.size()));
    if (!s.empty()) {
        os.write(s.data(), static_cast<std::streamsize>(s.size()));
    }
}

void WrEnt(std::ofstream& os, const brepdb::EntityEntry& e)
{
    Wr32(os, e.persistent_id);
    Wr32(os, static_cast<uint32_t>(e.type));
    WrF64(os, e.min_pt, 3);
    WrF64(os, e.max_pt, 3);
    Wr32(os, static_cast<uint32_t>(e.params.size()));
    if (!e.params.empty()) {
        os.write(reinterpret_cast<const char*>(e.params.data()),
                 static_cast<std::streamsize>(e.params.size() * sizeof(double)));
    }
}

void WrHunks(std::ofstream& os, const std::vector<brepdb::ParamHunk>& hunks)
{
    Wr32(os, static_cast<uint32_t>(hunks.size()));
    for (const auto& h : hunks)
    {
        Wr32(os, h.offset);
        Wr32(os, static_cast<uint32_t>(h.data.size()));
        if (!h.data.empty()) {
            os.write(reinterpret_cast<const char*>(h.data.data()),
                     static_cast<std::streamsize>(h.data.size() * sizeof(double)));
        }
    }
}

void WrDiff(std::ofstream& os, const brepdb::PoolDiff& d)
{
    Wr32(os, static_cast<uint32_t>(d.added.size()));
    for (const auto& e : d.added) { WrEnt(os, e); }

    Wr32(os, static_cast<uint32_t>(d.removed.size()));
    for (const auto& e : d.removed) { WrEnt(os, e); }

    Wr32(os, static_cast<uint32_t>(d.modified.size()));
    for (const auto& m : d.modified)
    {
        Wr32(os, m.old_persistent_id);
        Wr32(os, m.new_persistent_id);
        Wr32(os, static_cast<uint32_t>(m.old_type));
        Wr32(os, static_cast<uint32_t>(m.new_type));
        WrF64(os, m.old_min_pt, 3);
        WrF64(os, m.old_max_pt, 3);
        WrF64(os, m.new_min_pt, 3);
        WrF64(os, m.new_max_pt, 3);
        Wr32(os, m.old_param_count);
        Wr32(os, m.new_param_count);
        WrHunks(os, m.forward_hunks);
        WrHunks(os, m.reverse_hunks);
    }

    Wr32(os, static_cast<uint32_t>(d.new_order.size()));
    if (!d.new_order.empty()) {
        os.write(reinterpret_cast<const char*>(d.new_order.data()),
                 static_cast<std::streamsize>(d.new_order.size() * sizeof(uint32_t)));
    }

    Wr32(os, static_cast<uint32_t>(d.old_order.size()));
    if (!d.old_order.empty()) {
        os.write(reinterpret_cast<const char*>(d.old_order.data()),
                 static_cast<std::streamsize>(d.old_order.size() * sizeof(uint32_t)));
    }
}

// ---- stream read helpers ----

uint32_t Rd32(std::ifstream& is)
{
    uint32_t v;
    is.read(reinterpret_cast<char*>(&v), sizeof(v));
    return v;
}

uint64_t Rd64(std::ifstream& is)
{
    uint64_t v;
    is.read(reinterpret_cast<char*>(&v), sizeof(v));
    return v;
}

void RdF64(std::ifstream& is, double* data, size_t count) {
    if (count > 0)
        is.read(reinterpret_cast<char*>(data),
                static_cast<std::streamsize>(count * sizeof(double)));
}

std::string RdStr(std::ifstream& is)
{
    uint32_t len = Rd32(is);
    std::string s(len, '\0');
    if (len > 0) {
        is.read(s.data(), static_cast<std::streamsize>(len));
    }
    return s;
}

brepdb::EntityEntry RdEnt(std::ifstream& is)
{
    brepdb::EntityEntry e;
    e.persistent_id = Rd32(is);
    e.type = static_cast<brepdb::Type>(Rd32(is));
    RdF64(is, e.min_pt, 3);
    RdF64(is, e.max_pt, 3);
    uint32_t pc = Rd32(is);
    e.params.resize(pc);
    if (pc > 0) {
        is.read(reinterpret_cast<char*>(e.params.data()),
                static_cast<std::streamsize>(pc * sizeof(double)));
    }
    return e;
}

std::vector<brepdb::ParamHunk> RdHunks(std::ifstream& is)
{
    uint32_t n = Rd32(is);
    std::vector<brepdb::ParamHunk> hunks(n);
    for (uint32_t j = 0; j < n; ++j)
    {
        hunks[j].offset = Rd32(is);
        uint32_t cnt = Rd32(is);
        hunks[j].data.resize(cnt);
        if (cnt > 0) {
            is.read(reinterpret_cast<char*>(hunks[j].data.data()),
                    static_cast<std::streamsize>(cnt * sizeof(double)));
        }
    }
    return hunks;
}

brepdb::PoolDiff RdDiff(std::ifstream& is)
{
    brepdb::PoolDiff d;

    uint32_t na = Rd32(is);
    d.added.resize(na);
    for (uint32_t j = 0; j < na; ++j) { d.added[j] = RdEnt(is); }

    uint32_t nr = Rd32(is);
    d.removed.resize(nr);
    for (uint32_t j = 0; j < nr; ++j) { d.removed[j] = RdEnt(is); }

    uint32_t nm = Rd32(is);
    d.modified.resize(nm);
    for (uint32_t j = 0; j < nm; ++j)
    {
        auto& m = d.modified[j];
        m.old_persistent_id = Rd32(is);
        m.new_persistent_id = Rd32(is);
        m.old_type = static_cast<brepdb::Type>(Rd32(is));
        m.new_type = static_cast<brepdb::Type>(Rd32(is));
        RdF64(is, m.old_min_pt, 3);
        RdF64(is, m.old_max_pt, 3);
        RdF64(is, m.new_min_pt, 3);
        RdF64(is, m.new_max_pt, 3);
        m.old_param_count = Rd32(is);
        m.new_param_count = Rd32(is);
        m.forward_hunks   = RdHunks(is);
        m.reverse_hunks   = RdHunks(is);
    }

    uint32_t no = Rd32(is);
    d.new_order.resize(no);
    if (no > 0) {
        is.read(reinterpret_cast<char*>(d.new_order.data()),
                static_cast<std::streamsize>(no * sizeof(uint32_t)));
    }

    uint32_t oo = Rd32(is);
    d.old_order.resize(oo);
    if (oo > 0) {
        is.read(reinterpret_cast<char*>(d.old_order.data()),
                static_cast<std::streamsize>(oo * sizeof(uint32_t)));
    }

    return d;
}

void WrWorld(std::ofstream& os, const brepdb::BRepWorld& world)
{
    const auto& alive = world.AliveEntities();
    Wr32(os, static_cast<uint32_t>(alive.size()));
    for (uint32_t id : alive)
    {
        Wr32(os, id);
        const brepdb::Type* t = world.Types().Get(id);
        Wr32(os, static_cast<uint32_t>(t ? *t : brepdb::Type::Empty));
        const brepdb::AabbComp* aabb = world.Aabbs().Get(id);
        double min_pt[3] = {0,0,0}, max_pt[3] = {0,0,0};
        if (aabb) {
            std::memcpy(min_pt, aabb->min_pt, sizeof(min_pt));
            std::memcpy(max_pt, aabb->max_pt, sizeof(max_pt));
        }
        os.write(reinterpret_cast<const char*>(min_pt), sizeof(min_pt));
        os.write(reinterpret_cast<const char*>(max_pt), sizeof(max_pt));
        auto params = world.ExportEntityParams(id);
        Wr32(os, static_cast<uint32_t>(params.size()));
        if (!params.empty())
            os.write(reinterpret_cast<const char*>(params.data()),
                     static_cast<std::streamsize>(params.size() * sizeof(double)));
    }
}

brepdb::WorldPtr RdWorld(std::ifstream& is)
{
    auto w = std::make_shared<brepdb::BRepWorld>();
    uint32_t count = Rd32(is);
    for (uint32_t i = 0; i < count; ++i)
    {
        uint32_t id = Rd32(is);
        brepdb::Type type = static_cast<brepdb::Type>(Rd32(is));
        double min_pt[3], max_pt[3];
        is.read(reinterpret_cast<char*>(min_pt), sizeof(min_pt));
        is.read(reinterpret_cast<char*>(max_pt), sizeof(max_pt));
        uint32_t pc = Rd32(is);
        std::vector<double> params(pc);
        if (pc > 0)
            is.read(reinterpret_cast<char*>(params.data()),
                    static_cast<std::streamsize>(pc * sizeof(double)));

        w->RegisterEntity(id);
        w->Types().Set(id, type);
        brepdb::AabbComp aabb;
        std::memcpy(aabb.min_pt, min_pt, sizeof(aabb.min_pt));
        std::memcpy(aabb.max_pt, max_pt, sizeof(aabb.max_pt));
        w->Aabbs().Set(id, aabb);
        if (!params.empty()) {
            brepdb::ParamsComp p;
            p.data = std::move(params);
            w->Params().Set(id, p);
        }
    }
    return w;
}

// ---- memory buffer write helpers (for byte-array serialization) ----

struct MemWriter
{
    std::vector<uint8_t> buf;

    void Write(const void* data, size_t size)
    {
        const auto* p = reinterpret_cast<const uint8_t*>(data);
        buf.insert(buf.end(), p, p + size);
    }

    void W32(uint32_t v) { Write(&v, 4); }
    void W64(uint64_t v) { Write(&v, 8); }

    void WWorld(const brepdb::BRepWorld& world)
    {
        const auto& alive = world.AliveEntities();
        W32(static_cast<uint32_t>(alive.size()));
        for (uint32_t id : alive)
        {
            W32(id);
            const brepdb::Type* t = world.Types().Get(id);
            W32(static_cast<uint32_t>(t ? *t : brepdb::Type::Empty));
            const brepdb::AabbComp* aabb = world.Aabbs().Get(id);
            double min_pt[3] = {0,0,0}, max_pt[3] = {0,0,0};
            if (aabb) {
                std::memcpy(min_pt, aabb->min_pt, sizeof(min_pt));
                std::memcpy(max_pt, aabb->max_pt, sizeof(max_pt));
            }
            Write(min_pt, sizeof(min_pt));
            Write(max_pt, sizeof(max_pt));
            auto params = world.ExportEntityParams(id);
            W32(static_cast<uint32_t>(params.size()));
            if (!params.empty()) { Write(params.data(), params.size() * sizeof(double)); }
        }
    }

    void WStr(const std::string& s)
    {
        W32(static_cast<uint32_t>(s.size()));
        if (!s.empty()) { Write(s.data(), s.size()); }
    }

    void WEnt(const brepdb::EntityEntry& e)
    {
        W32(e.persistent_id);
        W32(static_cast<uint32_t>(e.type));
        Write(e.min_pt, 3 * sizeof(double));
        Write(e.max_pt, 3 * sizeof(double));
        W32(static_cast<uint32_t>(e.params.size()));
        if (!e.params.empty()) { Write(e.params.data(), e.params.size() * sizeof(double)); }
    }

    void WHunks(const std::vector<brepdb::ParamHunk>& hunks)
    {
        W32(static_cast<uint32_t>(hunks.size()));
        for (const auto& h : hunks)
        {
            W32(h.offset);
            W32(static_cast<uint32_t>(h.data.size()));
            if (!h.data.empty()) { Write(h.data.data(), h.data.size() * sizeof(double)); }
        }
    }

    void WDiff(const brepdb::PoolDiff& d)
    {
        W32(static_cast<uint32_t>(d.added.size()));
        for (const auto& e : d.added) { WEnt(e); }

        W32(static_cast<uint32_t>(d.removed.size()));
        for (const auto& e : d.removed) { WEnt(e); }

        W32(static_cast<uint32_t>(d.modified.size()));
        for (const auto& m : d.modified)
        {
            W32(m.old_persistent_id);
            W32(m.new_persistent_id);
            W32(static_cast<uint32_t>(m.old_type));
            W32(static_cast<uint32_t>(m.new_type));
            Write(m.old_min_pt, 3 * sizeof(double));
            Write(m.old_max_pt, 3 * sizeof(double));
            Write(m.new_min_pt, 3 * sizeof(double));
            Write(m.new_max_pt, 3 * sizeof(double));
            W32(m.old_param_count);
            W32(m.new_param_count);
            WHunks(m.forward_hunks);
            WHunks(m.reverse_hunks);
        }

        W32(static_cast<uint32_t>(d.new_order.size()));
        if (!d.new_order.empty()) { Write(d.new_order.data(), d.new_order.size() * sizeof(uint32_t)); }

        W32(static_cast<uint32_t>(d.old_order.size()));
        if (!d.old_order.empty()) { Write(d.old_order.data(), d.old_order.size() * sizeof(uint32_t)); }
    }
};

// ---- memory buffer read helpers ----

struct MemReader
{
    const uint8_t* data;
    size_t         size;
    size_t         pos = 0;

    void Read(void* dst, size_t n)
    {
        assert(pos + n <= size);
        std::memcpy(dst, data + pos, n);
        pos += n;
    }

    uint32_t R32() { uint32_t v; Read(&v, 4); return v; }
    uint64_t R64() { uint64_t v; Read(&v, 8); return v; }

    brepdb::WorldPtr RWorld()
    {
        auto w = std::make_shared<brepdb::BRepWorld>();
        uint32_t count = R32();
        for (uint32_t i = 0; i < count; ++i)
        {
            uint32_t id = R32();
            brepdb::Type type = static_cast<brepdb::Type>(R32());
            double min_pt[3], max_pt[3];
            Read(min_pt, sizeof(min_pt));
            Read(max_pt, sizeof(max_pt));
            uint32_t pc = R32();
            std::vector<double> params(pc);
            if (pc > 0) { Read(params.data(), pc * sizeof(double)); }

            w->RegisterEntity(id);
            w->Types().Set(id, type);
            brepdb::AabbComp aabb;
            std::memcpy(aabb.min_pt, min_pt, sizeof(aabb.min_pt));
            std::memcpy(aabb.max_pt, max_pt, sizeof(aabb.max_pt));
            w->Aabbs().Set(id, aabb);
            if (!params.empty()) {
                brepdb::ParamsComp p;
                p.data = std::move(params);
                w->Params().Set(id, p);
            }
        }
        return w;
    }

    std::string RStr()
    {
        uint32_t len = R32();
        std::string s(len, '\0');
        if (len > 0) { Read(s.data(), len); }
        return s;
    }

    brepdb::EntityEntry REnt()
    {
        brepdb::EntityEntry e;
        e.persistent_id = R32();
        e.type = static_cast<brepdb::Type>(R32());
        Read(e.min_pt, 3 * sizeof(double));
        Read(e.max_pt, 3 * sizeof(double));
        uint32_t pc = R32();
        e.params.resize(pc);
        if (pc > 0) { Read(e.params.data(), pc * sizeof(double)); }
        return e;
    }

    std::vector<brepdb::ParamHunk> RHunks()
    {
        uint32_t n = R32();
        std::vector<brepdb::ParamHunk> hunks(n);
        for (uint32_t j = 0; j < n; ++j)
        {
            hunks[j].offset = R32();
            uint32_t cnt = R32();
            hunks[j].data.resize(cnt);
            if (cnt > 0) { Read(hunks[j].data.data(), cnt * sizeof(double)); }
        }
        return hunks;
    }

    brepdb::PoolDiff RDiff()
    {
        brepdb::PoolDiff d;

        uint32_t na = R32();
        d.added.resize(na);
        for (uint32_t j = 0; j < na; ++j) { d.added[j] = REnt(); }

        uint32_t nr = R32();
        d.removed.resize(nr);
        for (uint32_t j = 0; j < nr; ++j) { d.removed[j] = REnt(); }

        uint32_t nm = R32();
        d.modified.resize(nm);
        for (uint32_t j = 0; j < nm; ++j)
        {
            auto& m = d.modified[j];
            m.old_persistent_id = R32();
            m.new_persistent_id = R32();
            m.old_type = static_cast<brepdb::Type>(R32());
            m.new_type = static_cast<brepdb::Type>(R32());
            Read(m.old_min_pt, 3 * sizeof(double));
            Read(m.old_max_pt, 3 * sizeof(double));
            Read(m.new_min_pt, 3 * sizeof(double));
            Read(m.new_max_pt, 3 * sizeof(double));
            m.old_param_count = R32();
            m.new_param_count = R32();
            m.forward_hunks   = RHunks();
            m.reverse_hunks   = RHunks();
        }

        uint32_t no = R32();
        d.new_order.resize(no);
        if (no > 0) { Read(d.new_order.data(), no * sizeof(uint32_t)); }

        uint32_t oo = R32();
        d.old_order.resize(oo);
        if (oo > 0) { Read(d.old_order.data(), oo * sizeof(uint32_t)); }

        return d;
    }
};

// Write all nodes in BFS order (roots first), DAG-safe.
void WriteNodesBfs(std::ofstream& os,
                   const std::unordered_map<uint32_t, brepdb::VersionNode>& nodes,
                   const std::vector<uint32_t>& root_ids,
                   bool include_diff)
{
    std::set<uint32_t> visited;
    std::queue<uint32_t> q;
    for (uint32_t rid : root_ids)
    {
        if (rid != UINT32_MAX) { q.push(rid); visited.insert(rid); }
    }

    while (!q.empty())
    {
        uint32_t id = q.front();
        q.pop();

        auto it = nodes.find(id);
        if (it == nodes.end()) { continue; }

        const brepdb::VersionNode& node = it->second;

        Wr32(os, node.id);
        Wr32(os, node.parent_id);

        Wr32(os, static_cast<uint32_t>(node.aux_parent_ids.size()));
        for (uint32_t ap : node.aux_parent_ids) { Wr32(os, ap); }

        Wr32(os, static_cast<uint32_t>(node.children.size()));
        for (uint32_t c : node.children) { Wr32(os, c); }

        WrStr(os, node.op_desc);
        Wr32(os, node.op_type);
        Wr64(os, node.timestamp);

        if (include_diff && node.parent_id != UINT32_MAX) { WrDiff(os, node.diff); }

        for (uint32_t c : node.children)
        {
            if (visited.insert(c).second) { q.push(c); }
        }
    }
}

void WriteNodesBfs(MemWriter& w,
                   const std::unordered_map<uint32_t, brepdb::VersionNode>& nodes,
                   const std::vector<uint32_t>& root_ids)
{
    std::set<uint32_t> visited;
    std::queue<uint32_t> q;
    for (uint32_t rid : root_ids)
    {
        if (rid != UINT32_MAX) { q.push(rid); visited.insert(rid); }
    }

    while (!q.empty())
    {
        uint32_t id = q.front();
        q.pop();

        auto it = nodes.find(id);
        if (it == nodes.end()) { continue; }

        const brepdb::VersionNode& node = it->second;

        w.W32(node.id);
        w.W32(node.parent_id);

        w.W32(static_cast<uint32_t>(node.aux_parent_ids.size()));
        for (uint32_t ap : node.aux_parent_ids) { w.W32(ap); }

        w.W32(static_cast<uint32_t>(node.children.size()));
        for (uint32_t c : node.children) { w.W32(c); }

        w.WStr(node.op_desc);
        w.W32(node.op_type);
        w.W64(node.timestamp);

        if (node.parent_id != UINT32_MAX) { w.WDiff(node.diff); }

        for (uint32_t c : node.children)
        {
            if (visited.insert(c).second) { q.push(c); }
        }
    }
}

uint32_t ReadNodes(std::ifstream& is,
                   uint32_t node_count,
                   std::unordered_map<uint32_t, brepdb::VersionNode>& out_nodes,
                   bool read_diff)
{
    uint32_t max_id = 0;
    for (uint32_t i = 0; i < node_count; ++i)
    {
        brepdb::VersionNode node;
        node.id        = Rd32(is);
        node.parent_id = Rd32(is);

        uint32_t ac = Rd32(is);
        node.aux_parent_ids.resize(ac);
        for (uint32_t j = 0; j < ac; ++j) { node.aux_parent_ids[j] = Rd32(is); }

        uint32_t cc = Rd32(is);
        node.children.resize(cc);
        for (uint32_t j = 0; j < cc; ++j) { node.children[j] = Rd32(is); }

        node.op_desc   = RdStr(is);
        node.op_type   = Rd32(is);
        node.timestamp = Rd64(is);

        if (read_diff && node.parent_id != UINT32_MAX) { node.diff = RdDiff(is); }

        if (node.id > max_id) { max_id = node.id; }
        out_nodes[node.id] = std::move(node);
    }
    return max_id;
}

uint32_t ReadNodes(MemReader& r,
                   uint32_t node_count,
                   std::unordered_map<uint32_t, brepdb::VersionNode>& out_nodes)
{
    uint32_t max_id = 0;
    for (uint32_t i = 0; i < node_count; ++i)
    {
        brepdb::VersionNode node;
        node.id        = r.R32();
        node.parent_id = r.R32();

        uint32_t ac = r.R32();
        node.aux_parent_ids.resize(ac);
        for (uint32_t j = 0; j < ac; ++j) { node.aux_parent_ids[j] = r.R32(); }

        uint32_t cc = r.R32();
        node.children.resize(cc);
        for (uint32_t j = 0; j < cc; ++j) { node.children[j] = r.R32(); }

        node.op_desc   = r.RStr();
        node.op_type   = r.R32();
        node.timestamp = r.R64();

        if (node.parent_id != UINT32_MAX) { node.diff = r.RDiff(); }

        if (node.id > max_id) { max_id = node.id; }
        out_nodes[node.id] = std::move(node);
    }
    return max_id;
}

} // anonymous namespace


namespace brepdb
{

// ============================================================
// EntityEntry
// ============================================================

bool EntityEntry::operator==(const EntityEntry& rhs) const
{
    if (persistent_id != rhs.persistent_id) { return false; }
    if (type != rhs.type)                   { return false; }

    for (int i = 0; i < 3; ++i)
    {
        if (min_pt[i] != rhs.min_pt[i]) { return false; }
        if (max_pt[i] != rhs.max_pt[i]) { return false; }
    }

    if (params.size() != rhs.params.size()) { return false; }

    if (!params.empty()) {
        if (std::memcmp(params.data(), rhs.params.data(),
                        params.size() * sizeof(double)) != 0) {
            return false;
        }
    }

    return true;
}

// ============================================================
// Param hunk computation
// ============================================================

void VersionTree::ComputeParamHunks(const std::vector<double>& old_params,
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
        if (old_params[i] == new_params[i]) {
            ++i;
            continue;
        }

        size_t start = i;
        while (i < common)
        {
            if (old_params[i] != new_params[i]) {
                ++i;
                continue;
            }
            size_t gap_end = std::min(i + COALESCE_GAP, common);
            bool   more    = false;
            for (size_t j = i; j < gap_end; ++j)
            {
                if (old_params[j] != new_params[j]) {
                    more = true;
                    break;
                }
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

std::vector<double> VersionTree::ApplyParamHunks(const std::vector<double>& base,
                                                  const std::vector<ParamHunk>& hunks,
                                                  uint32_t target_size)
{
    std::vector<double> result(target_size);

    size_t copy_len = std::min(base.size(), static_cast<size_t>(target_size));
    if (copy_len > 0) {
        std::memcpy(result.data(), base.data(), copy_len * sizeof(double));
    }

    for (const auto& h : hunks)
    {
        size_t end = h.offset + h.data.size();
        if (end > result.size()) { result.resize(end); }
        std::memcpy(result.data() + h.offset,
                    h.data.data(),
                    h.data.size() * sizeof(double));
    }

    result.resize(target_size);
    return result;
}

PoolDiff::ModifiedEntry VersionTree::BuildModifiedEntry(uint32_t           old_pid,
                                                         uint32_t           new_pid,
                                                         const EntityEntry& old_entry,
                                                         const EntityEntry& new_entry)
{
    PoolDiff::ModifiedEntry m;
    m.old_persistent_id = old_pid;
    m.new_persistent_id = new_pid;
    m.old_type          = old_entry.type;
    m.new_type          = new_entry.type;
    std::memcpy(m.old_min_pt, old_entry.min_pt, sizeof(m.old_min_pt));
    std::memcpy(m.old_max_pt, old_entry.max_pt, sizeof(m.old_max_pt));
    std::memcpy(m.new_min_pt, new_entry.min_pt, sizeof(m.new_min_pt));
    std::memcpy(m.new_max_pt, new_entry.max_pt, sizeof(m.new_max_pt));
    m.old_param_count   = static_cast<uint32_t>(old_entry.params.size());
    m.new_param_count   = static_cast<uint32_t>(new_entry.params.size());

    ComputeParamHunks(old_entry.params, new_entry.params,
                      m.forward_hunks, m.reverse_hunks);
    return m;
}

// ============================================================
// VersionTree - construction
// ============================================================

VersionTree::VersionTree() {}

uint32_t VersionTree::AddRoot(const BRepWorld&   world,
                               const std::string& op_desc,
                               uint32_t           op_type)
{
    uint32_t id = AllocNodeId();

    if (m_root_id == UINT32_MAX) { m_root_id = id; }

    VersionNode node;
    node.id        = id;
    node.parent_id = UINT32_MAX;
    node.op_desc   = op_desc;
    node.op_type   = op_type;
    node.timestamp = NowMs();

    m_nodes[id] = std::move(node);

    auto world_ptr = std::make_shared<BRepWorld>(world);
    m_root_worlds[id] = world_ptr;

    RootCursor cursor;
    cursor.current_id    = id;
    cursor.current_world = world_ptr;
    m_cursors[id] = cursor;

    return id;
}

uint32_t VersionTree::Commit(uint32_t            root_id,
                              const BRepWorld&    new_world,
                              PoolDiff&&          diff,
                              const std::string&  op_desc,
                              uint32_t            op_type)
{
    auto& cursor = m_cursors.at(root_id);
    return Branch(cursor.current_id, new_world, std::move(diff), op_desc, op_type);
}

uint32_t VersionTree::Commit(uint32_t            root_id,
                              const BRepWorld&    new_world,
                              const std::string&  op_desc,
                              uint32_t            op_type)
{
    auto& cursor = m_cursors.at(root_id);
    PoolDiff diff = ComputeDiff(*cursor.current_world, new_world);
    return Branch(cursor.current_id, new_world, std::move(diff), op_desc, op_type);
}

uint32_t VersionTree::Commit(uint32_t            root_id,
                              const BRepWorld&    new_world,
                              const PidMapping&   pid_map,
                              const std::string&  op_desc,
                              uint32_t            op_type)
{
    auto& cursor = m_cursors.at(root_id);
    PoolDiff diff = BuildDiffFromPidMapping(*cursor.current_world, new_world, pid_map);
    return Branch(cursor.current_id, new_world, std::move(diff), op_desc, op_type);
}

uint32_t VersionTree::Branch(uint32_t            parent_id,
                              const BRepWorld&    new_world,
                              PoolDiff&&          diff,
                              const std::string&  op_desc,
                              uint32_t            op_type)
{
    assert(m_nodes.count(parent_id));

    uint32_t root_id = FindRootOf(parent_id);
    auto& cursor = m_cursors.at(root_id);
    if (parent_id != cursor.current_id) { NavigateTo(root_id, parent_id); }

    uint32_t new_id = AllocNodeId();

    VersionNode node;
    node.id        = new_id;
    node.parent_id = parent_id;
    node.op_desc   = op_desc;
    node.op_type   = op_type;
    node.timestamp = NowMs();
    node.diff      = std::move(diff);

    m_nodes[parent_id].children.push_back(new_id);
    m_nodes[new_id] = std::move(node);

    cursor.current_id    = new_id;
    cursor.current_world = std::make_shared<BRepWorld>(new_world);

    return new_id;
}

uint32_t VersionTree::Merge(uint32_t                       primary_parent_id,
                             const std::vector<uint32_t>&   aux_parent_ids,
                             const BRepWorld&               new_world,
                             PoolDiff&&                     diff,
                             const std::string&             op_desc,
                             uint32_t                       op_type)
{
    assert(m_nodes.count(primary_parent_id));

    uint32_t root_id = FindRootOf(primary_parent_id);
    auto& cursor = m_cursors.at(root_id);
    if (primary_parent_id != cursor.current_id) { NavigateTo(root_id, primary_parent_id); }

    uint32_t new_id = AllocNodeId();

    VersionNode node;
    node.id             = new_id;
    node.parent_id      = primary_parent_id;
    node.aux_parent_ids = aux_parent_ids;
    node.op_desc        = op_desc;
    node.op_type        = op_type;
    node.timestamp      = NowMs();
    node.diff           = std::move(diff);

    m_nodes[primary_parent_id].children.push_back(new_id);
    for (uint32_t aux_id : aux_parent_ids)
    {
        assert(m_nodes.count(aux_id));
        m_nodes[aux_id].children.push_back(new_id);
    }
    m_nodes[new_id] = std::move(node);

    cursor.current_id    = new_id;
    cursor.current_world = std::make_shared<BRepWorld>(new_world);

    return new_id;
}

uint32_t VersionTree::Merge(uint32_t                       primary_parent_id,
                             const std::vector<uint32_t>&   aux_parent_ids,
                             const BRepWorld&               new_world,
                             const PidMapping&              pid_map,
                             const std::string&             op_desc,
                             uint32_t                       op_type)
{
    assert(m_nodes.count(primary_parent_id));

    uint32_t root_id = FindRootOf(primary_parent_id);
    auto& cursor = m_cursors.at(root_id);
    if (primary_parent_id != cursor.current_id) { NavigateTo(root_id, primary_parent_id); }

    PoolDiff diff = BuildDiffFromPidMapping(*cursor.current_world, new_world, pid_map);
    return Merge(primary_parent_id, aux_parent_ids, new_world, std::move(diff), op_desc, op_type);
}

// ============================================================
// VersionTree - navigation (per root)
// ============================================================

WorldPtr VersionTree::Checkout(uint32_t root_id, uint32_t node_id)
{
    assert(m_nodes.count(node_id));
    auto& cursor = m_cursors.at(root_id);
    if (node_id != cursor.current_id) { NavigateTo(root_id, node_id); }
    return cursor.current_world;
}

WorldPtr VersionTree::Undo(uint32_t root_id)
{
    assert(CanUndo(root_id));
    auto& cursor = m_cursors.at(root_id);
    return Checkout(root_id, m_nodes.at(cursor.current_id).parent_id);
}

WorldPtr VersionTree::Redo(uint32_t root_id, int child_index)
{
    assert(CanRedo(root_id));
    auto& cursor = m_cursors.at(root_id);
    const auto& children = m_nodes.at(cursor.current_id).children;

    uint32_t target;
    if (child_index < 0 || child_index >= static_cast<int>(children.size())) {
        target = children.back();
    } else {
        target = children[child_index];
    }

    return Checkout(root_id, target);
}

// ============================================================
// VersionTree - query
// ============================================================

WorldPtr VersionTree::GetCurrentWorld(uint32_t root_id) const
{
    auto it = m_cursors.find(root_id);
    return it != m_cursors.end() ? it->second.current_world : nullptr;
}

uint32_t VersionTree::GetCurrentId(uint32_t root_id) const
{
    auto it = m_cursors.find(root_id);
    return it != m_cursors.end() ? it->second.current_id : UINT32_MAX;
}

std::vector<uint32_t> VersionTree::GetRoots() const
{
    std::vector<uint32_t> roots;
    for (const auto& [id, node] : m_nodes)
    {
        if (node.parent_id == UINT32_MAX) { roots.push_back(id); }
    }
    return roots;
}

uint32_t VersionTree::FindRootOf(uint32_t node_id) const
{
    uint32_t cur = node_id;
    while (true)
    {
        auto it = m_nodes.find(cur);
        if (it == m_nodes.end()) { return UINT32_MAX; }
        if (it->second.parent_id == UINT32_MAX) { return cur; }
        cur = it->second.parent_id;
    }
}

std::vector<uint32_t> VersionTree::GetAllCurrentIds() const
{
    std::vector<uint32_t> ids;
    for (const auto& [root_id, cursor] : m_cursors)
    {
        ids.push_back(cursor.current_id);
    }
    return ids;
}

const RootCursor* VersionTree::GetCursor(uint32_t root_id) const
{
    auto it = m_cursors.find(root_id);
    return it != m_cursors.end() ? &it->second : nullptr;
}

const VersionNode* VersionTree::GetNode(uint32_t id) const
{
    auto it = m_nodes.find(id);
    return it != m_nodes.end() ? &it->second : nullptr;
}

bool VersionTree::CanUndo(uint32_t root_id) const
{
    auto cit = m_cursors.find(root_id);
    if (cit == m_cursors.end()) { return false; }
    auto nit = m_nodes.find(cit->second.current_id);
    return nit != m_nodes.end() && nit->second.parent_id != UINT32_MAX;
}

bool VersionTree::CanRedo(uint32_t root_id) const
{
    auto cit = m_cursors.find(root_id);
    if (cit == m_cursors.end()) { return false; }
    auto nit = m_nodes.find(cit->second.current_id);
    return nit != m_nodes.end() && !nit->second.children.empty();
}

std::vector<uint32_t> VersionTree::GetPathFromRoot(uint32_t node_id) const
{
    std::vector<uint32_t> path;
    uint32_t cur = node_id;

    while (cur != UINT32_MAX)
    {
        path.push_back(cur);
        auto it = m_nodes.find(cur);
        if (it == m_nodes.end()) { break; }
        cur = it->second.parent_id;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<uint32_t> VersionTree::GetLeaves() const
{
    std::vector<uint32_t> leaves;
    for (const auto& [id, node] : m_nodes)
    {
        if (node.children.empty()) { leaves.push_back(id); }
    }
    return leaves;
}

void VersionTree::TraverseAll(const std::function<void(const VersionNode&)>& visitor) const
{
    if (m_nodes.empty()) { return; }

    std::set<uint32_t> visited;
    std::queue<uint32_t> bfs;

    for (const auto& [id, node] : m_nodes)
    {
        if (node.parent_id == UINT32_MAX)
        {
            bfs.push(id);
            visited.insert(id);
        }
    }

    while (!bfs.empty())
    {
        uint32_t id = bfs.front();
        bfs.pop();

        auto it = m_nodes.find(id);
        if (it == m_nodes.end()) { continue; }

        visitor(it->second);
        for (uint32_t c : it->second.children)
        {
            if (visited.insert(c).second) { bfs.push(c); }
        }
    }
}

// ============================================================
// VersionTree - entity helpers
// ============================================================

EntityEntry VersionTree::ExtractEntity(const BRepWorld& world, uint32_t entity_id)
{
    EntityEntry e;
    e.persistent_id = entity_id;

    const Type* t = world.Types().Get(entity_id);
    if (t) e.type = *t;

    const AabbComp* aabb = world.Aabbs().Get(entity_id);
    if (aabb) {
        std::memcpy(e.min_pt, aabb->min_pt, sizeof(e.min_pt));
        std::memcpy(e.max_pt, aabb->max_pt, sizeof(e.max_pt));
    }

    e.params = world.ExportEntityParams(entity_id);

    return e;
}

// ============================================================
// VersionTree - diff building
// ============================================================

PoolDiff VersionTree::BuildDiffFromPidMapping(const BRepWorld& old_world,
                                               const BRepWorld& new_world,
                                               const PidMapping& pid_map)
{
    PoolDiff diff;

    const auto& old_alive = old_world.AliveEntities();
    const auto& new_alive = new_world.AliveEntities();

    std::set<uint32_t> old_set(old_alive.begin(), old_alive.end());
    std::set<uint32_t> new_set(new_alive.begin(), new_alive.end());

    for (uint32_t id : old_alive) { diff.old_order.push_back(id); }
    for (uint32_t id : new_alive) { diff.new_order.push_back(id); }

    std::set<uint32_t> accounted_new;

    for (const auto& [old_pid, new_pids] : pid_map)
    {
        bool has_old = old_set.count(old_pid) > 0;

        if (new_pids.empty())
        {
            if (has_old) {
                diff.removed.push_back(ExtractEntity(old_world, old_pid));
            }
            continue;
        }

        for (size_t k = 0; k < new_pids.size(); ++k)
        {
            uint32_t new_pid = new_pids[k];
            accounted_new.insert(new_pid);

            if (!new_set.count(new_pid)) { continue; }

            if (k == 0 && has_old) {
                diff.modified.push_back(
                    BuildModifiedEntry(old_pid, new_pid,
                                       ExtractEntity(old_world, old_pid),
                                       ExtractEntity(new_world, new_pid)));
            } else {
                diff.added.push_back(ExtractEntity(new_world, new_pid));
            }
        }
    }

    for (uint32_t pid : new_alive)
    {
        if (accounted_new.count(pid)) { continue; }

        if (!old_set.count(pid))
        {
            diff.added.push_back(ExtractEntity(new_world, pid));
        }
        else
        {
            auto oe = ExtractEntity(old_world, pid);
            auto ne = ExtractEntity(new_world, pid);
            if (oe != ne) {
                diff.modified.push_back(BuildModifiedEntry(pid, pid, oe, ne));
            }
        }
    }

    for (uint32_t pid : old_alive)
    {
        if (pid_map.count(pid)) { continue; }
        if (new_set.count(pid)) { continue; }
        diff.removed.push_back(ExtractEntity(old_world, pid));
    }

    return diff;
}

PoolDiff VersionTree::ComputeDiff(const BRepWorld& old_world,
                                   const BRepWorld& new_world)
{
    PoolDiff diff;

    const auto& old_alive = old_world.AliveEntities();
    const auto& new_alive = new_world.AliveEntities();

    std::set<uint32_t> old_set(old_alive.begin(), old_alive.end());
    std::set<uint32_t> new_set(new_alive.begin(), new_alive.end());

    for (uint32_t id : old_alive) { diff.old_order.push_back(id); }
    for (uint32_t id : new_alive) { diff.new_order.push_back(id); }

    for (uint32_t pid : new_alive)
    {
        if (!old_set.count(pid)) {
            diff.added.push_back(ExtractEntity(new_world, pid));
        } else {
            auto oe = ExtractEntity(old_world, pid);
            auto ne = ExtractEntity(new_world, pid);
            if (oe != ne) {
                diff.modified.push_back(BuildModifiedEntry(pid, pid, oe, ne));
            }
        }
    }

    for (uint32_t pid : old_alive)
    {
        if (!new_set.count(pid)) {
            diff.removed.push_back(ExtractEntity(old_world, pid));
        }
    }

    return diff;
}

// ============================================================
// VersionTree - world rebuild and apply
// ============================================================

WorldPtr VersionTree::RebuildWorld(
    const std::unordered_map<uint32_t, EntityEntry>& entities,
    const std::vector<uint32_t>&                     order)
{
    auto w = std::make_shared<BRepWorld>();
    for (uint32_t pid : order)
    {
        const EntityEntry& e = entities.at(pid);
        w->RegisterEntity(pid);
        w->Types().Set(pid, e.type);
        AabbComp aabb;
        std::memcpy(aabb.min_pt, e.min_pt, sizeof(aabb.min_pt));
        std::memcpy(aabb.max_pt, e.max_pt, sizeof(aabb.max_pt));
        w->Aabbs().Set(pid, aabb);
        if (!e.params.empty()) {
            ParamsComp pc;
            pc.data = e.params;
            w->Params().Set(pid, pc);
        }
    }
    w->RebuildTypedFromParams();
    return w;
}

WorldPtr VersionTree::ApplyForward(const BRepWorld& base, const PoolDiff& diff)
{
    std::unordered_map<uint32_t, EntityEntry> ents;
    for (uint32_t id : base.AliveEntities())
    {
        ents[id] = ExtractEntity(base, id);
    }

    for (const auto& e : diff.removed) { ents.erase(e.PersistentId()); }

    for (const auto& m : diff.modified)
    {
        auto it = ents.find(m.old_persistent_id);
        assert(it != ents.end());

        auto new_params = ApplyParamHunks(it->second.params,
                                           m.forward_hunks,
                                           m.new_param_count);
        ents.erase(it);

        EntityEntry ne;
        ne.persistent_id = m.new_persistent_id;
        ne.type          = m.new_type;
        std::memcpy(ne.min_pt, m.new_min_pt, sizeof(ne.min_pt));
        std::memcpy(ne.max_pt, m.new_max_pt, sizeof(ne.max_pt));
        ne.params        = std::move(new_params);
        ents[m.new_persistent_id] = std::move(ne);
    }

    for (const auto& e : diff.added) { ents[e.PersistentId()] = e; }

    return RebuildWorld(ents, diff.new_order);
}

WorldPtr VersionTree::ApplyReverse(const BRepWorld& current, const PoolDiff& diff)
{
    std::unordered_map<uint32_t, EntityEntry> ents;
    for (uint32_t id : current.AliveEntities())
    {
        ents[id] = ExtractEntity(current, id);
    }

    for (const auto& e : diff.added) { ents.erase(e.PersistentId()); }

    for (const auto& m : diff.modified)
    {
        auto it = ents.find(m.new_persistent_id);
        assert(it != ents.end());

        auto old_params = ApplyParamHunks(it->second.params,
                                           m.reverse_hunks,
                                           m.old_param_count);
        ents.erase(it);

        EntityEntry oe;
        oe.persistent_id = m.old_persistent_id;
        oe.type          = m.old_type;
        std::memcpy(oe.min_pt, m.old_min_pt, sizeof(oe.min_pt));
        std::memcpy(oe.max_pt, m.old_max_pt, sizeof(oe.max_pt));
        oe.params        = std::move(old_params);
        ents[m.old_persistent_id] = std::move(oe);
    }

    for (const auto& e : diff.removed) { ents[e.PersistentId()] = e; }

    return RebuildWorld(ents, diff.old_order);
}

// ============================================================
// VersionTree - internal navigation
// ============================================================

uint32_t VersionTree::AllocNodeId() { return m_next_id++; }

void VersionTree::NavigateTo(uint32_t root_id, uint32_t target_id)
{
    auto& cursor = m_cursors.at(root_id);
    if (cursor.current_id == target_id) { return; }

    uint32_t lca = FindLCA(cursor.current_id, target_id);

    WorldPtr world = cursor.current_world;

    uint32_t cur = cursor.current_id;
    while (cur != lca)
    {
        auto it = m_nodes.find(cur);
        world = ApplyReverse(*world, it->second.diff);
        cur = it->second.parent_id;
    }

    auto tgt_path = GetPathFromRoot(target_id);
    auto lca_it = std::find(tgt_path.begin(), tgt_path.end(), lca);
    assert(lca_it != tgt_path.end());

    for (auto it = lca_it + 1; it != tgt_path.end(); ++it)
    {
        auto node_it = m_nodes.find(*it);
        world = ApplyForward(*world, node_it->second.diff);
    }

    cursor.current_world = world;
    cursor.current_id    = target_id;
}

uint32_t VersionTree::FindLCA(uint32_t a, uint32_t b) const
{
    auto path_a = GetPathFromRoot(a);
    auto path_b = GetPathFromRoot(b);

    uint32_t lca = m_root_id;
    size_t   len = std::min(path_a.size(), path_b.size());

    for (size_t i = 0; i < len; ++i)
    {
        if (path_a[i] == path_b[i]) { lca = path_a[i]; }
        else { break; }
    }

    return lca;
}

// ============================================================
// VersionTree — persistence (file)
// ============================================================

bool VersionTree::SaveToFile(const std::string& filepath) const
{
    std::ofstream os(filepath, std::ios::binary);
    if (!os) { return false; }

    auto roots = GetRoots();

    FileHeader fh;
    fh.magic        = FILE_MAGIC;
    fh.version      = FILE_VERSION;
    fh.node_count   = static_cast<uint32_t>(m_nodes.size());
    fh.root_id      = m_root_id;
    fh.cursor_count = static_cast<uint32_t>(m_cursors.size());
    fh.reserved0    = static_cast<uint32_t>(roots.size());
    fh.reserved1    = 0;

    os.write(reinterpret_cast<const char*>(&fh), sizeof(fh));

    // Root worlds
    Wr32(os, static_cast<uint32_t>(roots.size()));
    for (uint32_t rid : roots)
    {
        Wr32(os, rid);
        auto rw_it = m_root_worlds.find(rid);
        if (rw_it != m_root_worlds.end()) {
            WrWorld(os, *rw_it->second);
        } else {
            auto cit = m_cursors.find(rid);
            assert(cit != m_cursors.end());
            WorldPtr root_world = cit->second.current_world;
            auto path = GetPathFromRoot(cit->second.current_id);
            for (int i = static_cast<int>(path.size()) - 1; i > 0; --i)
            {
                auto it = m_nodes.find(path[i]);
                root_world = ApplyReverse(*root_world, it->second.diff);
            }
            WrWorld(os, *root_world);
        }
    }

    // Cursor map
    Wr32(os, static_cast<uint32_t>(m_cursors.size()));
    for (const auto& [rid, cursor] : m_cursors)
    {
        Wr32(os, rid);
        Wr32(os, cursor.current_id);
    }

    WriteNodesBfs(os, m_nodes, roots, true);

    return true;
}

bool VersionTree::LoadFromFile(const std::string& filepath)
{
    std::ifstream is(filepath, std::ios::binary);
    if (!is) { return false; }

    FileHeader fh;
    is.read(reinterpret_cast<char*>(&fh), sizeof(fh));
    if (fh.magic != FILE_MAGIC || fh.version != FILE_VERSION) { return false; }

    Clear();
    m_root_id = fh.root_id;

    // Root worlds
    uint32_t root_count = Rd32(is);
    for (uint32_t i = 0; i < root_count; ++i)
    {
        uint32_t rid = Rd32(is);
        m_root_worlds[rid] = RdWorld(is);
    }

    // Cursor map
    uint32_t cursor_count = Rd32(is);
    for (uint32_t i = 0; i < cursor_count; ++i)
    {
        uint32_t rid = Rd32(is);
        uint32_t cid = Rd32(is);
        RootCursor cursor;
        cursor.current_id = cid;
        m_cursors[rid] = cursor;
    }

    uint32_t max_id = ReadNodes(is, fh.node_count, m_nodes, true);
    m_next_id       = max_id + 1;

    // Reconstruct each cursor's world from root world + forward diffs
    for (auto& [rid, cursor] : m_cursors)
    {
        auto rw_it = m_root_worlds.find(rid);
        if (rw_it == m_root_worlds.end()) { continue; }

        WorldPtr world = rw_it->second;
        auto path = GetPathFromRoot(cursor.current_id);
        for (size_t j = 1; j < path.size(); ++j)
        {
            auto node_it = m_nodes.find(path[j]);
            world = ApplyForward(*world, node_it->second.diff);
        }
        cursor.current_world = world;
    }

    return true;
}

// ============================================================
// VersionTree — persistence (byte array for BrepDB meta page)
// ============================================================

void VersionTree::StoreToByteArray(uint8_t** buf, uint32_t& len) const
{
    MemWriter w;

    auto roots = GetRoots();

    w.W32(FILE_MAGIC);
    w.W32(FILE_VERSION);
    w.W32(static_cast<uint32_t>(m_nodes.size()));
    w.W32(m_root_id);
    w.W32(static_cast<uint32_t>(m_cursors.size()));
    w.W32(static_cast<uint32_t>(roots.size()));
    w.W32(0);

    // Root worlds
    w.W32(static_cast<uint32_t>(roots.size()));
    for (uint32_t rid : roots)
    {
        w.W32(rid);
        auto rw_it = m_root_worlds.find(rid);
        if (rw_it != m_root_worlds.end()) {
            w.WWorld(*rw_it->second);
        } else {
            auto cit = m_cursors.find(rid);
            assert(cit != m_cursors.end());
            WorldPtr root_world = cit->second.current_world;
            auto path = GetPathFromRoot(cit->second.current_id);
            for (int i = static_cast<int>(path.size()) - 1; i > 0; --i)
            {
                auto it = m_nodes.find(path[i]);
                root_world = ApplyReverse(*root_world, it->second.diff);
            }
            w.WWorld(*root_world);
        }
    }

    // Cursor map
    for (const auto& [rid, cursor] : m_cursors)
    {
        w.W32(rid);
        w.W32(cursor.current_id);
    }

    WriteNodesBfs(w, m_nodes, roots);

    len  = static_cast<uint32_t>(w.buf.size());
    *buf = new uint8_t[len];
    std::memcpy(*buf, w.buf.data(), len);
}

void VersionTree::LoadFromByteArray(const uint8_t* buf, uint32_t len)
{
    MemReader r;
    r.data = buf;
    r.size = len;

    uint32_t magic        = r.R32();
    uint32_t version      = r.R32();
    if (magic != FILE_MAGIC || version != FILE_VERSION) { return; }

    uint32_t node_count   = r.R32();
    uint32_t root_id      = r.R32();
    uint32_t cursor_count = r.R32();
    r.R32();
    r.R32();

    Clear();
    m_root_id = root_id;

    // Root worlds
    uint32_t root_count = r.R32();
    for (uint32_t i = 0; i < root_count; ++i)
    {
        uint32_t rid = r.R32();
        m_root_worlds[rid] = r.RWorld();
    }

    // Cursor map
    for (uint32_t i = 0; i < cursor_count; ++i)
    {
        uint32_t rid = r.R32();
        uint32_t cid = r.R32();
        RootCursor cursor;
        cursor.current_id = cid;
        m_cursors[rid] = cursor;
    }

    uint32_t max_id = ReadNodes(r, node_count, m_nodes);
    m_next_id       = max_id + 1;

    // Reconstruct each cursor's world from root world + forward diffs
    for (auto& [rid, cursor] : m_cursors)
    {
        auto rw_it = m_root_worlds.find(rid);
        if (rw_it == m_root_worlds.end()) { continue; }

        WorldPtr world = rw_it->second;
        auto path = GetPathFromRoot(cursor.current_id);
        for (size_t j = 1; j < path.size(); ++j)
        {
            auto node_it = m_nodes.find(path[j]);
            world = ApplyForward(*world, node_it->second.diff);
        }
        cursor.current_world = world;
    }
}

void VersionTree::Clear()
{
    m_nodes.clear();
    m_cursors.clear();
    m_root_worlds.clear();
    m_root_id = UINT32_MAX;
    m_next_id = 0;
}

} // namespace brepdb
