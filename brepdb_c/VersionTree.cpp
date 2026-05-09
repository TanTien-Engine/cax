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
    uint32_t current_id;
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

void WrStr(std::ofstream& os, const std::string& s)
{
    Wr32(os, static_cast<uint32_t>(s.size()));
    if (!s.empty()) {
        os.write(s.data(), static_cast<std::streamsize>(s.size()));
    }
}

void WrEnt(std::ofstream& os, const brepdb::EntityEntry& e)
{
    os.write(reinterpret_cast<const char*>(&e.header), sizeof(brepdb::GeomHeader));
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
        os.write(reinterpret_cast<const char*>(&m.old_header), sizeof(brepdb::GeomHeader));
        os.write(reinterpret_cast<const char*>(&m.new_header), sizeof(brepdb::GeomHeader));
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
    is.read(reinterpret_cast<char*>(&e.header), sizeof(brepdb::GeomHeader));
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
        is.read(reinterpret_cast<char*>(&m.old_header), sizeof(brepdb::GeomHeader));
        is.read(reinterpret_cast<char*>(&m.new_header), sizeof(brepdb::GeomHeader));
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

void WrPool(std::ofstream& os, const brepdb::GeometryPool& pool)
{
    Wr32(os, static_cast<uint32_t>(pool.headers.size()));
    Wr32(os, static_cast<uint32_t>(pool.data_pool.size()));
    if (!pool.headers.empty()) {
        os.write(reinterpret_cast<const char*>(pool.headers.data()),
                 static_cast<std::streamsize>(pool.headers.size() * sizeof(brepdb::GeomHeader)));
    }
    if (!pool.data_pool.empty()) {
        os.write(reinterpret_cast<const char*>(pool.data_pool.data()),
                 static_cast<std::streamsize>(pool.data_pool.size() * sizeof(double)));
    }
}

brepdb::GeometryPool RdPool(std::ifstream& is)
{
    brepdb::GeometryPool pool;
    uint32_t hc = Rd32(is);
    uint32_t dc = Rd32(is);
    pool.headers.resize(hc);
    pool.data_pool.resize(dc);
    if (hc > 0) {
        is.read(reinterpret_cast<char*>(pool.headers.data()),
                static_cast<std::streamsize>(hc * sizeof(brepdb::GeomHeader)));
    }
    if (dc > 0) {
        is.read(reinterpret_cast<char*>(pool.data_pool.data()),
                static_cast<std::streamsize>(dc * sizeof(double)));
    }
    return pool;
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

    void WStr(const std::string& s)
    {
        W32(static_cast<uint32_t>(s.size()));
        if (!s.empty()) { Write(s.data(), s.size()); }
    }

    void WEnt(const brepdb::EntityEntry& e)
    {
        Write(&e.header, sizeof(brepdb::GeomHeader));
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
            Write(&m.old_header, sizeof(brepdb::GeomHeader));
            Write(&m.new_header, sizeof(brepdb::GeomHeader));
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
        Read(&e.header, sizeof(brepdb::GeomHeader));
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
            Read(&m.old_header, sizeof(brepdb::GeomHeader));
            Read(&m.new_header, sizeof(brepdb::GeomHeader));
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

// Write all nodes in BFS order (root first).
void WriteNodesBfs(std::ofstream& os,
                   const std::unordered_map<uint32_t, brepdb::VersionNode>& nodes,
                   uint32_t root_id,
                   bool include_diff)
{
    std::queue<uint32_t> q;
    if (root_id != UINT32_MAX) { q.push(root_id); }

    while (!q.empty())
    {
        uint32_t id = q.front();
        q.pop();

        auto it = nodes.find(id);
        if (it == nodes.end()) { continue; }

        const brepdb::VersionNode& node = it->second;

        Wr32(os, node.id);
        Wr32(os, node.parent_id);
        Wr32(os, static_cast<uint32_t>(node.children.size()));
        for (uint32_t c : node.children) { Wr32(os, c); }

        WrStr(os, node.op_desc);
        Wr32(os, node.op_type);
        Wr64(os, node.timestamp);

        if (include_diff && node.parent_id != UINT32_MAX) { WrDiff(os, node.diff); }

        for (uint32_t c : node.children) { q.push(c); }
    }
}

void WriteNodesBfs(MemWriter& w,
                   const std::unordered_map<uint32_t, brepdb::VersionNode>& nodes,
                   uint32_t root_id)
{
    std::queue<uint32_t> q;
    if (root_id != UINT32_MAX) { q.push(root_id); }

    while (!q.empty())
    {
        uint32_t id = q.front();
        q.pop();

        auto it = nodes.find(id);
        if (it == nodes.end()) { continue; }

        const brepdb::VersionNode& node = it->second;

        w.W32(node.id);
        w.W32(node.parent_id);
        w.W32(static_cast<uint32_t>(node.children.size()));
        for (uint32_t c : node.children) { w.W32(c); }

        w.WStr(node.op_desc);
        w.W32(node.op_type);
        w.W64(node.timestamp);

        if (node.parent_id != UINT32_MAX) { w.WDiff(node.diff); }

        for (uint32_t c : node.children) { q.push(c); }
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
    if (header.type != rhs.header.type)                     { return false; }
    if (header.persistent_id != rhs.header.persistent_id)  { return false; }
    if (header.param_count != rhs.header.param_count)       { return false; }

    for (int i = 0; i < 3; ++i)
    {
        if (header.min_pt[i] != rhs.header.min_pt[i]) { return false; }
        if (header.max_pt[i] != rhs.header.max_pt[i]) { return false; }
    }

    if (params.size() != rhs.params.size()) { return false; }

    if (!params.empty()) {
        if (std::memcmp(params.data(), rhs.params.data(),
                        params.size() * sizeof(double)) != 0) {
            return false;
        }
    }

    // param_offset is ignored — it is a pool-layout detail, not entity identity
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
    constexpr size_t COALESCE_GAP = 4;  // merge hunks closer than this many doubles

    size_t i = 0;
    while (i < common)
    {
        if (old_params[i] == new_params[i]) {
            ++i;
            continue;
        }

        // Found a difference — extend the hunk, coalescing nearby changes
        size_t start = i;
        while (i < common)
        {
            if (old_params[i] != new_params[i]) {
                ++i;
                continue;
            }
            // Check whether a further change appears within COALESCE_GAP
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

    // New params are longer: forward hunk appends the tail
    if (new_params.size() > old_params.size())
    {
        ParamHunk fh;
        fh.offset = static_cast<uint32_t>(old_params.size());
        fh.data.assign(new_params.begin() + old_params.size(), new_params.end());
        forward_hunks.push_back(std::move(fh));
    }

    // Old params are longer: reverse hunk appends the tail
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
    m.old_header        = old_entry.header;
    m.new_header        = new_entry.header;
    m.old_param_count   = static_cast<uint32_t>(old_entry.params.size());
    m.new_param_count   = static_cast<uint32_t>(new_entry.params.size());

    ComputeParamHunks(old_entry.params, new_entry.params,
                      m.forward_hunks, m.reverse_hunks);
    return m;
}

// ============================================================
// VersionTree — construction
// ============================================================

VersionTree::VersionTree() {}

uint32_t VersionTree::InitRoot(const GeometryPool& pool, const std::string& desc, uint32_t op_type)
{
    Clear();

    uint32_t id = AllocNodeId();
    m_root_id = id;
    m_current_id = id;

    VersionNode node;
    node.id = id;
    node.parent_id = UINT32_MAX;
    node.op_desc = desc;
    node.op_type = op_type;
    node.timestamp = NowMs();

    m_nodes[id] = std::move(node);
    m_current_pool = std::make_shared<GeometryPool>(pool);

    return id;
}

uint32_t VersionTree::Commit(const GeometryPool& new_pool,
                              PoolDiff&&          diff,
                              const std::string&  op_desc,
                              uint32_t            op_type)
{
    // First call: create root node, diff is irrelevant
    if (m_root_id == UINT32_MAX) { 
        return InitRoot(new_pool, op_desc, op_type); 
    }
    return Branch(m_current_id, new_pool, std::move(diff), op_desc, op_type);
}

uint32_t VersionTree::Commit(const GeometryPool& new_pool,
                              const std::string&  op_desc,
                              uint32_t            op_type)
{
    // First call: create root node, no diff needed
    if (m_root_id == UINT32_MAX) { 
        return InitRoot(new_pool, op_desc, op_type); 
    }
    PoolDiff diff = ComputeDiff(*m_current_pool, new_pool);
    return Branch(m_current_id, new_pool, std::move(diff), op_desc, op_type);
}

uint32_t VersionTree::Commit(const GeometryPool& new_pool,
                              const PidMapping&   pid_map,
                              const std::string&  op_desc,
                              uint32_t            op_type)
{
    // First call: no current pool yet, pid_map carries no useful information
    // (there is nothing to map _from_). Install new_pool as the root snapshot.
    if (m_root_id == UINT32_MAX) {
        return InitRoot(new_pool, op_desc, op_type);
    }

    PoolDiff diff = BuildDiffFromPidMapping(*m_current_pool, new_pool, pid_map);
    return Branch(m_current_id, new_pool, std::move(diff), op_desc, op_type);
}

uint32_t VersionTree::Branch(uint32_t            parent_id,
                              const GeometryPool& new_pool,
                              PoolDiff&&          diff,
                              const std::string&  op_desc,
                              uint32_t            op_type)
{
    assert(m_nodes.count(parent_id));
    if (parent_id != m_current_id) { NavigateTo(parent_id); }

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

    m_current_id   = new_id;
    m_current_pool = std::make_shared<GeometryPool>(new_pool);

    return new_id;
}

// ============================================================
// VersionTree — navigation
// ============================================================

PoolPtr VersionTree::Checkout(uint32_t node_id)
{
    assert(m_nodes.count(node_id));
    if (node_id != m_current_id) { NavigateTo(node_id); }
    return m_current_pool;
}

PoolPtr VersionTree::Undo()
{
    assert(CanUndo());
    return Checkout(m_nodes[m_current_id].parent_id);
}

PoolPtr VersionTree::Redo(int child_index)
{
    assert(CanRedo());
    const auto& children = m_nodes[m_current_id].children;

    uint32_t target;
    if (child_index < 0 || child_index >= static_cast<int>(children.size())) {
        target = children.back();
    } else {
        target = children[child_index];
    }

    return Checkout(target);
}

// ============================================================
// VersionTree — query
// ============================================================

const VersionNode* VersionTree::GetNode(uint32_t id) const
{
    auto it = m_nodes.find(id);
    return it != m_nodes.end() ? &it->second : nullptr;
}

bool VersionTree::CanUndo() const
{
    if (m_current_id == UINT32_MAX) { return false; }
    auto it = m_nodes.find(m_current_id);
    return it != m_nodes.end() && it->second.parent_id != UINT32_MAX;
}

bool VersionTree::CanRedo() const
{
    if (m_current_id == UINT32_MAX) { return false; }
    auto it = m_nodes.find(m_current_id);
    return it != m_nodes.end() && !it->second.children.empty();
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
    if (m_root_id == UINT32_MAX) { return; }

    std::queue<uint32_t> bfs;
    bfs.push(m_root_id);

    while (!bfs.empty())
    {
        uint32_t id = bfs.front();
        bfs.pop();

        auto it = m_nodes.find(id);
        if (it == m_nodes.end()) { continue; }

        visitor(it->second);
        for (uint32_t c : it->second.children) { bfs.push(c); }
    }
}

// ============================================================
// VersionTree — entity helpers
// ============================================================

EntityEntry VersionTree::ExtractEntity(const GeometryPool& pool, uint32_t idx)
{
    assert(idx < pool.headers.size());
    const auto& h = pool.headers[idx];

    EntityEntry e;
    e.header = h;

    if (h.param_count > 0 && h.param_offset + h.param_count <= pool.data_pool.size()) {
        e.params.assign(pool.data_pool.begin() + h.param_offset,
                        pool.data_pool.begin() + h.param_offset + h.param_count);
    }

    e.header.param_offset = 0;  // normalize: offset is meaningless outside pool
    return e;
}

std::unordered_map<uint32_t, uint32_t> VersionTree::BuildIdIndex(const GeometryPool& pool)
{
    std::unordered_map<uint32_t, uint32_t> idx;
    idx.reserve(pool.headers.size());
    for (uint32_t i = 0; i < static_cast<uint32_t>(pool.headers.size()); ++i) {
        idx[pool.headers[i].persistent_id] = i;
    }
    return idx;
}

// ============================================================
// VersionTree — diff building
// ============================================================

PoolDiff VersionTree::BuildDiffFromPidMapping(const GeometryPool& old_pool,
                                               const GeometryPool& new_pool,
                                               const PidMapping&   pid_map)
{
    PoolDiff diff;

    auto old_idx = BuildIdIndex(old_pool);
    auto new_idx = BuildIdIndex(new_pool);

    for (const auto& h : old_pool.headers) { diff.old_order.push_back(h.persistent_id); }
    for (const auto& h : new_pool.headers) { diff.new_order.push_back(h.persistent_id); }

    // Track which new pids are accounted for by the mapping
    std::set<uint32_t> accounted_new;

    for (const auto& [old_pid, new_pids] : pid_map)
    {
        auto old_it = old_idx.find(old_pid);
        bool has_old = (old_it != old_idx.end());

        if (new_pids.empty())
        {
            // Entity deleted
            if (has_old) {
                diff.removed.push_back(ExtractEntity(old_pool, old_it->second));
            }
            continue;
        }

        // k==0 with has_old: modified; k>0 or !has_old: added.
        // The !has_old case covers the initial commit (empty old_pool) and
        // any pid_map entry whose old_pid was never in old_pool — without
        // this, the primary new pid would be marked accounted but never
        // pushed into diff, silently disappearing.
        for (size_t k = 0; k < new_pids.size(); ++k)
        {
            uint32_t new_pid = new_pids[k];
            accounted_new.insert(new_pid);

            auto new_it = new_idx.find(new_pid);
            if (new_it == new_idx.end()) { continue; }

            if (k == 0 && has_old) {
                diff.modified.push_back(
                    BuildModifiedEntry(old_pid, new_pid,
                                       ExtractEntity(old_pool, old_it->second),
                                       ExtractEntity(new_pool, new_it->second)));
            } else {
                diff.added.push_back(ExtractEntity(new_pool, new_it->second));
            }
        }
    }

    // Entities in new_pool not referenced by the mapping:
    //   if the same pid exists in old_pool -> fallback modification check
    //   otherwise -> genuinely added
    for (const auto& h : new_pool.headers)
    {
        uint32_t pid = h.persistent_id;
        if (accounted_new.count(pid)) { continue; }

        auto old_it = old_idx.find(pid);
        if (old_it == old_idx.end())
        {
            // Pid not in old pool -> added
            auto new_it = new_idx.find(pid);
            if (new_it != new_idx.end()) {
                diff.added.push_back(ExtractEntity(new_pool, new_it->second));
            }
        }
        else
        {
            // Same pid survives; check if data changed
            auto oe = ExtractEntity(old_pool, old_it->second);
            auto ne = ExtractEntity(new_pool, new_idx.at(pid));
            if (oe != ne) {
                diff.modified.push_back(BuildModifiedEntry(pid, pid, oe, ne));
            }
        }
    }

    // Entities in old_pool not in new_pool and not already accounted for -> removed
    for (const auto& h : old_pool.headers)
    {
        uint32_t pid = h.persistent_id;
        if (pid_map.count(pid)) { continue; }  // handled above
        if (new_idx.count(pid)) { continue; }  // still exists

        auto it = old_idx.find(pid);
        if (it != old_idx.end()) {
            diff.removed.push_back(ExtractEntity(old_pool, it->second));
        }
    }

    return diff;
}

PoolDiff VersionTree::ComputeDiff(const GeometryPool& old_pool,
                                   const GeometryPool& new_pool)
{
    PoolDiff diff;

    auto old_idx = BuildIdIndex(old_pool);
    auto new_idx = BuildIdIndex(new_pool);

    for (const auto& h : old_pool.headers) { diff.old_order.push_back(h.persistent_id); }
    for (const auto& h : new_pool.headers) { diff.new_order.push_back(h.persistent_id); }

    for (const auto& [pid, ni] : new_idx)
    {
        auto it = old_idx.find(pid);
        if (it == old_idx.end()) {
            diff.added.push_back(ExtractEntity(new_pool, ni));
        } else {
            auto oe = ExtractEntity(old_pool, it->second);
            auto ne = ExtractEntity(new_pool, ni);
            if (oe != ne) {
                diff.modified.push_back(BuildModifiedEntry(pid, pid, oe, ne));
            }
        }
    }

    for (const auto& [pid, oi] : old_idx)
    {
        if (!new_idx.count(pid)) {
            diff.removed.push_back(ExtractEntity(old_pool, oi));
        }
    }

    return diff;
}

// ============================================================
// VersionTree — pool rebuild and apply
// ============================================================

GeometryPool VersionTree::RebuildPool(
    const std::unordered_map<uint32_t, EntityEntry>& entities,
    const std::vector<uint32_t>&                     order)
{
    GeometryPool pool;
    pool.headers.reserve(order.size());

    size_t total = 0;
    for (uint32_t pid : order)
    {
        auto it = entities.find(pid);
        assert(it != entities.end());
        total += it->second.params.size();
    }
    pool.data_pool.reserve(total);

    for (uint32_t pid : order)
    {
        const EntityEntry& e = entities.at(pid);

        GeomHeader h    = e.header;
        h.param_offset  = static_cast<uint32_t>(pool.data_pool.size());
        h.param_count   = static_cast<uint32_t>(e.params.size());

        pool.headers.push_back(h);
        pool.data_pool.insert(pool.data_pool.end(), e.params.begin(), e.params.end());
    }

    return pool;
}

GeometryPool VersionTree::ApplyForward(const GeometryPool& base, const PoolDiff& diff)
{
    std::unordered_map<uint32_t, EntityEntry> ents;
    for (uint32_t i = 0; i < static_cast<uint32_t>(base.headers.size()); ++i)
    {
        auto e = ExtractEntity(base, i);
        ents[e.PersistentId()] = std::move(e);
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
        ne.header              = m.new_header;
        ne.header.param_offset = 0;
        ne.params              = std::move(new_params);
        ents[m.new_persistent_id] = std::move(ne);
    }

    for (const auto& e : diff.added) { ents[e.PersistentId()] = e; }

    return RebuildPool(ents, diff.new_order);
}

GeometryPool VersionTree::ApplyReverse(const GeometryPool& current, const PoolDiff& diff)
{
    std::unordered_map<uint32_t, EntityEntry> ents;
    for (uint32_t i = 0; i < static_cast<uint32_t>(current.headers.size()); ++i)
    {
        auto e = ExtractEntity(current, i);
        ents[e.PersistentId()] = std::move(e);
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
        oe.header              = m.old_header;
        oe.header.param_offset = 0;
        oe.params              = std::move(old_params);
        ents[m.old_persistent_id] = std::move(oe);
    }

    for (const auto& e : diff.removed) { ents[e.PersistentId()] = e; }

    return RebuildPool(ents, diff.old_order);
}

// ============================================================
// VersionTree — internal navigation
// ============================================================

uint32_t VersionTree::AllocNodeId() { return m_next_id++; }

void VersionTree::NavigateTo(uint32_t target_id)
{
    if (m_current_id == target_id) { return; }

    uint32_t lca = FindLCA(m_current_id, target_id);

    // Walk up from current to LCA, applying reverse diffs
    uint32_t cur = m_current_id;
    while (cur != lca)
    {
        auto it = m_nodes.find(cur);
        m_current_pool = std::make_shared<GeometryPool>(
            ApplyReverse(*m_current_pool, it->second.diff));
        cur = it->second.parent_id;
    }

    // Walk down from LCA to target, applying forward diffs
    auto path  = GetPathFromRoot(target_id);
    auto lca_it = std::find(path.begin(), path.end(), lca);
    assert(lca_it != path.end());

    for (auto it = lca_it + 1; it != path.end(); ++it)
    {
        auto node_it = m_nodes.find(*it);
        m_current_pool = std::make_shared<GeometryPool>(
            ApplyForward(*m_current_pool, node_it->second.diff));
    }

    m_current_id = target_id;
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

    // Reconstruct root pool by reversing all diffs from current back to root
    GeometryPool root_pool = *m_current_pool;
    auto path = GetPathFromRoot(m_current_id);
    for (int i = static_cast<int>(path.size()) - 1; i > 0; --i)
    {
        auto it = m_nodes.find(path[i]);
        root_pool = ApplyReverse(root_pool, it->second.diff);
    }

    FileHeader fh;
    fh.magic      = FILE_MAGIC;
    fh.version    = FILE_VERSION;
    fh.node_count = static_cast<uint32_t>(m_nodes.size());
    fh.root_id    = m_root_id;
    fh.current_id = m_current_id;
    fh.reserved0  = 0;
    fh.reserved1  = 0;

    os.write(reinterpret_cast<const char*>(&fh), sizeof(fh));
    WrPool(os, root_pool);
    WriteNodesBfs(os, m_nodes, m_root_id, true);

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
    m_root_id    = fh.root_id;
    m_current_id = fh.root_id;

    GeometryPool root_pool = RdPool(is);

    uint32_t max_id = ReadNodes(is, fh.node_count, m_nodes, true);
    m_next_id       = max_id + 1;

    m_current_pool = std::make_shared<GeometryPool>(std::move(root_pool));

    if (fh.current_id != m_root_id)
    {
        m_current_id = m_root_id;
        NavigateTo(fh.current_id);
    }

    return true;
}

// ============================================================
// VersionTree — persistence (byte array for BrepDB meta page)
// ============================================================

void VersionTree::StoreToByteArray(uint8_t** buf, uint32_t& len) const
{
    MemWriter w;

    // Header
    w.W32(FILE_MAGIC);
    w.W32(FILE_VERSION);
    w.W32(static_cast<uint32_t>(m_nodes.size()));
    w.W32(m_root_id);
    w.W32(m_current_id);
    w.W32(0);  // reserved
    w.W32(0);  // reserved

    // Nodes — no root pool snapshot; entity data lives in the RTree
    WriteNodesBfs(w, m_nodes, m_root_id);

    len  = static_cast<uint32_t>(w.buf.size());
    *buf = new uint8_t[len];
    std::memcpy(*buf, w.buf.data(), len);
}

void VersionTree::LoadFromByteArray(const uint8_t*      buf,
                                     uint32_t            len,
                                     const GeometryPool& current_pool)
{
    MemReader r;
    r.data = buf;
    r.size = len;

    uint32_t magic   = r.R32();
    uint32_t version = r.R32();
    if (magic != FILE_MAGIC || version != FILE_VERSION) { return; }

    uint32_t node_count = r.R32();
    uint32_t root_id    = r.R32();
    uint32_t current_id = r.R32();
    r.R32();  // reserved
    r.R32();  // reserved

    Clear();
    m_root_id    = root_id;
    m_current_id = current_id;

    uint32_t max_id = ReadNodes(r, node_count, m_nodes);
    m_next_id       = max_id + 1;

    // Anchor: pool is provided by the caller (exported from BrepDB RTree)
    m_current_pool = std::make_shared<GeometryPool>(current_pool);
}

void VersionTree::Clear()
{
    m_nodes.clear();
    m_current_pool = {};
    m_current_id   = UINT32_MAX;
    m_root_id      = UINT32_MAX;
    m_next_id      = 0;
}

} // namespace brepdb
