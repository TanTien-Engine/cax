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

void WrBytes(std::ofstream& os, const void* data, size_t n)
{
    if (n > 0) os.write(reinterpret_cast<const char*>(data),
                        static_cast<std::streamsize>(n));
}

void WrStr(std::ofstream& os, const std::string& s)
{
    Wr32(os, static_cast<uint32_t>(s.size()));
    WrBytes(os, s.data(), s.size());
}

void WrSnap(std::ofstream& os, const brepdb::EntitySnapshot& s)
{
    Wr32(os, s.id);
    Wr32(os, static_cast<uint32_t>(s.type));
    WrBytes(os, s.min_pt, 3 * sizeof(double));
    WrBytes(os, s.max_pt, 3 * sizeof(double));
    Wr32(os, static_cast<uint32_t>(s.params.size()));
    WrBytes(os, s.params.data(), s.params.size() * sizeof(double));
}

void WrHunks(std::ofstream& os, const std::vector<brepdb::ParamHunk>& hunks)
{
    Wr32(os, static_cast<uint32_t>(hunks.size()));
    for (const auto& h : hunks)
    {
        Wr32(os, h.offset);
        Wr32(os, static_cast<uint32_t>(h.data.size()));
        WrBytes(os, h.data.data(), h.data.size() * sizeof(double));
    }
}

void WrPatch(std::ofstream& os, const brepdb::ComponentPatch& p)
{
    Wr32(os, p.entity_id);
    Wr32(os, static_cast<uint32_t>(p.kind));
    Wr32(os, static_cast<uint32_t>(p.old_data.size()));
    WrBytes(os, p.old_data.data(), p.old_data.size());
    Wr32(os, static_cast<uint32_t>(p.new_data.size()));
    WrBytes(os, p.new_data.data(), p.new_data.size());
    Wr32(os, p.old_param_count);
    Wr32(os, p.new_param_count);
    WrHunks(os, p.forward_hunks);
    WrHunks(os, p.reverse_hunks);
}

void WrDiff(std::ofstream& os, const brepdb::ComponentDiff& d)
{
    Wr32(os, static_cast<uint32_t>(d.added.size()));
    for (const auto& s : d.added) WrSnap(os, s);

    Wr32(os, static_cast<uint32_t>(d.removed.size()));
    for (const auto& s : d.removed) WrSnap(os, s);

    Wr32(os, static_cast<uint32_t>(d.renamed.size()));
    for (const auto& [o, n] : d.renamed) { Wr32(os, o); Wr32(os, n); }

    Wr32(os, static_cast<uint32_t>(d.patches.size()));
    for (const auto& p : d.patches) WrPatch(os, p);

    Wr32(os, static_cast<uint32_t>(d.old_order.size()));
    WrBytes(os, d.old_order.data(), d.old_order.size() * sizeof(uint32_t));

    Wr32(os, static_cast<uint32_t>(d.new_order.size()));
    WrBytes(os, d.new_order.data(), d.new_order.size() * sizeof(uint32_t));
}

// ---- stream read helpers ----

uint32_t Rd32(std::ifstream& is) {
    uint32_t v; is.read(reinterpret_cast<char*>(&v), sizeof(v)); return v;
}

uint64_t Rd64(std::ifstream& is) {
    uint64_t v; is.read(reinterpret_cast<char*>(&v), sizeof(v)); return v;
}

void RdBytes(std::ifstream& is, void* dst, size_t n) {
    if (n > 0) is.read(reinterpret_cast<char*>(dst),
                       static_cast<std::streamsize>(n));
}

std::string RdStr(std::ifstream& is)
{
    uint32_t len = Rd32(is);
    std::string s(len, '\0');
    RdBytes(is, s.data(), len);
    return s;
}

brepdb::EntitySnapshot RdSnap(std::ifstream& is)
{
    brepdb::EntitySnapshot s;
    s.id = Rd32(is);
    s.type = static_cast<brepdb::Type>(Rd32(is));
    RdBytes(is, s.min_pt, 3 * sizeof(double));
    RdBytes(is, s.max_pt, 3 * sizeof(double));
    uint32_t pc = Rd32(is);
    s.params.resize(pc);
    RdBytes(is, s.params.data(), pc * sizeof(double));
    return s;
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
        RdBytes(is, hunks[j].data.data(), cnt * sizeof(double));
    }
    return hunks;
}

brepdb::ComponentPatch RdPatch(std::ifstream& is)
{
    brepdb::ComponentPatch p;
    p.entity_id = Rd32(is);
    p.kind = static_cast<brepdb::ComponentKind>(Rd32(is));
    uint32_t od = Rd32(is); p.old_data.resize(od); RdBytes(is, p.old_data.data(), od);
    uint32_t nd = Rd32(is); p.new_data.resize(nd); RdBytes(is, p.new_data.data(), nd);
    p.old_param_count = Rd32(is);
    p.new_param_count = Rd32(is);
    p.forward_hunks = RdHunks(is);
    p.reverse_hunks = RdHunks(is);
    return p;
}

brepdb::ComponentDiff RdDiff(std::ifstream& is)
{
    brepdb::ComponentDiff d;

    uint32_t na = Rd32(is);
    d.added.reserve(na);
    for (uint32_t j = 0; j < na; ++j) d.added.push_back(RdSnap(is));

    uint32_t nr = Rd32(is);
    d.removed.reserve(nr);
    for (uint32_t j = 0; j < nr; ++j) d.removed.push_back(RdSnap(is));

    uint32_t nrn = Rd32(is);
    d.renamed.reserve(nrn);
    for (uint32_t j = 0; j < nrn; ++j) {
        uint32_t o = Rd32(is), n = Rd32(is);
        d.renamed.emplace_back(o, n);
    }

    uint32_t np = Rd32(is);
    d.patches.reserve(np);
    for (uint32_t j = 0; j < np; ++j) d.patches.push_back(RdPatch(is));

    uint32_t oo = Rd32(is);
    d.old_order.resize(oo);
    RdBytes(is, d.old_order.data(), oo * sizeof(uint32_t));

    uint32_t no = Rd32(is);
    d.new_order.resize(no);
    RdBytes(is, d.new_order.data(), no * sizeof(uint32_t));

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
        WrBytes(os, min_pt, sizeof(min_pt));
        WrBytes(os, max_pt, sizeof(max_pt));
        auto params = world.ExportEntityParams(id);
        Wr32(os, static_cast<uint32_t>(params.size()));
        WrBytes(os, params.data(), params.size() * sizeof(double));
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
        RdBytes(is, min_pt, sizeof(min_pt));
        RdBytes(is, max_pt, sizeof(max_pt));
        uint32_t pc = Rd32(is);
        std::vector<double> params(pc);
        RdBytes(is, params.data(), pc * sizeof(double));

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
    w->RebuildTypedFromParams();
    return w;
}

// ---- memory buffer helpers ----

struct MemWriter
{
    std::vector<uint8_t> buf;

    void Write(const void* data, size_t size)
    {
        if (size == 0) return;
        const auto* p = reinterpret_cast<const uint8_t*>(data);
        buf.insert(buf.end(), p, p + size);
    }

    void W32(uint32_t v) { Write(&v, 4); }
    void W64(uint64_t v) { Write(&v, 8); }

    void WStr(const std::string& s)
    {
        W32(static_cast<uint32_t>(s.size()));
        Write(s.data(), s.size());
    }

    void WSnap(const brepdb::EntitySnapshot& s)
    {
        W32(s.id);
        W32(static_cast<uint32_t>(s.type));
        Write(s.min_pt, 3 * sizeof(double));
        Write(s.max_pt, 3 * sizeof(double));
        W32(static_cast<uint32_t>(s.params.size()));
        Write(s.params.data(), s.params.size() * sizeof(double));
    }

    void WHunks(const std::vector<brepdb::ParamHunk>& hunks)
    {
        W32(static_cast<uint32_t>(hunks.size()));
        for (const auto& h : hunks)
        {
            W32(h.offset);
            W32(static_cast<uint32_t>(h.data.size()));
            Write(h.data.data(), h.data.size() * sizeof(double));
        }
    }

    void WPatch(const brepdb::ComponentPatch& p)
    {
        W32(p.entity_id);
        W32(static_cast<uint32_t>(p.kind));
        W32(static_cast<uint32_t>(p.old_data.size()));
        Write(p.old_data.data(), p.old_data.size());
        W32(static_cast<uint32_t>(p.new_data.size()));
        Write(p.new_data.data(), p.new_data.size());
        W32(p.old_param_count);
        W32(p.new_param_count);
        WHunks(p.forward_hunks);
        WHunks(p.reverse_hunks);
    }

    void WDiff(const brepdb::ComponentDiff& d)
    {
        W32(static_cast<uint32_t>(d.added.size()));
        for (const auto& s : d.added) WSnap(s);

        W32(static_cast<uint32_t>(d.removed.size()));
        for (const auto& s : d.removed) WSnap(s);

        W32(static_cast<uint32_t>(d.renamed.size()));
        for (const auto& [o, n] : d.renamed) { W32(o); W32(n); }

        W32(static_cast<uint32_t>(d.patches.size()));
        for (const auto& p : d.patches) WPatch(p);

        W32(static_cast<uint32_t>(d.old_order.size()));
        Write(d.old_order.data(), d.old_order.size() * sizeof(uint32_t));

        W32(static_cast<uint32_t>(d.new_order.size()));
        Write(d.new_order.data(), d.new_order.size() * sizeof(uint32_t));
    }

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
            Write(params.data(), params.size() * sizeof(double));
        }
    }
};

struct MemReader
{
    const uint8_t* data;
    size_t         size;
    size_t         pos = 0;

    void Read(void* dst, size_t n)
    {
        if (n == 0) return;
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
        Read(s.data(), len);
        return s;
    }

    brepdb::EntitySnapshot RSnap()
    {
        brepdb::EntitySnapshot s;
        s.id = R32();
        s.type = static_cast<brepdb::Type>(R32());
        Read(s.min_pt, 3 * sizeof(double));
        Read(s.max_pt, 3 * sizeof(double));
        uint32_t pc = R32();
        s.params.resize(pc);
        Read(s.params.data(), pc * sizeof(double));
        return s;
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
            Read(hunks[j].data.data(), cnt * sizeof(double));
        }
        return hunks;
    }

    brepdb::ComponentPatch RPatch()
    {
        brepdb::ComponentPatch p;
        p.entity_id = R32();
        p.kind = static_cast<brepdb::ComponentKind>(R32());
        uint32_t od = R32(); p.old_data.resize(od); Read(p.old_data.data(), od);
        uint32_t nd = R32(); p.new_data.resize(nd); Read(p.new_data.data(), nd);
        p.old_param_count = R32();
        p.new_param_count = R32();
        p.forward_hunks = RHunks();
        p.reverse_hunks = RHunks();
        return p;
    }

    brepdb::ComponentDiff RDiff()
    {
        brepdb::ComponentDiff d;

        uint32_t na = R32();
        d.added.reserve(na);
        for (uint32_t j = 0; j < na; ++j) d.added.push_back(RSnap());

        uint32_t nr = R32();
        d.removed.reserve(nr);
        for (uint32_t j = 0; j < nr; ++j) d.removed.push_back(RSnap());

        uint32_t nrn = R32();
        d.renamed.reserve(nrn);
        for (uint32_t j = 0; j < nrn; ++j) {
            uint32_t o = R32(), n = R32();
            d.renamed.emplace_back(o, n);
        }

        uint32_t np = R32();
        d.patches.reserve(np);
        for (uint32_t j = 0; j < np; ++j) d.patches.push_back(RPatch());

        uint32_t oo = R32();
        d.old_order.resize(oo);
        Read(d.old_order.data(), oo * sizeof(uint32_t));

        uint32_t no = R32();
        d.new_order.resize(no);
        Read(d.new_order.data(), no * sizeof(uint32_t));

        return d;
    }

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
            Read(params.data(), pc * sizeof(double));

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
        w->RebuildTypedFromParams();
        return w;
    }
};

// BFS write of all nodes (DAG-safe, roots first)
void WriteNodesBfs(std::ofstream& os,
                   const std::unordered_map<uint32_t, brepdb::VersionNode>& nodes,
                   const std::vector<uint32_t>& root_ids,
                   bool include_diff)
{
    std::set<uint32_t> visited;
    std::queue<uint32_t> q;
    for (uint32_t rid : root_ids)
        if (rid != UINT32_MAX) { q.push(rid); visited.insert(rid); }

    while (!q.empty())
    {
        uint32_t id = q.front(); q.pop();
        auto it = nodes.find(id);
        if (it == nodes.end()) continue;
        const brepdb::VersionNode& node = it->second;

        Wr32(os, node.id);
        Wr32(os, node.parent_id);
        Wr32(os, static_cast<uint32_t>(node.aux_parent_ids.size()));
        for (uint32_t ap : node.aux_parent_ids) Wr32(os, ap);
        Wr32(os, static_cast<uint32_t>(node.children.size()));
        for (uint32_t c : node.children) Wr32(os, c);

        WrStr(os, node.op_desc);
        Wr32(os, node.op_type);
        Wr64(os, node.timestamp);

        if (include_diff && node.parent_id != UINT32_MAX) WrDiff(os, node.diff);

        for (uint32_t c : node.children)
            if (visited.insert(c).second) q.push(c);
    }
}

void WriteNodesBfs(MemWriter& w,
                   const std::unordered_map<uint32_t, brepdb::VersionNode>& nodes,
                   const std::vector<uint32_t>& root_ids)
{
    std::set<uint32_t> visited;
    std::queue<uint32_t> q;
    for (uint32_t rid : root_ids)
        if (rid != UINT32_MAX) { q.push(rid); visited.insert(rid); }

    while (!q.empty())
    {
        uint32_t id = q.front(); q.pop();
        auto it = nodes.find(id);
        if (it == nodes.end()) continue;
        const brepdb::VersionNode& node = it->second;

        w.W32(node.id);
        w.W32(node.parent_id);
        w.W32(static_cast<uint32_t>(node.aux_parent_ids.size()));
        for (uint32_t ap : node.aux_parent_ids) w.W32(ap);
        w.W32(static_cast<uint32_t>(node.children.size()));
        for (uint32_t c : node.children) w.W32(c);

        w.WStr(node.op_desc);
        w.W32(node.op_type);
        w.W64(node.timestamp);

        if (node.parent_id != UINT32_MAX) w.WDiff(node.diff);

        for (uint32_t c : node.children)
            if (visited.insert(c).second) q.push(c);
    }
}

uint32_t ReadNodes(std::ifstream& is,
                   uint32_t node_count,
                   std::unordered_map<uint32_t, brepdb::VersionNode>& out_nodes)
{
    uint32_t max_id = 0;
    for (uint32_t i = 0; i < node_count; ++i)
    {
        brepdb::VersionNode node;
        node.id = Rd32(is);
        node.parent_id = Rd32(is);

        uint32_t ac = Rd32(is);
        node.aux_parent_ids.resize(ac);
        for (uint32_t j = 0; j < ac; ++j) node.aux_parent_ids[j] = Rd32(is);

        uint32_t cc = Rd32(is);
        node.children.resize(cc);
        for (uint32_t j = 0; j < cc; ++j) node.children[j] = Rd32(is);

        node.op_desc = RdStr(is);
        node.op_type = Rd32(is);
        node.timestamp = Rd64(is);

        if (node.parent_id != UINT32_MAX) node.diff = RdDiff(is);

        if (node.id > max_id) max_id = node.id;
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
        node.id = r.R32();
        node.parent_id = r.R32();

        uint32_t ac = r.R32();
        node.aux_parent_ids.resize(ac);
        for (uint32_t j = 0; j < ac; ++j) node.aux_parent_ids[j] = r.R32();

        uint32_t cc = r.R32();
        node.children.resize(cc);
        for (uint32_t j = 0; j < cc; ++j) node.children[j] = r.R32();

        node.op_desc = r.RStr();
        node.op_type = r.R32();
        node.timestamp = r.R64();

        if (node.parent_id != UINT32_MAX) node.diff = r.RDiff();

        if (node.id > max_id) max_id = node.id;
        out_nodes[node.id] = std::move(node);
    }
    return max_id;
}

} // anonymous namespace


namespace brepdb
{

// ============================================================
// VersionTree - construction
// ============================================================

VersionTree::VersionTree() {}

uint32_t VersionTree::AddRoot(const BRepWorld&   world,
                               const std::string& op_desc,
                               uint32_t           op_type)
{
    uint32_t id = AllocNodeId();

    if (m_root_id == UINT32_MAX) m_root_id = id;

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
                              ComponentDiff&&     diff,
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
    ComponentDiff diff = ComponentDiff::Compute(*cursor.current_world, new_world);
    return Branch(cursor.current_id, new_world, std::move(diff), op_desc, op_type);
}

uint32_t VersionTree::Commit(uint32_t            root_id,
                              const BRepWorld&    new_world,
                              const PidMapping&   pid_map,
                              const std::string&  op_desc,
                              uint32_t            op_type)
{
    auto& cursor = m_cursors.at(root_id);
    ComponentDiff diff = ComponentDiff::ComputeWithPidMapping(
        *cursor.current_world, new_world, pid_map);
    return Branch(cursor.current_id, new_world, std::move(diff), op_desc, op_type);
}

uint32_t VersionTree::Branch(uint32_t            parent_id,
                              const BRepWorld&    new_world,
                              ComponentDiff&&     diff,
                              const std::string&  op_desc,
                              uint32_t            op_type)
{
    assert(m_nodes.count(parent_id));

    uint32_t root_id = FindRootOf(parent_id);
    auto& cursor = m_cursors.at(root_id);
    if (parent_id != cursor.current_id) NavigateTo(root_id, parent_id);

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
                             ComponentDiff&&                diff,
                             const std::string&             op_desc,
                             uint32_t                       op_type)
{
    assert(m_nodes.count(primary_parent_id));

    uint32_t root_id = FindRootOf(primary_parent_id);
    auto& cursor = m_cursors.at(root_id);
    if (primary_parent_id != cursor.current_id) NavigateTo(root_id, primary_parent_id);

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
    if (primary_parent_id != cursor.current_id) NavigateTo(root_id, primary_parent_id);

    ComponentDiff diff = ComponentDiff::ComputeWithPidMapping(
        *cursor.current_world, new_world, pid_map);
    return Merge(primary_parent_id, aux_parent_ids, new_world, std::move(diff), op_desc, op_type);
}

// ============================================================
// VersionTree - navigation
// ============================================================

WorldPtr VersionTree::Checkout(uint32_t root_id, uint32_t node_id)
{
    assert(m_nodes.count(node_id));
    auto& cursor = m_cursors.at(root_id);
    if (node_id != cursor.current_id) NavigateTo(root_id, node_id);
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
    uint32_t target = (child_index < 0 || child_index >= static_cast<int>(children.size()))
                      ? children.back() : children[child_index];
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
        if (node.parent_id == UINT32_MAX) roots.push_back(id);
    return roots;
}

uint32_t VersionTree::FindRootOf(uint32_t node_id) const
{
    uint32_t cur = node_id;
    while (true)
    {
        auto it = m_nodes.find(cur);
        if (it == m_nodes.end()) return UINT32_MAX;
        if (it->second.parent_id == UINT32_MAX) return cur;
        cur = it->second.parent_id;
    }
}

std::vector<uint32_t> VersionTree::GetAllCurrentIds() const
{
    std::vector<uint32_t> ids;
    for (const auto& [root_id, cursor] : m_cursors) ids.push_back(cursor.current_id);
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
    if (cit == m_cursors.end()) return false;
    auto nit = m_nodes.find(cit->second.current_id);
    return nit != m_nodes.end() && nit->second.parent_id != UINT32_MAX;
}

bool VersionTree::CanRedo(uint32_t root_id) const
{
    auto cit = m_cursors.find(root_id);
    if (cit == m_cursors.end()) return false;
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
        if (it == m_nodes.end()) break;
        cur = it->second.parent_id;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<uint32_t> VersionTree::GetLeaves() const
{
    std::vector<uint32_t> leaves;
    for (const auto& [id, node] : m_nodes)
        if (node.children.empty()) leaves.push_back(id);
    return leaves;
}

void VersionTree::TraverseAll(const std::function<void(const VersionNode&)>& visitor) const
{
    if (m_nodes.empty()) return;
    std::set<uint32_t> visited;
    std::queue<uint32_t> bfs;

    for (const auto& [id, node] : m_nodes)
        if (node.parent_id == UINT32_MAX) { bfs.push(id); visited.insert(id); }

    while (!bfs.empty())
    {
        uint32_t id = bfs.front(); bfs.pop();
        auto it = m_nodes.find(id);
        if (it == m_nodes.end()) continue;
        visitor(it->second);
        for (uint32_t c : it->second.children)
            if (visited.insert(c).second) bfs.push(c);
    }
}

// ============================================================
// VersionTree - internal navigation
// ============================================================

uint32_t VersionTree::AllocNodeId() { return m_next_id++; }

void VersionTree::NavigateTo(uint32_t root_id, uint32_t target_id)
{
    auto& cursor = m_cursors.at(root_id);
    if (cursor.current_id == target_id) return;

    uint32_t lca = FindLCA(cursor.current_id, target_id);

    WorldPtr world = cursor.current_world;

    uint32_t cur = cursor.current_id;
    while (cur != lca)
    {
        auto it = m_nodes.find(cur);
        world = ComponentDiff::ApplyReverse(*world, it->second.diff);
        cur = it->second.parent_id;
    }

    auto tgt_path = GetPathFromRoot(target_id);
    auto lca_it = std::find(tgt_path.begin(), tgt_path.end(), lca);
    assert(lca_it != tgt_path.end());

    for (auto it = lca_it + 1; it != tgt_path.end(); ++it)
    {
        auto node_it = m_nodes.find(*it);
        world = ComponentDiff::ApplyForward(*world, node_it->second.diff);
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
        if (path_a[i] == path_b[i]) lca = path_a[i];
        else break;
    }
    return lca;
}

// ============================================================
// VersionTree — persistence (file)
// ============================================================

bool VersionTree::SaveToFile(const std::string& filepath) const
{
    std::ofstream os(filepath, std::ios::binary);
    if (!os) return false;

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
                root_world = ComponentDiff::ApplyReverse(*root_world, it->second.diff);
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
    if (!is) return false;

    FileHeader fh;
    is.read(reinterpret_cast<char*>(&fh), sizeof(fh));
    if (fh.magic != FILE_MAGIC || fh.version != FILE_VERSION) return false;

    Clear();
    m_root_id = fh.root_id;

    uint32_t root_count = Rd32(is);
    for (uint32_t i = 0; i < root_count; ++i)
    {
        uint32_t rid = Rd32(is);
        m_root_worlds[rid] = RdWorld(is);
    }

    uint32_t cursor_count = Rd32(is);
    for (uint32_t i = 0; i < cursor_count; ++i)
    {
        uint32_t rid = Rd32(is);
        uint32_t cid = Rd32(is);
        RootCursor cursor;
        cursor.current_id = cid;
        m_cursors[rid] = cursor;
    }

    uint32_t max_id = ReadNodes(is, fh.node_count, m_nodes);
    m_next_id       = max_id + 1;

    for (auto& [rid, cursor] : m_cursors)
    {
        auto rw_it = m_root_worlds.find(rid);
        if (rw_it == m_root_worlds.end()) continue;

        WorldPtr world = rw_it->second;
        auto path = GetPathFromRoot(cursor.current_id);
        for (size_t j = 1; j < path.size(); ++j)
        {
            auto node_it = m_nodes.find(path[j]);
            world = ComponentDiff::ApplyForward(*world, node_it->second.diff);
        }
        cursor.current_world = world;
    }

    return true;
}

// ============================================================
// VersionTree — persistence (byte array)
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
                root_world = ComponentDiff::ApplyReverse(*root_world, it->second.diff);
            }
            w.WWorld(*root_world);
        }
    }

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

    uint32_t magic   = r.R32();
    uint32_t version = r.R32();
    if (magic != FILE_MAGIC || version != FILE_VERSION) return;

    uint32_t node_count   = r.R32();
    uint32_t root_id      = r.R32();
    uint32_t cursor_count = r.R32();
    r.R32();
    r.R32();

    Clear();
    m_root_id = root_id;

    uint32_t root_count = r.R32();
    for (uint32_t i = 0; i < root_count; ++i)
    {
        uint32_t rid = r.R32();
        m_root_worlds[rid] = r.RWorld();
    }

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

    for (auto& [rid, cursor] : m_cursors)
    {
        auto rw_it = m_root_worlds.find(rid);
        if (rw_it == m_root_worlds.end()) continue;

        WorldPtr world = rw_it->second;
        auto path = GetPathFromRoot(cursor.current_id);
        for (size_t j = 1; j < path.size(); ++j)
        {
            auto node_it = m_nodes.find(path[j]);
            world = ComponentDiff::ApplyForward(*world, node_it->second.diff);
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
