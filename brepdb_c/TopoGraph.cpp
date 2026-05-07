#include "brepdb_c/TopoGraph.h"

#include <algorithm>
#include <cstring>
#include <cassert>
#include <thread>
#include <set>

namespace brepdb
{

static const uint32_t PACKED_EDGE_SIZE = sizeof(uint32_t) + sizeof(uint8_t);

// ============================================================
// TopoBlock
// ============================================================

void TopoBlock::AddEdge(uint32_t parent_id, uint32_t child_id, Rel rel)
{
    m_temp_edges.push_back({ parent_id, child_id, rel });
}

void TopoBlock::AddFaceAdjacency(uint32_t face_a, uint32_t face_b)
{
    if (face_a == face_b) 
        return;
    m_temp_adj.push_back({ face_a, face_b });
    m_temp_adj.push_back({ face_b, face_a });
}

void TopoBlock::Finalize()
{
    std::vector<uint32_t> ids;
    ids.reserve(m_temp_edges.size() * 2);
    for (const auto& e : m_temp_edges) 
    {
        ids.push_back(e.from);
        ids.push_back(e.to);
    }
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    m_node_ids = std::move(ids);

    const uint32_t n = static_cast<uint32_t>(m_node_ids.size());
    const uint32_t ec = static_cast<uint32_t>(m_temp_edges.size());

    // --- Forward CSR ---
    m_fwd_offsets.assign(n + 1, 0);
    m_fwd_edges.resize(ec);
    for (const auto& e : m_temp_edges)
        m_fwd_offsets[FindIndex(e.from) + 1]++;
    for (uint32_t i = 1; i <= n; ++i)
        m_fwd_offsets[i] += m_fwd_offsets[i - 1];

    std::vector<uint32_t> cursor(m_fwd_offsets.begin(), m_fwd_offsets.end());
    for (const auto& e : m_temp_edges) 
    {
        uint32_t pos = cursor[FindIndex(e.from)]++;
        m_fwd_edges[pos] = Edge{ e.to, e.rel };
    }

    // --- Reverse CSR ---
    m_rev_offsets.assign(n + 1, 0);
    m_rev_edges.resize(ec);
    for (const auto& e : m_temp_edges)
        m_rev_offsets[FindIndex(e.to) + 1]++;
    for (uint32_t i = 1; i <= n; ++i)
        m_rev_offsets[i] += m_rev_offsets[i - 1];

    cursor.assign(m_rev_offsets.begin(), m_rev_offsets.end());
    for (const auto& e : m_temp_edges) 
    {
        uint32_t pos = cursor[FindIndex(e.to)]++;
        m_rev_edges[pos] = Edge{ e.from, e.rel };
    }

    m_temp_edges.clear();
    m_temp_edges.shrink_to_fit();

    // --- Face adjacency CSR ---
    std::vector<uint32_t> fids;
    fids.reserve(m_temp_adj.size());
    for (const auto& a : m_temp_adj)
        fids.push_back(a.a);
    std::sort(fids.begin(), fids.end());
    fids.erase(std::unique(fids.begin(), fids.end()), fids.end());
    m_face_ids = std::move(fids);

    std::sort(m_temp_adj.begin(), m_temp_adj.end(),
        [](const TempAdj& x, const TempAdj& y) {
            return x.a < y.a || (x.a == y.a && x.b < y.b);
        });
    m_temp_adj.erase(std::unique(m_temp_adj.begin(), m_temp_adj.end(),
        [](const TempAdj& x, const TempAdj& y) {
            return x.a == y.a && x.b == y.b;
        }), m_temp_adj.end());

    uint32_t fn = static_cast<uint32_t>(m_face_ids.size());
    m_adj_offsets.assign(fn + 1, 0);
    m_adj_faces.resize(m_temp_adj.size());

    for (const auto& a : m_temp_adj)
        m_adj_offsets[FindFaceIndex(a.a) + 1]++;
    for (uint32_t i = 1; i <= fn; ++i)
        m_adj_offsets[i] += m_adj_offsets[i - 1];

    cursor.assign(m_adj_offsets.begin(), m_adj_offsets.end());
    for (const auto& a : m_temp_adj) 
    {
        uint32_t pos = cursor[FindFaceIndex(a.a)]++;
        m_adj_faces[pos] = a.b;
    }

    m_temp_adj.clear();
    m_temp_adj.shrink_to_fit();

    m_finalized = true;
}

void TopoBlock::Clear()
{
    m_node_ids.clear(); 
    m_fwd_offsets.clear(); 
    m_fwd_edges.clear();
    m_rev_offsets.clear(); 
    m_rev_edges.clear();
    
    m_face_ids.clear(); 
    m_adj_offsets.clear(); 
    m_adj_faces.clear();

    m_temp_edges.clear(); 
    m_temp_adj.clear();

    m_finalized = false;
}

int32_t TopoBlock::FindIndex(uint32_t id) const
{
    auto it = std::lower_bound(m_node_ids.begin(), m_node_ids.end(), id);
    if (it == m_node_ids.end() || *it != id) 
        return -1;
    return static_cast<int32_t>(it - m_node_ids.begin());
}

int32_t TopoBlock::FindFaceIndex(uint32_t face_id) const
{
    auto it = std::lower_bound(m_face_ids.begin(), m_face_ids.end(), face_id);
    if (it == m_face_ids.end() || *it != face_id) 
        return -1;
    return static_cast<int32_t>(it - m_face_ids.begin());
}

bool TopoBlock::HasNode(uint32_t id) const 
{ 
    return FindIndex(id) >= 0; 
}

std::span<const TopoBlock::Edge> TopoBlock::GetChildren(uint32_t id) const
{
    int32_t idx = FindIndex(id);
    if (idx < 0) 
        return {};
    return { &m_fwd_edges[m_fwd_offsets[idx]], m_fwd_offsets[idx+1] - m_fwd_offsets[idx] };
}

std::span<const TopoBlock::Edge> TopoBlock::GetParents(uint32_t id) const
{
    int32_t idx = FindIndex(id);
    if (idx < 0) 
        return {};
    return { &m_rev_edges[m_rev_offsets[idx]], m_rev_offsets[idx+1] - m_rev_offsets[idx] };
}

std::span<const uint32_t> TopoBlock::GetAdjacentFaces(uint32_t face_id) const
{
    int32_t idx = FindFaceIndex(face_id);
    if (idx < 0) 
        return {};
    return { &m_adj_faces[m_adj_offsets[idx]], m_adj_offsets[idx+1] - m_adj_offsets[idx] };
}

//
// Format:
//   [solid_id: u32] [node_count: u32] [edge_count: u32]
//   [face_count: u32] [adj_count: u32]
//   [node_ids: node_count * u32]
//   [fwd_offsets: (node_count+1) * u32]
//   [fwd_edges:  edge_count * (u32 + u8)]
//   [rev_offsets: (node_count+1) * u32]
//   [rev_edges:  edge_count * (u32 + u8)]
//   [face_ids:   face_count * u32]
//   [adj_offsets: (face_count+1) * u32]
//   [adj_faces:  adj_count * u32]
//

void TopoBlock::StoreToByteArray(uint8_t** data, uint32_t& len) const
{
    const uint32_t nc = static_cast<uint32_t>(m_node_ids.size());
    const uint32_t ec = static_cast<uint32_t>(m_fwd_edges.size());
    const uint32_t fc = static_cast<uint32_t>(m_face_ids.size());
    const uint32_t ac = static_cast<uint32_t>(m_adj_faces.size());

    len = sizeof(uint32_t) * 5                  // header
        + nc * sizeof(uint32_t)                 // node_ids
        + (nc + 1) * sizeof(uint32_t) * 2       // fwd + rev offsets
        + ec * PACKED_EDGE_SIZE * 2             // fwd + rev edges
        + fc * sizeof(uint32_t)                 // face_ids
        + (fc + 1) * sizeof(uint32_t)           // adj_offsets
        + ac * sizeof(uint32_t);                // adj_faces

    *data = new uint8_t[len];
    uint8_t* ptr = *data;

    auto W = [&](const void* src, size_t sz) { 
        memcpy(ptr, src, sz); ptr += sz; 
    };
    auto W32 = [&](uint32_t v) { 
        W(&v, 4); 
    };

    W32(m_solid_id); 
    W32(nc); 
    W32(ec); 
    W32(fc); 
    W32(ac);

    if (nc) 
        W(m_node_ids.data(), nc * 4);
    if (nc) 
        W(m_fwd_offsets.data(), (nc + 1) * 4);

    for (uint32_t i = 0; i < ec; ++i) 
    {
        W(&m_fwd_edges[i].target, 4);
        *ptr++ = static_cast<uint8_t>(m_fwd_edges[i].rel);
    }

    if (nc) 
        W(m_rev_offsets.data(), (nc + 1) * 4);

    for (uint32_t i = 0; i < ec; ++i) 
    {
        W(&m_rev_edges[i].target, 4);
        *ptr++ = static_cast<uint8_t>(m_rev_edges[i].rel);
    }

    if (fc) 
        W(m_face_ids.data(), fc * 4);
    if (fc) 
        W(m_adj_offsets.data(), (fc + 1) * 4);
    if (ac) 
        W(m_adj_faces.data(), ac * 4);
}

void TopoBlock::LoadFromByteArray(const uint8_t* data, uint32_t len)
{
    Clear();
    const uint8_t* ptr = data;

    auto R = [&](void* dst, size_t sz) { 
        memcpy(dst, ptr, sz); ptr += sz; 
    };
    auto R32 = [&]() -> uint32_t 
    { 
        uint32_t v; 
        R(&v, 4); 
        return v; 
    };

    m_solid_id = R32();
    uint32_t nc = R32(), ec = R32(), fc = R32(), ac = R32();

    m_node_ids.resize(nc);
    if (nc) 
        R(m_node_ids.data(), nc * 4);

    m_fwd_offsets.resize(nc + 1);
    if (nc) 
        R(m_fwd_offsets.data(), (nc + 1) * 4);

    m_fwd_edges.resize(ec);
    for (uint32_t i = 0; i < ec; ++i) 
    {
        R(&m_fwd_edges[i].target, 4);
        m_fwd_edges[i].rel = static_cast<Rel>(*ptr++);
    }

    m_rev_offsets.resize(nc + 1);
    if (nc) 
        R(m_rev_offsets.data(), (nc + 1) * 4);

    m_rev_edges.resize(ec);
    for (uint32_t i = 0; i < ec; ++i) 
    {
        R(&m_rev_edges[i].target, 4);
        m_rev_edges[i].rel = static_cast<Rel>(*ptr++);
    }

    m_face_ids.resize(fc);
    if (fc) 
        R(m_face_ids.data(), fc * 4);

    m_adj_offsets.resize(fc + 1);
    if (fc) 
        R(m_adj_offsets.data(), (fc + 1) * 4);

    m_adj_faces.resize(ac);
    if (ac) 
        R(m_adj_faces.data(), ac * 4);

    m_finalized = true;
}


// ============================================================
// TopoGraph
// ============================================================

TopoBlock& TopoGraph::CreateBlock(uint32_t solid_id)
{
    uint32_t idx = static_cast<uint32_t>(m_blocks.size());
    m_blocks.emplace_back();
    m_blocks.back().SetSolidId(solid_id);
    m_solid_to_block[solid_id] = idx;
    return m_blocks.back();
}

void TopoGraph::FinalizeAll()
{
    uint32_t n = static_cast<uint32_t>(m_blocks.size());
    uint32_t hw = std::thread::hardware_concurrency();
    uint32_t num_threads = std::min(hw > 0 ? hw : 4, n);

    if (num_threads <= 1 || n <= 4) 
    {
        for (auto& b : m_blocks) 
            b.Finalize();
    } 
    else 
    {
        std::vector<std::thread> threads;
        uint32_t chunk = (n + num_threads - 1) / num_threads;
        for (uint32_t t = 0; t < num_threads; ++t) 
        {
            uint32_t begin = t * chunk;
            uint32_t end = std::min(begin + chunk, n);
            if (begin < end) {
                threads.emplace_back([&, begin, end]() {
                    for (uint32_t i = begin; i < end; ++i) m_blocks[i].Finalize();
                });
            }
        }
        for (auto& t : threads) 
            t.join();
    }

    m_entity_to_block.clear();
    for (uint32_t i = 0; i < n; ++i)
        for (uint32_t nid : m_blocks[i].GetNodeIds())
            m_entity_to_block[nid] = i;
}

void TopoGraph::Clear()
{
    m_blocks.clear();
    m_solid_to_block.clear();
    m_entity_to_block.clear();
}

const TopoBlock* TopoGraph::GetBlock(uint32_t solid_id) const
{
    auto it = m_solid_to_block.find(solid_id);
    return (it != m_solid_to_block.end()) ? &m_blocks[it->second] : nullptr;
}

uint32_t TopoGraph::FindSolidOf(uint32_t id) const
{
    auto it = m_entity_to_block.find(id);
    return (it != m_entity_to_block.end()) ? m_blocks[it->second].GetSolidId() : UINT32_MAX;
}

std::span<const TopoBlock::Edge> TopoGraph::GetChildren(uint32_t id) const
{
    auto it = m_entity_to_block.find(id);
    return (it != m_entity_to_block.end()) ? m_blocks[it->second].GetChildren(id) : std::span<const TopoBlock::Edge>{};
}

std::span<const TopoBlock::Edge> TopoGraph::GetParents(uint32_t id) const
{
    auto it = m_entity_to_block.find(id);
    return (it != m_entity_to_block.end()) ? m_blocks[it->second].GetParents(id) : std::span<const TopoBlock::Edge>{};
}

std::span<const uint32_t> TopoGraph::GetAdjacentFaces(uint32_t face_id) const
{
    auto it = m_entity_to_block.find(face_id);
    return (it != m_entity_to_block.end()) ? m_blocks[it->second].GetAdjacentFaces(face_id) : std::span<const uint32_t>{};
}

// Format:
//   [magic: u32 = 0x544F504F]
//   [version: u32 = 2]
//   [block_count: u32]
//   [block_sizes: block_count * u32]
//   [block_0 bytes...]
//   [block_1 bytes...]
//   ...

static const uint32_t TOPO_MAGIC   = 0x544F504F;
static const uint32_t TOPO_VERSION = 2;

void TopoGraph::StoreToByteArray(uint8_t** data, uint32_t& len) const
{
    uint32_t bc = static_cast<uint32_t>(m_blocks.size());

    std::vector<uint8_t*> bufs(bc);
    std::vector<uint32_t> lens(bc);
    for (uint32_t i = 0; i < bc; ++i)
        m_blocks[i].StoreToByteArray(&bufs[i], lens[i]);

    uint32_t header_sz = sizeof(uint32_t) * (3 + bc);
    uint32_t total_blocks = 0;
    for (uint32_t i = 0; i < bc; ++i) 
        total_blocks += lens[i];

    len = header_sz + total_blocks;
    *data = new uint8_t[len];
    uint8_t* ptr = *data;

    auto W = [&](const void* src, size_t sz) { 
        memcpy(ptr, src, sz); ptr += sz; 
    };
    auto W32 = [&](uint32_t v) { 
        W(&v, 4); 
    };

    W32(TOPO_MAGIC); 
    W32(TOPO_VERSION); 
    W32(bc);
    for (uint32_t i = 0; i < bc; ++i) 
        W32(lens[i]);
    for (uint32_t i = 0; i < bc; ++i) 
    {
        memcpy(ptr, bufs[i], lens[i]);
        ptr += lens[i];
        delete[] bufs[i];
    }
}

void TopoGraph::LoadFromByteArray(const uint8_t* data, uint32_t len)
{
    Clear();
    if (len < 12) 
        return;
    const uint8_t* ptr = data;

    auto R32 = [&]() -> uint32_t 
    { 
        uint32_t v; 
        memcpy(&v, ptr, 4); 
        ptr += 4; 
        return v; 
    };

    uint32_t magic = R32();
    if (magic != TOPO_MAGIC) 
        return;
    R32(); // version
    uint32_t bc = R32();

    std::vector<uint32_t> sizes(bc);
    for (uint32_t i = 0; i < bc; ++i) 
        sizes[i] = R32();

    m_blocks.resize(bc);
    for (uint32_t i = 0; i < bc; ++i) 
    {
        m_blocks[i].LoadFromByteArray(ptr, sizes[i]);
        ptr += sizes[i];
        m_solid_to_block[m_blocks[i].GetSolidId()] = i;
    }

    for (uint32_t i = 0; i < bc; ++i)
        for (uint32_t nid : m_blocks[i].GetNodeIds())
            m_entity_to_block[nid] = i;
}

}
