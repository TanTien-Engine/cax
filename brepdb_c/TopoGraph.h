#pragma once

#include <cstdint>
#include <vector>
#include <span>
#include <unordered_map>

namespace brepdb
{

class TopoBlock
{
public:
    enum Rel : uint8_t
    {
        VertexOfEdge = 0,
        EdgeOfFace   = 2,
        FaceOfSolid  = 4,
    };

    struct Edge
    {
        uint32_t target;
        Rel      rel;
    };

    // ----- Build -----
    void SetSolidId(uint32_t id) { m_solid_id = id; }
    uint32_t GetSolidId() const { return m_solid_id; }

    void AddEdge(uint32_t parent_id, uint32_t child_id, Rel rel);
    void AddFaceAdjacency(uint32_t face_a, uint32_t face_b);

    void Finalize();
    void Clear();

    // ----- Query -----
    std::span<const Edge> GetChildren(uint32_t id) const;
    std::span<const Edge> GetParents(uint32_t id) const;

    std::span<const uint32_t> GetAdjacentFaces(uint32_t face_id) const;

    bool     HasNode(uint32_t id) const;
    uint32_t NodeCount() const { return static_cast<uint32_t>(m_node_ids.size()); }

    const std::vector<uint32_t>& GetNodeIds() const { return m_node_ids; }

    // ----- Serialization -----
    void StoreToByteArray(uint8_t** data, uint32_t& len) const;
    void LoadFromByteArray(const uint8_t* data, uint32_t len);

private:
    int32_t FindIndex(uint32_t id) const;
    int32_t FindFaceIndex(uint32_t face_id) const;

    uint32_t m_solid_id = 0;

    // Temp storage before Finalize
    struct TempEdge { uint32_t from, to; Rel rel; };
    std::vector<TempEdge> m_temp_edges;

    struct TempAdj { uint32_t a, b; };
    std::vector<TempAdj> m_temp_adj;

    // CSR for parent-child
    std::vector<uint32_t> m_node_ids;      // sorted
    std::vector<uint32_t> m_fwd_offsets;
    std::vector<Edge>     m_fwd_edges;
    std::vector<uint32_t> m_rev_offsets;
    std::vector<Edge>     m_rev_edges;

    // Face adjacency CSR
    std::vector<uint32_t> m_face_ids;       // sorted face ids
    std::vector<uint32_t> m_adj_offsets;     // face_count + 1
    std::vector<uint32_t> m_adj_faces;       // adjacent face ids

    bool m_finalized = false;

}; // TopoBlock

class TopoGraph
{
public:
    using Rel = TopoBlock::Rel;

    // ----- Build -----
    TopoBlock& CreateBlock(uint32_t solid_id);
    void FinalizeAll();
    void Clear();

    // ----- Query -----
    std::span<const TopoBlock::Edge> GetChildren(uint32_t id) const;
    std::span<const TopoBlock::Edge> GetParents(uint32_t id) const;
    std::span<const uint32_t> GetAdjacentFaces(uint32_t face_id) const;
    const TopoBlock* GetBlock(uint32_t solid_id) const;
    uint32_t FindSolidOf(uint32_t id) const;
    size_t BlockCount() const { return m_blocks.size(); }

    // ----- Serialization -----
    void StoreToByteArray(uint8_t** data, uint32_t& len) const;
    void LoadFromByteArray(const uint8_t* data, uint32_t len);

private:
    std::vector<TopoBlock> m_blocks;
    std::unordered_map<uint32_t, uint32_t> m_solid_to_block;
    std::unordered_map<uint32_t, uint32_t> m_entity_to_block;

}; // TopoGraph

}
