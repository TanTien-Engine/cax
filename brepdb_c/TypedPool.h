#pragma once

#include "GeomPool.h"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

namespace brepdb
{

// Sparse-set backed component pool. O(1) add/remove/lookup.
// T must be trivially copyable or a POD-like struct.
template<typename T>
class ComponentPool
{
public:
    bool Has(uint32_t entity_id) const
    {
        auto it = m_sparse.find(entity_id);
        return it != m_sparse.end();
    }

    T* Get(uint32_t entity_id)
    {
        auto it = m_sparse.find(entity_id);
        if (it == m_sparse.end()) return nullptr;
        return &m_dense[it->second];
    }

    const T* Get(uint32_t entity_id) const
    {
        auto it = m_sparse.find(entity_id);
        if (it == m_sparse.end()) return nullptr;
        return &m_dense[it->second];
    }

    void Set(uint32_t entity_id, const T& comp)
    {
        auto it = m_sparse.find(entity_id);
        if (it != m_sparse.end())
        {
            m_dense[it->second] = comp;
        }
        else
        {
            m_sparse[entity_id] = static_cast<uint32_t>(m_dense.size());
            m_dense.push_back(comp);
            m_dense_ids.push_back(entity_id);
        }
    }

    void Remove(uint32_t entity_id)
    {
        auto it = m_sparse.find(entity_id);
        if (it == m_sparse.end()) return;

        uint32_t idx = it->second;
        uint32_t last = static_cast<uint32_t>(m_dense.size()) - 1;

        if (idx != last)
        {
            m_dense[idx]     = m_dense[last];
            m_dense_ids[idx] = m_dense_ids[last];
            m_sparse[m_dense_ids[idx]] = idx;
        }

        m_dense.pop_back();
        m_dense_ids.pop_back();
        m_sparse.erase(it);
    }

    void Clear()
    {
        m_dense.clear();
        m_dense_ids.clear();
        m_sparse.clear();
    }

    size_t Size() const { return m_dense.size(); }

    // Iteration: walk dense[] linearly for cache-perfect traversal.
    const T*        Data()      const { return m_dense.data(); }
    const uint32_t* EntityIds() const { return m_dense_ids.data(); }

    T*        MutableData()      { return m_dense.data(); }
    uint32_t* MutableEntityIds() { return m_dense_ids.data(); }

private:
    std::vector<T>        m_dense;
    std::vector<uint32_t> m_dense_ids;
    std::unordered_map<uint32_t, uint32_t> m_sparse;
};

// ============================================================
// Typed component structs
// ============================================================

struct PositionComp
{
    double x, y, z;
};

struct AabbComp
{
    double min_pt[3];
    double max_pt[3];
};

struct ToleranceComp
{
    double value;
};

// Variable-length geometry params (curve / surface control points etc.)
struct ParamsComp
{
    std::vector<double> data;
};

// Curve geometry (for edges)
struct CurveComp
{
    Type curve_type = Type::Empty;  // Line, Circle, BSplineCurve, or Empty
    std::vector<double> data;       // serialized curve params
};

// 2D curve on surface (pcurve, for edge-face pairs)
struct Curve2dComp
{
    Type curve_type = Type::Empty;
    double first = 0, last = 0;
    std::vector<double> data;
};

// Surface geometry (for faces)
struct SurfaceComp
{
    Type surface_type = Type::Empty;  // Plane, Cylinder, BSplineSurface
    std::vector<double> data;         // serialized surface params
};

// Edge topology: references to vertices + curve range
struct EdgeTopoComp
{
    uint32_t v_first = UINT32_MAX;
    uint32_t v_last  = UINT32_MAX;
    double   t_first = 0;
    double   t_last  = 0;
};

// Face topology: orientation + wire structure
struct FaceTopoComp
{
    uint8_t orientation = 0;

    // Outer wire
    bool has_outer_wire = false;
    uint8_t outer_wire_orientation = 0;
    struct WireEdgeRef
    {
        uint32_t edge_uid;
        uint8_t  orientation;
        Curve2dComp pcurve;
    };
    std::vector<WireEdgeRef> outer_wire_edges;

    // Inner wires
    struct WireComp
    {
        uint8_t orientation = 0;
        std::vector<WireEdgeRef> edges;
    };
    std::vector<WireComp> inner_wires;
};

// Solid topology: shells containing face references
struct SolidTopoComp
{
    struct ShellComp
    {
        uint8_t orientation = 0;
        std::vector<uint32_t> face_uids;
    };
    std::vector<ShellComp> shells;
};

// ============================================================
// BRepWorld: the ECS container holding all typed pools.
// ============================================================

class BRepWorld
{
public:
    // Entity management
    uint32_t CreateEntity()
    {
        uint32_t id = m_next_id++;
        m_alive.push_back(id);
        return id;
    }

    void DestroyEntity(uint32_t id)
    {
        m_types.Remove(id);
        m_aabbs.Remove(id);
        m_positions.Remove(id);
        m_tolerances.Remove(id);
        m_params.Remove(id);

        auto it = std::find(m_alive.begin(), m_alive.end(), id);
        if (it != m_alive.end())
        {
            *it = m_alive.back();
            m_alive.pop_back();
        }
    }

    bool IsAlive(uint32_t id) const
    {
        return std::find(m_alive.begin(), m_alive.end(), id) != m_alive.end();
    }

    size_t EntityCount() const { return m_alive.size(); }
    const std::vector<uint32_t>& AliveEntities() const { return m_alive; }

    // Register entity with known id (for import scenarios)
    void RegisterEntity(uint32_t id)
    {
        if (!IsAlive(id))
            m_alive.push_back(id);
        if (id >= m_next_id)
            m_next_id = id + 1;
    }

    // Component access
    ComponentPool<Type>&          Types()       { return m_types; }
    ComponentPool<AabbComp>&      Aabbs()       { return m_aabbs; }
    ComponentPool<PositionComp>&  Positions()   { return m_positions; }
    ComponentPool<ToleranceComp>& Tolerances()  { return m_tolerances; }
    ComponentPool<ParamsComp>&    Params()      { return m_params; }
    ComponentPool<CurveComp>&     Curves()      { return m_curves; }
    ComponentPool<SurfaceComp>&   Surfaces()    { return m_surfaces; }
    ComponentPool<EdgeTopoComp>&  EdgeTopos()   { return m_edge_topos; }
    ComponentPool<FaceTopoComp>&  FaceTopos()   { return m_face_topos; }
    ComponentPool<SolidTopoComp>& SolidTopos()  { return m_solid_topos; }

    const ComponentPool<Type>&          Types()       const { return m_types; }
    const ComponentPool<AabbComp>&      Aabbs()       const { return m_aabbs; }
    const ComponentPool<PositionComp>&  Positions()   const { return m_positions; }
    const ComponentPool<ToleranceComp>& Tolerances()  const { return m_tolerances; }
    const ComponentPool<ParamsComp>&    Params()      const { return m_params; }
    const ComponentPool<CurveComp>&     Curves()      const { return m_curves; }
    const ComponentPool<SurfaceComp>&   Surfaces()    const { return m_surfaces; }
    const ComponentPool<EdgeTopoComp>&  EdgeTopos()   const { return m_edge_topos; }
    const ComponentPool<FaceTopoComp>&  FaceTopos()   const { return m_face_topos; }
    const ComponentPool<SolidTopoComp>& SolidTopos()  const { return m_solid_topos; }

    // Import from legacy GeometryPool
    void ImportFromPool(const GeometryPool& pool)
    {
        for (size_t i = 0; i < pool.headers.size(); ++i)
        {
            const auto& h = pool.headers[i];
            uint32_t id = h.persistent_id;

            if (!IsAlive(id))
                m_alive.push_back(id);
            if (id >= m_next_id)
                m_next_id = id + 1;

            m_types.Set(id, h.type);

            AabbComp aabb;
            std::memcpy(aabb.min_pt, h.min_pt, 24);
            std::memcpy(aabb.max_pt, h.max_pt, 24);
            m_aabbs.Set(id, aabb);

            if (h.param_count > 0)
            {
                ParamsComp pc;
                pc.data.assign(pool.data_pool.begin() + h.param_offset,
                               pool.data_pool.begin() + h.param_offset + h.param_count);
                m_params.Set(id, pc);
            }
        }
    }

    // Export back to legacy GeometryPool.
    // If typed components (Curves, Surfaces, etc.) are populated,
    // reconstructs the flat data_pool in GeomSender format.
    // Falls back to ParamsComp if no typed components exist.
    GeometryPool ExportToPool() const
    {
        GeometryPool pool;
        for (uint32_t id : m_alive)
        {
            GeomHeader h{};
            h.persistent_id = id;

            const Type* t = m_types.Get(id);
            if (t) h.type = *t;

            const AabbComp* aabb = m_aabbs.Get(id);
            if (aabb)
            {
                std::memcpy(h.min_pt, aabb->min_pt, 24);
                std::memcpy(h.max_pt, aabb->max_pt, 24);
            }

            h.param_offset = static_cast<uint32_t>(pool.data_pool.size());

            // Try typed export first
            bool has_typed = false;
            if (t)
            {
                switch (*t)
                {
                case Type::Vertex:
                    has_typed = ExportVertex(id, pool.data_pool);
                    break;
                case Type::Edge:
                    has_typed = ExportEdge(id, pool.data_pool);
                    break;
                case Type::Face:
                    has_typed = ExportFace(id, pool.data_pool);
                    break;
                case Type::Solid:
                    has_typed = ExportSolid(id, pool.data_pool);
                    break;
                default:
                    break;
                }
            }

            if (!has_typed)
            {
                const ParamsComp* pc = m_params.Get(id);
                if (pc && !pc->data.empty())
                    pool.data_pool.insert(pool.data_pool.end(), pc->data.begin(), pc->data.end());
            }

            h.param_count = static_cast<uint32_t>(pool.data_pool.size()) - h.param_offset;
            pool.headers.push_back(h);
        }
        return pool;
    }

private:
    // Reconstruct flat data_pool segments from typed components
    bool ExportVertex(uint32_t id, std::vector<double>& d) const
    {
        const PositionComp* pos = m_positions.Get(id);
        if (!pos) return false;
        d.push_back(pos->x);
        d.push_back(pos->y);
        d.push_back(pos->z);
        const ToleranceComp* tol = m_tolerances.Get(id);
        d.push_back(tol ? tol->value : 0.0);
        return true;
    }

    bool ExportEdge(uint32_t id, std::vector<double>& d) const
    {
        const EdgeTopoComp* et = m_edge_topos.Get(id);
        if (!et) return false;
        d.push_back(et->v_first == UINT32_MAX ? -1.0 : static_cast<double>(et->v_first));
        d.push_back(et->v_last  == UINT32_MAX ? -1.0 : static_cast<double>(et->v_last));
        const ToleranceComp* tol = m_tolerances.Get(id);
        d.push_back(tol ? tol->value : 0.0);
        d.push_back(et->t_first);
        d.push_back(et->t_last);
        const CurveComp* curve = m_curves.Get(id);
        if (curve)
        {
            d.push_back(static_cast<double>(curve->curve_type));
            d.insert(d.end(), curve->data.begin(), curve->data.end());
        }
        else
        {
            d.push_back(static_cast<double>(Type::Empty));
        }
        return true;
    }

    bool ExportFace(uint32_t id, std::vector<double>& d) const
    {
        const FaceTopoComp* ft = m_face_topos.Get(id);
        if (!ft) return false;
        const ToleranceComp* tol = m_tolerances.Get(id);
        d.push_back(tol ? tol->value : 0.0);
        d.push_back(static_cast<double>(ft->orientation));
        const SurfaceComp* surf = m_surfaces.Get(id);
        if (surf)
        {
            d.push_back(static_cast<double>(surf->surface_type));
            d.insert(d.end(), surf->data.begin(), surf->data.end());
        }
        d.push_back(ft->has_outer_wire ? 1.0 : 0.0);
        if (ft->has_outer_wire)
            ExportWire(ft->outer_wire_orientation, ft->outer_wire_edges, d);
        d.push_back(static_cast<double>(ft->inner_wires.size()));
        for (auto& iw : ft->inner_wires)
            ExportWire(iw.orientation, iw.edges, d);
        return true;
    }

    void ExportWire(uint8_t orientation,
                    const std::vector<FaceTopoComp::WireEdgeRef>& edges,
                    std::vector<double>& d) const
    {
        d.push_back(static_cast<double>(orientation));
        d.push_back(static_cast<double>(edges.size()));
        for (auto& ref : edges)
        {
            d.push_back(static_cast<double>(ref.edge_uid));
            d.push_back(static_cast<double>(ref.orientation));
            // pcurve
            d.push_back(static_cast<double>(ref.pcurve.curve_type));
            if (ref.pcurve.curve_type != Type::Empty)
                d.insert(d.end(), ref.pcurve.data.begin(), ref.pcurve.data.end());
            d.push_back(ref.pcurve.first);
            d.push_back(ref.pcurve.last);
        }
    }

    bool ExportSolid(uint32_t id, std::vector<double>& d) const
    {
        const SolidTopoComp* st = m_solid_topos.Get(id);
        if (!st) return false;
        d.push_back(static_cast<double>(st->shells.size()));
        for (auto& sh : st->shells)
        {
            d.push_back(static_cast<double>(sh.orientation));
            d.push_back(static_cast<double>(sh.face_uids.size()));
            for (uint32_t fuid : sh.face_uids)
                d.push_back(static_cast<double>(fuid));
        }
        return true;
    }

public:
    void Clear()
    {
        m_alive.clear();
        m_types.Clear();
        m_aabbs.Clear();
        m_positions.Clear();
        m_tolerances.Clear();
        m_params.Clear();
        m_curves.Clear();
        m_surfaces.Clear();
        m_edge_topos.Clear();
        m_face_topos.Clear();
        m_solid_topos.Clear();
        m_next_id = 1;
    }

private:
    std::vector<uint32_t> m_alive;
    uint32_t m_next_id = 1;

    ComponentPool<Type>          m_types;
    ComponentPool<AabbComp>      m_aabbs;
    ComponentPool<PositionComp>  m_positions;
    ComponentPool<ToleranceComp> m_tolerances;
    ComponentPool<ParamsComp>    m_params;
    ComponentPool<CurveComp>     m_curves;
    ComponentPool<SurfaceComp>   m_surfaces;
    ComponentPool<EdgeTopoComp>  m_edge_topos;
    ComponentPool<FaceTopoComp>  m_face_topos;
    ComponentPool<SolidTopoComp> m_solid_topos;
};

} // namespace brepdb
