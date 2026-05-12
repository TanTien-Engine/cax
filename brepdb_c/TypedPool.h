#pragma once

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

namespace brepdb
{

enum class Type : uint8_t
{
    Empty = 0,
    Line = 1,
    Circle = 2,
    BSplineCurve = 3,
    Ellipse = 4,

    Plane = 10,
    Cylinder = 11,
    BSplineSurface = 12,
    Sphere = 13,
    Torus = 14,
    Cone = 15,

    Vertex = 20,
    Edge = 21,
    Wire = 22,
    Face = 23,
    Shell = 24,
    Solid = 25,
    Compound = 26
};

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
    Type curve_type = Type::Empty;
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
    Type surface_type = Type::Empty;
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

    // Parse flat ParamsComp data into typed components for each entity.
    void RebuildTypedFromParams()
    {
        for (uint32_t id : m_alive)
        {
            const Type* t = m_types.Get(id);
            if (!t) continue;
            const ParamsComp* pc = m_params.Get(id);
            if (!pc || pc->data.empty()) continue;

            uint32_t offset = 0;
            const auto& d = pc->data;

            switch (*t)
            {
            case Type::Vertex:
            {
                if (d.size() < 4) break;
                m_positions.Set(id, {d[0], d[1], d[2]});
                m_tolerances.Set(id, {d[3]});
                break;
            }
            case Type::Edge:
            {
                if (d.size() < 6) break;
                EdgeTopoComp et;
                et.v_first = d[0] < 0 ? UINT32_MAX : static_cast<uint32_t>(d[0]);
                et.v_last  = d[1] < 0 ? UINT32_MAX : static_cast<uint32_t>(d[1]);
                m_tolerances.Set(id, {d[2]});
                et.t_first = d[3];
                et.t_last  = d[4];
                m_edge_topos.Set(id, et);
                offset = 5;
                if (offset < d.size())
                {
                    Type ct = static_cast<Type>(static_cast<int>(d[offset++]));
                    if (ct != Type::Empty)
                    {
                        CurveComp curve;
                        curve.curve_type = ct;
                        curve.data.assign(d.begin() + offset, d.end());
                        m_curves.Set(id, curve);
                    }
                }
                break;
            }
            case Type::Face:
            {
                if (d.size() < 3) break;
                FaceTopoComp ft;
                m_tolerances.Set(id, {d[0]});
                ft.orientation = static_cast<uint8_t>(d[1]);
                offset = 2;
                // surface
                if (offset < d.size())
                {
                    Type st = static_cast<Type>(static_cast<int>(d[offset++]));
                    SurfaceComp surf;
                    surf.surface_type = st;
                    // find end of surface data: before has_outer_wire flag
                    // We scan for wire structure; surface data ends at has_outer_wire
                    // Parse surface data by reading until we hit wire section
                    // This requires knowing surface param counts per type...
                    // For now store remaining as surface + wire combined in params
                    // and let the full parse happen when we have surface size info.
                    ParseFaceParams(id, d, offset, ft, surf);
                    m_surfaces.Set(id, surf);
                }
                m_face_topos.Set(id, ft);
                break;
            }
            case Type::Solid:
            {
                if (d.empty()) break;
                SolidTopoComp st;
                offset = 0;
                int shell_count = static_cast<int>(d[offset++]);
                for (int s = 0; s < shell_count && offset < d.size(); ++s)
                {
                    SolidTopoComp::ShellComp sh;
                    sh.orientation = static_cast<uint8_t>(d[offset++]);
                    int face_count = static_cast<int>(d[offset++]);
                    for (int f = 0; f < face_count && offset < d.size(); ++f)
                        sh.face_uids.push_back(static_cast<uint32_t>(d[offset++]));
                    st.shells.push_back(std::move(sh));
                }
                m_solid_topos.Set(id, st);
                break;
            }
            default:
                break;
            }
        }
    }

private:
    // Parse face params into typed components, advancing offset past surface+wires
    void ParseFaceParams(uint32_t id, const std::vector<double>& d, uint32_t& offset,
                         FaceTopoComp& ft, SurfaceComp& surf)
    {
        // Surface data was already started: surf.surface_type is set, offset is past type
        uint32_t surf_start = offset;
        // Advance offset past surface data based on type
        SkipSurfaceData(surf.surface_type, d, offset);
        surf.data.assign(d.begin() + surf_start, d.begin() + offset);

        // has_outer_wire
        if (offset >= d.size()) return;
        ft.has_outer_wire = d[offset++] > 0.5;
        if (ft.has_outer_wire)
            ParseWire(d, offset, ft.outer_wire_orientation, ft.outer_wire_edges);

        // inner wires
        if (offset >= d.size()) return;
        int inner_count = static_cast<int>(d[offset++]);
        for (int i = 0; i < inner_count && offset < d.size(); ++i)
        {
            FaceTopoComp::WireComp iw;
            ParseWire(d, offset, iw.orientation, iw.edges);
            ft.inner_wires.push_back(std::move(iw));
        }
    }

    void ParseWire(const std::vector<double>& d, uint32_t& offset,
                   uint8_t& wire_ori, std::vector<FaceTopoComp::WireEdgeRef>& edges)
    {
        if (offset >= d.size()) return;
        wire_ori = static_cast<uint8_t>(d[offset++]);
        if (offset >= d.size()) return;
        int count = static_cast<int>(d[offset++]);
        for (int i = 0; i < count && offset < d.size(); ++i)
        {
            FaceTopoComp::WireEdgeRef ref;
            ref.edge_uid = static_cast<uint32_t>(d[offset++]);
            ref.orientation = static_cast<uint8_t>(d[offset++]);
            // pcurve
            ref.pcurve.curve_type = static_cast<Type>(static_cast<int>(d[offset++]));
            if (ref.pcurve.curve_type != Type::Empty)
            {
                uint32_t pc_start = offset;
                SkipCurve2dData(ref.pcurve.curve_type, d, offset);
                ref.pcurve.data.assign(d.begin() + pc_start, d.begin() + offset);
            }
            ref.pcurve.first = d[offset++];
            ref.pcurve.last  = d[offset++];
            edges.push_back(std::move(ref));
        }
    }

    static void SkipCurveData(Type curve_type, const std::vector<double>& d, uint32_t& offset)
    {
        if (curve_type == Type::Line) {
            offset += 6; // point(3) + dir(3)
        } else if (curve_type == Type::Circle) {
            offset += 10; // point(3) + dir(3) + xdir(3) + radius(1)
        } else if (curve_type == Type::Ellipse) {
            offset += 11; // point(3) + dir(3) + xdir(3) + majorR(1) + minorR(1)
        } else if (curve_type == Type::BSplineCurve) {
            int degree = static_cast<int>(d[offset++]); (void)degree;
            int nbPoles = static_cast<int>(d[offset++]);
            int nbKnots = static_cast<int>(d[offset++]);
            bool isRational = d[offset++] > 0.5;
            offset++; // isPeriodic
            offset += nbPoles * 3; // poles
            if (isRational) offset += nbPoles; // weights
            offset += nbKnots; // knots
            offset += nbKnots; // mults
        }
    }

    static void SkipCurve2dData(Type curve_type, const std::vector<double>& d, uint32_t& offset)
    {
        if (curve_type == Type::Line) {
            offset += 4; // lx, ly, dx, dy
        } else if (curve_type == Type::Circle) {
            offset += 5; // cx, cy, xx, xy, r
        } else if (curve_type == Type::Ellipse) {
            offset += 6; // cx, cy, xx, xy, majorR, minorR
        } else if (curve_type == Type::BSplineCurve) {
            int degree = static_cast<int>(d[offset++]); (void)degree;
            int nbPoles = static_cast<int>(d[offset++]);
            int nbKnots = static_cast<int>(d[offset++]);
            bool isRational = d[offset++] > 0.5;
            offset++; // isPeriodic
            offset += nbPoles * 2; // poles (2d)
            if (isRational) offset += nbPoles; // weights
            offset += nbKnots; // knots
            offset += nbKnots; // mults
        }
    }

    static void SkipSurfaceData(Type surf_type, const std::vector<double>& d, uint32_t& offset)
    {
        if (surf_type == Type::Plane) {
            offset += 9; // point(3) + axisDir(3) + xDir(3)
        } else if (surf_type == Type::Cylinder) {
            offset += 10; // point(3) + axisDir(3) + xDir(3) + radius(1)
        } else if (surf_type == Type::Sphere) {
            offset += 10; // point(3) + axisDir(3) + xDir(3) + radius(1)
        } else if (surf_type == Type::Torus) {
            offset += 11; // point(3) + axisDir(3) + xDir(3) + majorR(1) + minorR(1)
        } else if (surf_type == Type::Cone) {
            offset += 11; // point(3) + axisDir(3) + xDir(3) + refR(1) + semiAngle(1)
        } else if (surf_type == Type::BSplineSurface) {
            int uDeg = static_cast<int>(d[offset++]); (void)uDeg;
            int vDeg = static_cast<int>(d[offset++]); (void)vDeg;
            int nbUPoles = static_cast<int>(d[offset++]);
            int nbVPoles = static_cast<int>(d[offset++]);
            int nbUKnots = static_cast<int>(d[offset++]);
            int nbVKnots = static_cast<int>(d[offset++]);
            bool isURational = d[offset++] > 0.5;
            bool isVRational = d[offset++] > 0.5;
            offset++; // isUPeriodic
            offset++; // isVPeriodic
            offset += nbUPoles * nbVPoles * 3; // poles
            if (isURational || isVRational) offset += nbUPoles * nbVPoles; // weights
            offset += nbUKnots; // uKnots
            offset += nbUKnots; // uMults
            offset += nbVKnots; // vKnots
            offset += nbVKnots; // vMults
        }
    }

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
    std::vector<double> ExportEntityParams(uint32_t id) const
    {
        std::vector<double> d;
        const Type* t = m_types.Get(id);
        if (t)
        {
            switch (*t)
            {
            case Type::Vertex:  ExportVertex(id, d);  break;
            case Type::Edge:    ExportEdge(id, d);    break;
            case Type::Face:    ExportFace(id, d);    break;
            case Type::Solid:   ExportSolid(id, d);   break;
            default: break;
            }
        }
        if (d.empty())
        {
            const ParamsComp* pc = m_params.Get(id);
            if (pc && !pc->data.empty())
                d = pc->data;
        }
        return d;
    }

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
