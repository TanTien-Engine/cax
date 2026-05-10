#include "brepdb_c/WorldFile.h"
#include "brepdb_c/TypedPool.h"

#include <fstream>
#include <cstring>

namespace
{

constexpr uint32_t MAGIC   = 0x43574C44; // "CWLD" (CAD World)
constexpr uint32_t VERSION = 1;

struct FileHeader
{
    uint32_t magic   = MAGIC;
    uint32_t version = VERSION;
    uint32_t entity_count = 0;
};

// Per-entity record header
struct EntityRecord
{
    uint32_t entity_id;
    uint8_t  type;
    uint8_t  has_position;
    uint8_t  has_tolerance;
    uint8_t  has_curve;
    uint8_t  has_surface;
    uint8_t  has_edge_topo;
    uint8_t  has_face_topo;
    uint8_t  has_solid_topo;
};

void WriteVec(std::ofstream& os, const std::vector<double>& v)
{
    uint32_t n = static_cast<uint32_t>(v.size());
    os.write(reinterpret_cast<const char*>(&n), 4);
    if (n > 0)
        os.write(reinterpret_cast<const char*>(v.data()), n * sizeof(double));
}

void ReadVec(std::ifstream& is, std::vector<double>& v)
{
    uint32_t n = 0;
    is.read(reinterpret_cast<char*>(&n), 4);
    v.resize(n);
    if (n > 0)
        is.read(reinterpret_cast<char*>(v.data()), n * sizeof(double));
}

void WriteU32Vec(std::ofstream& os, const std::vector<uint32_t>& v)
{
    uint32_t n = static_cast<uint32_t>(v.size());
    os.write(reinterpret_cast<const char*>(&n), 4);
    if (n > 0)
        os.write(reinterpret_cast<const char*>(v.data()), n * 4);
}

void ReadU32Vec(std::ifstream& is, std::vector<uint32_t>& v)
{
    uint32_t n = 0;
    is.read(reinterpret_cast<char*>(&n), 4);
    v.resize(n);
    if (n > 0)
        is.read(reinterpret_cast<char*>(v.data()), n * 4);
}

void WriteCurve2d(std::ofstream& os, const brepdb::Curve2dComp& c)
{
    uint8_t ct = static_cast<uint8_t>(c.curve_type);
    os.write(reinterpret_cast<const char*>(&ct), 1);
    os.write(reinterpret_cast<const char*>(&c.first), 8);
    os.write(reinterpret_cast<const char*>(&c.last), 8);
    WriteVec(os, c.data);
}

void ReadCurve2d(std::ifstream& is, brepdb::Curve2dComp& c)
{
    uint8_t ct = 0;
    is.read(reinterpret_cast<char*>(&ct), 1);
    c.curve_type = static_cast<brepdb::Type>(ct);
    is.read(reinterpret_cast<char*>(&c.first), 8);
    is.read(reinterpret_cast<char*>(&c.last), 8);
    ReadVec(is, c.data);
}

void WriteWireEdgeRefs(std::ofstream& os, const std::vector<brepdb::FaceTopoComp::WireEdgeRef>& refs)
{
    uint32_t n = static_cast<uint32_t>(refs.size());
    os.write(reinterpret_cast<const char*>(&n), 4);
    for (auto& r : refs)
    {
        os.write(reinterpret_cast<const char*>(&r.edge_uid), 4);
        os.write(reinterpret_cast<const char*>(&r.orientation), 1);
        WriteCurve2d(os, r.pcurve);
    }
}

void ReadWireEdgeRefs(std::ifstream& is, std::vector<brepdb::FaceTopoComp::WireEdgeRef>& refs)
{
    uint32_t n = 0;
    is.read(reinterpret_cast<char*>(&n), 4);
    refs.resize(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        is.read(reinterpret_cast<char*>(&refs[i].edge_uid), 4);
        is.read(reinterpret_cast<char*>(&refs[i].orientation), 1);
        ReadCurve2d(is, refs[i].pcurve);
    }
}

} // anonymous

namespace brepdb
{

bool WorldFile::Save(const std::string& filename, const BRepWorld& world)
{
    std::ofstream os(filename, std::ios::binary);
    if (!os) return false;

    const auto& alive = world.AliveEntities();

    FileHeader fh;
    fh.entity_count = static_cast<uint32_t>(alive.size());
    os.write(reinterpret_cast<const char*>(&fh), sizeof(FileHeader));

    for (uint32_t id : alive)
    {
        EntityRecord rec{};
        rec.entity_id = id;

        const Type* t = world.Types().Get(id);
        rec.type = t ? static_cast<uint8_t>(*t) : 0;
        rec.has_position   = world.Positions().Has(id) ? 1 : 0;
        rec.has_tolerance  = world.Tolerances().Has(id) ? 1 : 0;
        rec.has_curve      = world.Curves().Has(id) ? 1 : 0;
        rec.has_surface    = world.Surfaces().Has(id) ? 1 : 0;
        rec.has_edge_topo  = world.EdgeTopos().Has(id) ? 1 : 0;
        rec.has_face_topo  = world.FaceTopos().Has(id) ? 1 : 0;
        rec.has_solid_topo = world.SolidTopos().Has(id) ? 1 : 0;

        os.write(reinterpret_cast<const char*>(&rec), sizeof(EntityRecord));

        // AABB (always present for alive entities with type)
        const AabbComp* aabb = world.Aabbs().Get(id);
        if (aabb)
            os.write(reinterpret_cast<const char*>(aabb), sizeof(AabbComp));
        else
        {
            AabbComp empty{};
            os.write(reinterpret_cast<const char*>(&empty), sizeof(AabbComp));
        }

        // Position
        if (rec.has_position)
        {
            const PositionComp* pos = world.Positions().Get(id);
            os.write(reinterpret_cast<const char*>(pos), sizeof(PositionComp));
        }

        // Tolerance
        if (rec.has_tolerance)
        {
            const ToleranceComp* tol = world.Tolerances().Get(id);
            os.write(reinterpret_cast<const char*>(tol), sizeof(ToleranceComp));
        }

        // Curve
        if (rec.has_curve)
        {
            const CurveComp* curve = world.Curves().Get(id);
            uint8_t ct = static_cast<uint8_t>(curve->curve_type);
            os.write(reinterpret_cast<const char*>(&ct), 1);
            WriteVec(os, curve->data);
        }

        // Surface
        if (rec.has_surface)
        {
            const SurfaceComp* surf = world.Surfaces().Get(id);
            uint8_t st = static_cast<uint8_t>(surf->surface_type);
            os.write(reinterpret_cast<const char*>(&st), 1);
            WriteVec(os, surf->data);
        }

        // EdgeTopo
        if (rec.has_edge_topo)
        {
            const EdgeTopoComp* et = world.EdgeTopos().Get(id);
            os.write(reinterpret_cast<const char*>(et), sizeof(EdgeTopoComp));
        }

        // FaceTopo
        if (rec.has_face_topo)
        {
            const FaceTopoComp* ft = world.FaceTopos().Get(id);
            os.write(reinterpret_cast<const char*>(&ft->orientation), 1);
            uint8_t how = ft->has_outer_wire ? 1 : 0;
            os.write(reinterpret_cast<const char*>(&how), 1);
            if (ft->has_outer_wire)
            {
                os.write(reinterpret_cast<const char*>(&ft->outer_wire_orientation), 1);
                WriteWireEdgeRefs(os, ft->outer_wire_edges);
            }
            uint32_t iw_count = static_cast<uint32_t>(ft->inner_wires.size());
            os.write(reinterpret_cast<const char*>(&iw_count), 4);
            for (auto& iw : ft->inner_wires)
            {
                os.write(reinterpret_cast<const char*>(&iw.orientation), 1);
                WriteWireEdgeRefs(os, iw.edges);
            }
        }

        // SolidTopo
        if (rec.has_solid_topo)
        {
            const SolidTopoComp* st = world.SolidTopos().Get(id);
            uint32_t shell_count = static_cast<uint32_t>(st->shells.size());
            os.write(reinterpret_cast<const char*>(&shell_count), 4);
            for (auto& sh : st->shells)
            {
                os.write(reinterpret_cast<const char*>(&sh.orientation), 1);
                WriteU32Vec(os, sh.face_uids);
            }
        }
    }

    os.close();
    return true;
}

bool WorldFile::Load(const std::string& filename, BRepWorld& world)
{
    std::ifstream is(filename, std::ios::binary);
    if (!is) return false;

    FileHeader fh;
    is.read(reinterpret_cast<char*>(&fh), sizeof(FileHeader));
    if (fh.magic != MAGIC) return false;

    world.Clear();

    for (uint32_t i = 0; i < fh.entity_count; ++i)
    {
        EntityRecord rec{};
        is.read(reinterpret_cast<char*>(&rec), sizeof(EntityRecord));

        uint32_t id = rec.entity_id;
        world.RegisterEntity(id);
        world.Types().Set(id, static_cast<Type>(rec.type));

        // AABB
        AabbComp aabb{};
        is.read(reinterpret_cast<char*>(&aabb), sizeof(AabbComp));
        world.Aabbs().Set(id, aabb);

        // Position
        if (rec.has_position)
        {
            PositionComp pos{};
            is.read(reinterpret_cast<char*>(&pos), sizeof(PositionComp));
            world.Positions().Set(id, pos);
        }

        // Tolerance
        if (rec.has_tolerance)
        {
            ToleranceComp tol{};
            is.read(reinterpret_cast<char*>(&tol), sizeof(ToleranceComp));
            world.Tolerances().Set(id, tol);
        }

        // Curve
        if (rec.has_curve)
        {
            CurveComp curve;
            uint8_t ct = 0;
            is.read(reinterpret_cast<char*>(&ct), 1);
            curve.curve_type = static_cast<Type>(ct);
            ReadVec(is, curve.data);
            world.Curves().Set(id, std::move(curve));
        }

        // Surface
        if (rec.has_surface)
        {
            SurfaceComp surf;
            uint8_t st = 0;
            is.read(reinterpret_cast<char*>(&st), 1);
            surf.surface_type = static_cast<Type>(st);
            ReadVec(is, surf.data);
            world.Surfaces().Set(id, std::move(surf));
        }

        // EdgeTopo
        if (rec.has_edge_topo)
        {
            EdgeTopoComp et{};
            is.read(reinterpret_cast<char*>(&et), sizeof(EdgeTopoComp));
            world.EdgeTopos().Set(id, et);
        }

        // FaceTopo
        if (rec.has_face_topo)
        {
            FaceTopoComp ft;
            is.read(reinterpret_cast<char*>(&ft.orientation), 1);
            uint8_t how = 0;
            is.read(reinterpret_cast<char*>(&how), 1);
            ft.has_outer_wire = (how != 0);
            if (ft.has_outer_wire)
            {
                is.read(reinterpret_cast<char*>(&ft.outer_wire_orientation), 1);
                ReadWireEdgeRefs(is, ft.outer_wire_edges);
            }
            uint32_t iw_count = 0;
            is.read(reinterpret_cast<char*>(&iw_count), 4);
            ft.inner_wires.resize(iw_count);
            for (uint32_t w = 0; w < iw_count; ++w)
            {
                is.read(reinterpret_cast<char*>(&ft.inner_wires[w].orientation), 1);
                ReadWireEdgeRefs(is, ft.inner_wires[w].edges);
            }
            world.FaceTopos().Set(id, std::move(ft));
        }

        // SolidTopo
        if (rec.has_solid_topo)
        {
            SolidTopoComp st;
            uint32_t shell_count = 0;
            is.read(reinterpret_cast<char*>(&shell_count), 4);
            st.shells.resize(shell_count);
            for (uint32_t s = 0; s < shell_count; ++s)
            {
                is.read(reinterpret_cast<char*>(&st.shells[s].orientation), 1);
                ReadU32Vec(is, st.shells[s].face_uids);
            }
            world.SolidTopos().Set(id, std::move(st));
        }
    }

    is.close();
    return true;
}

} // namespace brepdb
