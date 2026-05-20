#include "brepdb_c/BrepDB.h"
#include "brepgraph_c/computation/CalcGraph.h"
#include "brepgraph_c/history/TopoNaming.h"
#include "brepdb_c/BrepDBInit.h"
#include "brepdb_c/NodeVersionInfo.h"
#include "cadapp_c/store/SketchStore.h"
#include "cadapp_c/store/FeatureStore.h"
#include "cadapp_c/ops/sketch_ops.h"
#include "cadapp_c/ops/resolve_ops.h"
#include "brepkit_c/GlobalConfig.h"

#include <graph/Node.h>
#include <vessel.h>
#include <wrapper/Graph.h>

#include <spatialdb/Region.h>
#include <spatialdb/Exception.h>

namespace
{

static constexpr const char* META_SHAPE_INDEX  = "shape_index";
static constexpr const char* META_TOPO_GRAPH   = "topo_graph";
static constexpr const char* META_CALC_GRAPH   = "calc_graph";
static constexpr const char* META_VERSION_TREE = "version_tree";
static constexpr const char* META_TOPO_NAMING = "topo_naming";
static constexpr const char* META_SKETCH_STORE  = "sketch_store";
static constexpr const char* META_FEATURE_STORE = "feature_store";

void SerializeWire(uint8_t orientation,
                   const std::vector<brepdb::FaceTopoComp::WireEdgeRef>& edges,
                   std::vector<double>& d)
{
    d.push_back(static_cast<double>(orientation));
    d.push_back(static_cast<double>(edges.size()));
    for (auto& ref : edges)
    {
        d.push_back(static_cast<double>(ref.edge_uid));
        d.push_back(static_cast<double>(ref.orientation));
        d.push_back(static_cast<double>(ref.pcurve.curve_type));
        if (ref.pcurve.curve_type != brepdb::Type::Empty)
            d.insert(d.end(), ref.pcurve.data.begin(), ref.pcurve.data.end());
        d.push_back(ref.pcurve.first);
        d.push_back(ref.pcurve.last);
    }
}

}

namespace brepdb
{

void init_cb()
{
    wrapper::Graph::Instance()->RegNodeGetCompCB("version_info", [](const graph::Node& node)
    {
        if (node.HasComponent<NodeVersionInfo>()) {
            auto& info = node.GetComponent<NodeVersionInfo>();
            ves_set_number(0, static_cast<double>(info.GetVersionId()));
        } else {
            ves_set_nil(0);
        }
    });
}

BrepDB::BrepDB(const std::shared_ptr<spatialdb::IStorageManager>& sm, bool overwrite)
    : m_sm(sm)
{
    if (overwrite)
    {
        m_rtree = std::make_unique<spatialdb::RTree>(sm, true);
        m_shape_index = std::make_unique<brepdb::ShapeIndex>(*m_rtree, sm);
    }
    else
    {
        m_rtree = std::make_unique<spatialdb::RTree>(sm, false);
        m_shape_index = std::make_unique<brepdb::ShapeIndex>(*m_rtree, sm);
        LoadShapeIndex();
        LoadTopoGraph();
    }
}

BrepDB::~BrepDB()
{
}

void BrepDB::Insert(uint32_t entity_id, const BRepWorld& world)
{
    const AabbComp* aabb = world.Aabbs().Get(entity_id);
    if (!aabb) return;

    spatialdb::Region mbr(aabb->min_pt, aabb->max_pt);

    // Serialize entity components into blob
    std::vector<double> params;
    const Type* t = world.Types().Get(entity_id);
    Type type = t ? *t : Type::Empty;

    switch (type)
    {
    case Type::Vertex:
    {
        const PositionComp* pos = world.Positions().Get(entity_id);
        if (pos) { params.push_back(pos->x); params.push_back(pos->y); params.push_back(pos->z); }
        const ToleranceComp* tol = world.Tolerances().Get(entity_id);
        params.push_back(tol ? tol->value : 0.0);
        break;
    }
    case Type::Edge:
    {
        const EdgeTopoComp* et = world.EdgeTopos().Get(entity_id);
        if (!et) break;
        params.push_back(et->v_first == UINT32_MAX ? -1.0 : static_cast<double>(et->v_first));
        params.push_back(et->v_last  == UINT32_MAX ? -1.0 : static_cast<double>(et->v_last));
        const ToleranceComp* tol = world.Tolerances().Get(entity_id);
        params.push_back(tol ? tol->value : 0.0);
        params.push_back(et->t_first);
        params.push_back(et->t_last);
        const CurveComp* curve = world.Curves().Get(entity_id);
        if (curve) {
            params.push_back(static_cast<double>(curve->curve_type));
            params.insert(params.end(), curve->data.begin(), curve->data.end());
        } else {
            params.push_back(static_cast<double>(Type::Empty));
        }
        break;
    }
    case Type::Face:
    {
        const FaceTopoComp* ft = world.FaceTopos().Get(entity_id);
        if (!ft) break;
        const ToleranceComp* tol = world.Tolerances().Get(entity_id);
        params.push_back(tol ? tol->value : 0.0);
        params.push_back(static_cast<double>(ft->orientation));
        const SurfaceComp* surf = world.Surfaces().Get(entity_id);
        if (surf) {
            params.push_back(static_cast<double>(surf->surface_type));
            params.insert(params.end(), surf->data.begin(), surf->data.end());
        }
        params.push_back(ft->has_outer_wire ? 1.0 : 0.0);
        if (ft->has_outer_wire)
            SerializeWire(ft->outer_wire_orientation, ft->outer_wire_edges, params);
        params.push_back(static_cast<double>(ft->inner_wires.size()));
        for (auto& iw : ft->inner_wires)
            SerializeWire(iw.orientation, iw.edges, params);
        break;
    }
    case Type::Solid:
    {
        const SolidTopoComp* st = world.SolidTopos().Get(entity_id);
        if (!st) break;
        params.push_back(static_cast<double>(st->shells.size()));
        for (auto& sh : st->shells) {
            params.push_back(static_cast<double>(sh.orientation));
            params.push_back(static_cast<double>(sh.face_uids.size()));
            for (uint32_t fuid : sh.face_uids)
                params.push_back(static_cast<double>(fuid));
        }
        break;
    }
    default:
        break;
    }

    // Blob layout: type(1) + pid(4) + min_pt(24) + max_pt(24) + param_count(4) + params(N*8)
    uint32_t param_count = static_cast<uint32_t>(params.size());
    uint32_t param_bytes = param_count * sizeof(double);
    constexpr uint32_t HDR = 1 + 4 + 24 + 24 + 4;  // 57 bytes header
    uint32_t total = HDR + param_bytes;

    std::vector<uint8_t> blob(total);
    uint8_t* p = blob.data();

    *p = static_cast<uint8_t>(type); p += 1;
    std::memcpy(p, &entity_id, 4);   p += 4;
    std::memcpy(p, aabb->min_pt, 24); p += 24;
    std::memcpy(p, aabb->max_pt, 24); p += 24;
    std::memcpy(p, &param_count, 4);  p += 4;
    if (param_bytes > 0)
        std::memcpy(p, params.data(), param_bytes);

    m_rtree->InsertData(total, blob.data(), mbr, entity_id);
}

void BrepDB::ImportWorld(const BRepWorld& world)
{
    for (uint32_t id : world.AliveEntities())
        Insert(id, world);
}

void BrepDB::Flush()
{
    StoreShapeIndex();
    StoreTopoGraph();

    auto gc = brepkit::GlobalConfig::Instance();
    if (gc->GetCalcGraph())
        StoreCalcGraph(*gc->GetCalcGraph());
    if (gc->GetTopoNaming())
        StoreTopoNaming(*gc->GetTopoNaming());
    if (gc->GetVersionTree())
        StoreVersionTree(*gc->GetVersionTree());

    m_rtree->Flush();
    m_sm->Flush();
}

void BrepDB::StoreShapeIndex()
{
    spatialdb::id_type page = m_rtree->GetMetaPage(META_SHAPE_INDEX);
    m_shape_index->Store(page);
    m_rtree->SetMetaPage(META_SHAPE_INDEX, page);
}

void BrepDB::LoadShapeIndex()
{
    spatialdb::id_type page = m_rtree->GetMetaPage(META_SHAPE_INDEX);
    if (page == spatialdb::NewPage) {
        m_shape_index->Rebuild();
        return;
    }
    m_shape_index->Load(page);
}

void BrepDB::StoreTopoGraph()
{
    uint8_t* buf = nullptr;
    uint32_t len = 0;
    m_topo_graph.StoreToByteArray(&buf, len);

    if (len == 0) { delete[] buf; return; }

    spatialdb::id_type page = m_rtree->GetMetaPage(META_TOPO_GRAPH);
    try {
        m_sm->StoreByteArray(page, len, buf);
        m_rtree->SetMetaPage(META_TOPO_GRAPH, page);
    } catch (...) {
        delete[] buf;
        throw;
    }
    delete[] buf;
}

void BrepDB::LoadTopoGraph()
{
    spatialdb::id_type page = m_rtree->GetMetaPage(META_TOPO_GRAPH);
    if (page == spatialdb::NewPage) return;

    uint32_t len = 0;
    uint8_t* buf = nullptr;
    try {
        m_sm->LoadByteArray(page, len, &buf);
    } catch (spatialdb::InvalidPageException&) {
        return;
    }
    m_topo_graph.LoadFromByteArray(buf, len);
    delete[] buf;
}

void BrepDB::StoreCalcGraph(const brepgraph::CalcGraph& cg)
{
    uint8_t* buf = nullptr;
    uint32_t len = 0;
    cg.StoreToByteArray(&buf, len);

    if (len == 0) { delete[] buf; return; }

    spatialdb::id_type page = m_rtree->GetMetaPage(META_CALC_GRAPH);
    try {
        m_sm->StoreByteArray(page, len, buf);
        m_rtree->SetMetaPage(META_CALC_GRAPH, page);
    } catch (...) {
        delete[] buf;
        throw;
    }
    delete[] buf;
}

bool BrepDB::LoadCalcGraph(brepgraph::CalcGraph& cg)
{
    spatialdb::id_type page = m_rtree->GetMetaPage(META_CALC_GRAPH);
    if (page == spatialdb::NewPage)
        return false;

    uint32_t len = 0;
    uint8_t* buf = nullptr;
    try {
        m_sm->LoadByteArray(page, len, &buf);
    } catch (spatialdb::InvalidPageException&) {
        return false;
    }

    // The saved graph may reference cadapp ops (sketch_face,
    // resolve_edge_ref, resolve_face_ref); make sure they're in this
    // CalcGraph's registry before LoadFromByteArray rebuilds the IR.
    cadapp::RegisterSketchOps(cg.GetRegistry());
    cadapp::RegisterResolveOps(cg.GetRegistry());

    bool ok = cg.LoadFromByteArray(buf, len);
    delete[] buf;
    return ok;
}

void BrepDB::StoreVersionTree(const VersionTree& vt)
{
    uint8_t* buf = nullptr;
    uint32_t len = 0;
    vt.StoreToByteArray(&buf, len);

    if (len == 0) { delete[] buf; return; }

    spatialdb::id_type page = m_rtree->GetMetaPage(META_VERSION_TREE);
    try {
        m_sm->StoreByteArray(page, len, buf);
        m_rtree->SetMetaPage(META_VERSION_TREE, page);
    } catch (...) {
        delete[] buf;
        throw;
    }
    delete[] buf;
}

bool BrepDB::LoadVersionTree(VersionTree& vt)
{
    spatialdb::id_type page = m_rtree->GetMetaPage(META_VERSION_TREE);
    if (page == spatialdb::NewPage)
        return false;

    uint32_t len = 0;
    uint8_t* buf = nullptr;
    try {
        m_sm->LoadByteArray(page, len, &buf);
    } catch (spatialdb::InvalidPageException&) {
        return false;
    }

    vt.LoadFromByteArray(buf, len);
    delete[] buf;
    return true;
}

void BrepDB::StoreTopoNaming(const brepgraph::TopoNaming& tn)
{
    uint8_t* buf = nullptr;
    uint32_t len = 0;
    tn.StoreToByteArray(&buf, len);

    if (len == 0) { delete[] buf; return; }

    spatialdb::id_type page = m_rtree->GetMetaPage(META_TOPO_NAMING);
    try {
        m_sm->StoreByteArray(page, len, buf);
        m_rtree->SetMetaPage(META_TOPO_NAMING, page);
    } catch (...) {
        delete[] buf;
        throw;
    }
    delete[] buf;
}

bool BrepDB::LoadTopoNaming(brepgraph::TopoNaming& tn)
{
    spatialdb::id_type page = m_rtree->GetMetaPage(META_TOPO_NAMING);
    if (page == spatialdb::NewPage)
        return false;

    uint32_t len = 0;
    uint8_t* buf = nullptr;
    try {
        m_sm->LoadByteArray(page, len, &buf);
    } catch (spatialdb::InvalidPageException&) {
        return false;
    }

    bool ok = tn.LoadFromByteArray(buf, len);
    delete[] buf;
    return ok;
}

void BrepDB::StoreSketchStore(const cadapp::SketchStore& ss)
{
    uint8_t* buf = nullptr;
    uint32_t len = 0;
    ss.StoreToByteArray(&buf, len);

    if (len == 0)
    {
        delete[] buf;
        return;
    }

    spatialdb::id_type page = m_rtree->GetMetaPage(META_SKETCH_STORE);
    try
    {
        m_sm->StoreByteArray(page, len, buf);
        m_rtree->SetMetaPage(META_SKETCH_STORE, page);
    }
    catch (...)
    {
        delete[] buf;
        throw;
    }
    delete[] buf;
}

bool BrepDB::LoadSketchStore(cadapp::SketchStore& ss)
{
    spatialdb::id_type page = m_rtree->GetMetaPage(META_SKETCH_STORE);
    if (page == spatialdb::NewPage) {
        return false;
    }

    uint32_t len = 0;
    uint8_t* buf = nullptr;
    try
    {
        m_sm->LoadByteArray(page, len, &buf);
    }
    catch (spatialdb::InvalidPageException&)
    {
        return false;
    }

    bool ok = ss.LoadFromByteArray(buf, len);
    delete[] buf;
    return ok;
}

void BrepDB::StoreFeatureStore(const cadapp::FeatureStore& fs)
{
    uint8_t* buf = nullptr;
    uint32_t len = 0;
    fs.StoreToByteArray(&buf, len);

    if (len == 0)
    {
        delete[] buf;
        return;
    }

    spatialdb::id_type page = m_rtree->GetMetaPage(META_FEATURE_STORE);
    try
    {
        m_sm->StoreByteArray(page, len, buf);
        m_rtree->SetMetaPage(META_FEATURE_STORE, page);
    }
    catch (...)
    {
        delete[] buf;
        throw;
    }
    delete[] buf;
}

bool BrepDB::LoadFeatureStore(cadapp::FeatureStore& fs)
{
    spatialdb::id_type page = m_rtree->GetMetaPage(META_FEATURE_STORE);
    if (page == spatialdb::NewPage) {
        return false;
    }

    uint32_t len = 0;
    uint8_t* buf = nullptr;
    try
    {
        m_sm->LoadByteArray(page, len, &buf);
    }
    catch (spatialdb::InvalidPageException&)
    {
        return false;
    }

    bool ok = fs.LoadFromByteArray(buf, len);
    delete[] buf;
    return ok;
}

}
