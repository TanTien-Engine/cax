#include "brepdb_c/BrepDB.h"
#include "brepdb_c/BrepDBInit.h"
#include "brepdb_c/GeomPool.h"
#include "brepdb_c/NodeVersionInfo.h"

#include <graph/Node.h>
#include <vessel.h>
#include <wrapper/Graph.h>

#include <spatialdb/Region.h>
#include <spatialdb/Exception.h>

namespace
{

static constexpr const char* META_SHAPE_INDEX = "shape_index";
static constexpr const char* META_TOPO_GRAPH  = "topo_graph";

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

void BrepDB::Insert(const GeomHeader& header, const double* params)
{
    spatialdb::Region mbr(header.min_pt, header.max_pt);

    uint32_t param_bytes = header.param_count * sizeof(double);
    uint32_t total = sizeof(GeomHeader) + param_bytes;

    std::vector<uint8_t> blob(total);
    memcpy(blob.data(), &header, sizeof(GeomHeader));
    if (param_bytes > 0)
        memcpy(blob.data() + sizeof(GeomHeader), params, param_bytes);

    m_rtree->InsertData(total, blob.data(), mbr, header.persistent_id);
}

void BrepDB::ImportPool(const GeometryPool& pool)
{
    for (const auto& header : pool.headers) {
        const double* params = header.param_count > 0
            ? &pool.data_pool[header.param_offset] : nullptr;
        Insert(header, params);
    }
}

void BrepDB::Flush()
{
    StoreShapeIndex();
    StoreTopoGraph();
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

}