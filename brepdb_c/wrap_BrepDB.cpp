#include "brepdb_c/wrap_BrepDB.h"

#include <brepir_c/Data.h>
#include <brepir_c/Receiver.h>

#include <spatialdb/RTree.h>
#include <spatialdb/DiskStorageManager.h>
#include <spatialdb/Region.h>
#include <spatialdb/ObjVisitor.h>
#include <partgraph_c/TransHelper.h>

#include <SM_Cube.h>

namespace
{

void w_BrepDB_allocate()
{
    auto proxy = (wrapper::Proxy<spatialdb::RTree>*)ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<spatialdb::RTree>));

    auto num = ves_argnum();
    if (num < 2)
    {
        auto sm = std::make_shared<spatialdb::DiskStorageManager>("test_db");
        proxy->obj = std::make_shared<spatialdb::RTree>(sm, true);
    }
    else
    {
        auto sm = ((wrapper::Proxy<spatialdb::DiskStorageManager>*)ves_toforeign(1))->obj;
        proxy->obj = std::make_shared<spatialdb::RTree>(sm, false);
    }
}

int w_BrepDB_finalize(void* data)
{
    auto proxy = (wrapper::Proxy<spatialdb::RTree>*)(data);
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<spatialdb::RTree>);
}

void w_BrepDB_insert()
{
    auto rtree = ((wrapper::Proxy<spatialdb::RTree>*)ves_toforeign(0))->obj;
    auto pool = ((wrapper::Proxy<brepir::GeometryPool>*)ves_toforeign(1))->obj;

    //for (const auto& h : pool->headers)
    //{
    //    const spatialdb::Region aabb(h.min_pt, h.max_pt);
    //    const spatialdb::id_type id = h.persistent_id;
    //    rtree->InsertData(h.param_count, (uint8_t*)&pool->data_pool[h.param_offset], aabb, id);
    //}

    size_t len = sizeof(size_t) + sizeof(brepir::Header) * pool->headers.size() + sizeof(double) * pool->data_pool.size();
    uint8_t* data = new uint8_t[len];
    uint8_t* ptr = data;

    size_t num = pool->headers.size();
    memcpy(ptr, &num, sizeof(num));
    ptr += sizeof(num);

    for (const auto& h : pool->headers) 
    {
        memcpy(ptr, &h, sizeof(h));
        ptr += sizeof(h);
    }

    memcpy(ptr, pool->data_pool.data(), sizeof(double) * pool->data_pool.size());

    spatialdb::Region aabb;
    for (const auto& h : pool->headers) {
        aabb.Combine({ h.min_pt, h.max_pt });
    }

    rtree->InsertData(len, data, aabb, 0);

    delete[] data;
}

void w_BrepDB_query()
{
    auto rtree = ((wrapper::Proxy<spatialdb::RTree>*)ves_toforeign(0))->obj;
    sm::cube* aabb = (sm::cube*)ves_toforeign(1);

    const double min[] = { aabb->xmin, aabb->ymin, aabb->zmin };
    const double max[] = { aabb->xmax, aabb->ymax, aabb->zmax };
    spatialdb::Region region(min, max);

    auto visitor = std::make_unique<spatialdb::ObjVisitor>();
    rtree->IntersectsWithQuery(region, *visitor);

    auto& items = visitor->GetResults();
    for (auto item : items)
    {
        uint32_t len = 0;
        uint8_t* data = nullptr;
        item->GetData(len, &data);

        auto pool = std::make_shared<brepir::GeometryPool>();

        uint8_t* ptr = data;
        size_t num = 0;
        memcpy(&num, ptr, sizeof(num));
        ptr += sizeof(num);

        for (size_t i = 0; i < num; ++i)
        {
            brepir::Header h;
            memcpy(&h, ptr, sizeof(h));
            ptr += sizeof(h);
            pool->headers.emplace_back(h);
        }

        pool->data_pool.assign((double*)ptr, (double*)(data + len));

        delete[] data;

        ves_pop(ves_argnum());

        ves_pushnil();
        ves_import_class("brepir", "BrepIR");
        auto proxy = (wrapper::Proxy<brepir::GeometryPool>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<brepir::GeometryPool>));
        proxy->obj = pool;
        ves_pop(1);

        return;
    }

    ves_set_nil(0);
}

}

namespace brepdb
{

VesselForeignMethodFn BrepDBBindMethod(const char* signature)
{
    if (strcmp(signature, "BrepDB.insert(_)") == 0) return w_BrepDB_insert;
    if (strcmp(signature, "BrepDB.query(_)") == 0) return w_BrepDB_query;

    return nullptr;
}

void BrepDBBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
    if (strcmp(class_name, "BrepDB") == 0)
    {
        methods->allocate = w_BrepDB_allocate;
        methods->finalize = w_BrepDB_finalize;
        return;
    }
}

}