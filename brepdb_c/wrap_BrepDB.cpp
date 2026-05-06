#include "brepdb_c/wrap_BrepDB.h"
#include "brepdb_c/GeomPool.h"
#include "brepdb_c/GeomSender.h"
#include "brepdb_c/GeomReceiver.h"
#include "brepdb_c/GeomFile.h"

#include <spatialdb/RTree.h>
#include <spatialdb/DiskStorageManager.h>
#include <spatialdb/Region.h>
#include <spatialdb/ObjVisitor.h>
#include <partgraph_c/TopoShape.h>
#include <partgraph_c/GlobalConfig.h>
#include <partgraph_c/TransHelper.h>
#include <wrapper/TransHelper.h>
#include <SM_Cube.h>

#include <TopTools_IndexedMapOfShape.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Solid.hxx>
#include <BRep_Builder.hxx>

namespace
{

void w_BrepIR_allocate()
{
    auto proxy = (wrapper::Proxy<brepdb::GeometryPool>*)ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<brepdb::GeometryPool>));
    auto pool = std::make_shared<brepdb::GeometryPool>();
    proxy->obj = pool;
}

int w_BrepIR_finalize(void* data)
{
    auto proxy = (wrapper::Proxy<brepdb::GeometryPool>*)(data);
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<brepdb::GeometryPool>);
}

void w_BrepIR_save()
{
    auto pool = ((wrapper::Proxy<brepdb::GeometryPool>*)ves_toforeign(0))->obj;
    std::string filepath = ves_tostring(1);
    brepdb::GeomFile::Save(filepath, *pool);
}

void w_BrepIR_load()
{
    auto pool = ((wrapper::Proxy<brepdb::GeometryPool>*)ves_toforeign(0))->obj;
    std::string filepath = ves_tostring(1);
    brepdb::GeomFile::Load(filepath, *pool);
}

void w_BrepIR_serialize()
{
    auto pool = ((wrapper::Proxy<brepdb::GeometryPool>*)ves_toforeign(0))->obj;
    auto shape = ((wrapper::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    const TopoDS_Shape& tshape = shape->GetShape();
    
    brepdb::GeomSender sender(partgraph::GlobalConfig::Instance()->GetTopoNaming());
    
    TopTools_IndexedMapOfShape all_shapes;
    TopExp::MapShapes(tshape, all_shapes);
    
    for (int i = 1; i <= all_shapes.Extent(); ++i)
    {
        const TopoDS_Shape& shape = all_shapes(i);
    
        uint32_t uid = sender.GetUID(shape);
        if (uid == 0xffffffff)
        {
            TopAbs_ShapeEnum type = shape.ShapeType();
            assert(type != TopAbs_VERTEX && type != TopAbs_EDGE && 
                type != TopAbs_FACE && type != TopAbs_SOLID);
            continue;
        }
    
        switch (shape.ShapeType())
        {
        case TopAbs_SOLID:
            sender.SerializeSolid(TopoDS::Solid(shape), uid, *pool);
            break;
        case TopAbs_FACE:
            sender.SerializeFace(TopoDS::Face(shape), uid, *pool);
            break;
        case TopAbs_EDGE:
            sender.SerializeEdge(TopoDS::Edge(shape), uid, *pool);
            break;
        case TopAbs_VERTEX:
            sender.SerializeVertex(TopoDS::Vertex(shape), uid, *pool);
            break;
        default:
            break;
        }
    }
}

void w_BrepIR_deserialize()
{
    auto pool = ((wrapper::Proxy<brepdb::GeometryPool>*)ves_toforeign(0))->obj;
        
    brepdb::GeomReceiver receiver(*pool);
    BRep_Builder B;
    TopoDS_Compound root_compound;
    B.MakeCompound(root_compound);
        
    for (const auto& h : pool->headers)
    {
        if (h.type == brepdb::Type::Solid)
        {
            TopoDS_Shape solid = receiver.GetShape(h.persistent_id);
            if (!solid.IsNull()) {
                B.Add(root_compound, solid);
            }
        }
    }
        
    auto shape = std::make_shared<partgraph::TopoShape>(root_compound);
    partgraph::return_topo_shape(shape);
}

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
    auto pool = ((wrapper::Proxy<brepdb::GeometryPool>*)ves_toforeign(1))->obj;

    //for (const auto& h : pool->headers)
    //{
    //    const spatialdb::Region aabb(h.min_pt, h.max_pt);
    //    const spatialdb::id_type id = h.persistent_id;
    //    rtree->InsertData(h.param_count, (uint8_t*)&pool->data_pool[h.param_offset], aabb, id);
    //}

    size_t len = sizeof(size_t) + sizeof(brepdb::Header) * pool->headers.size() + sizeof(double) * pool->data_pool.size();
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

        auto pool = std::make_shared<brepdb::GeometryPool>();

        uint8_t* ptr = data;
        size_t num = 0;
        memcpy(&num, ptr, sizeof(num));
        ptr += sizeof(num);

        for (size_t i = 0; i < num; ++i)
        {
            brepdb::Header h;
            memcpy(&h, ptr, sizeof(h));
            ptr += sizeof(h);
            pool->headers.emplace_back(h);
        }

        pool->data_pool.assign((double*)ptr, (double*)(data + len));

        delete[] data;

        ves_pop(ves_argnum());

        ves_pushnil();
        ves_import_class("brepdb", "BrepIR");
        auto proxy = (wrapper::Proxy<brepdb::GeometryPool>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<brepdb::GeometryPool>));
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
    if (strcmp(signature, "BrepIR.save(_)") == 0) return w_BrepIR_save;
    if (strcmp(signature, "BrepIR.load(_)") == 0) return w_BrepIR_load;

    if (strcmp(signature, "BrepIR.serialize(_)") == 0) return w_BrepIR_serialize;
    if (strcmp(signature, "BrepIR.deserialize()") == 0) return w_BrepIR_deserialize;

    if (strcmp(signature, "BrepDB.insert(_)") == 0) return w_BrepDB_insert;
    if (strcmp(signature, "BrepDB.query(_)") == 0) return w_BrepDB_query;

    return nullptr;
}

void BrepDBBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
    if (strcmp(class_name, "BrepIR") == 0)
    {
        methods->allocate = w_BrepIR_allocate;
        methods->finalize = w_BrepIR_finalize;
        return;
    }

    if (strcmp(class_name, "BrepDB") == 0)
    {
        methods->allocate = w_BrepDB_allocate;
        methods->finalize = w_BrepDB_finalize;
        return;
    }
}

}