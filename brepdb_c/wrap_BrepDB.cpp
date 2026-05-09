#include "brepdb_c/wrap_BrepDB.h"
#include "brepdb_c/GeomPool.h"
#include "brepdb_c/GeomSender.h"
#include "brepdb_c/GeomReceiver.h"
#include "brepdb_c/GeomFile.h"
#include "brepdb_c/BrepDB.h"
#include "brepdb_c/VersionTree.h"

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
#include <TopExp_Explorer.hxx>
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
    auto proxy = (wrapper::Proxy<brepdb::BrepDB>*)ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<brepdb::BrepDB>));

    auto num = ves_argnum();
    if (num < 2)
    {
        auto sm = std::make_shared<spatialdb::DiskStorageManager>("test_db");
        proxy->obj = std::make_shared<brepdb::BrepDB>(sm, true);
    }
    else
    {
        auto sm = ((wrapper::Proxy<spatialdb::DiskStorageManager>*)ves_toforeign(1))->obj;
        proxy->obj = std::make_shared<brepdb::BrepDB>(sm, false);
    }
}

int w_BrepDB_finalize(void* data)
{
    auto proxy = (wrapper::Proxy<brepdb::BrepDB>*)(data);
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<brepdb::BrepDB>);
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

    size_t len = sizeof(size_t) + sizeof(brepdb::GeomHeader) * pool->headers.size() + sizeof(double) * pool->data_pool.size();
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

void w_BrepDB_build()
{
    auto db = ((wrapper::Proxy<brepdb::BrepDB>*)ves_toforeign(0))->obj;
    auto shape = ((wrapper::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    auto root = shape->GetShape();

    brepdb::GeomSender sender(partgraph::GlobalConfig::Instance()->GetTopoNaming());

    brepdb::GeometryPool pool;
    TopTools_IndexedMapOfShape all_shapes;
    TopExp::MapShapes(root, all_shapes);

    for (int i = 1; i <= all_shapes.Extent(); ++i)
    {
        const TopoDS_Shape& shape = all_shapes(i);
        uint32_t uid = sender.GetUID(shape);
        if (uid == 0xffffffff) continue;

        switch (shape.ShapeType())
        {
        case TopAbs_SOLID:  sender.SerializeSolid(TopoDS::Solid(shape), uid, pool); break;
        case TopAbs_FACE:   sender.SerializeFace(TopoDS::Face(shape), uid, pool);   break;
        case TopAbs_EDGE:   sender.SerializeEdge(TopoDS::Edge(shape), uid, pool);   break;
        case TopAbs_VERTEX: sender.SerializeVertex(TopoDS::Vertex(shape), uid, pool); break;
        default: break;
        }
    }

    db->ImportPool(pool);

    brepdb::TopoGraph& topo = db->GetTopoGraph();

    for (TopExp_Explorer solid_exp(root, TopAbs_SOLID); solid_exp.More(); solid_exp.Next())
    {
        uint32_t solid_uid = sender.GetUID(solid_exp.Current());
        if (solid_uid == 0xffffffff) continue;

        brepdb::TopoBlock& block = topo.CreateBlock(solid_uid);

        // Solid ¡ú Face
        for (TopExp_Explorer fe(solid_exp.Current(), TopAbs_FACE); fe.More(); fe.Next()) {
            uint32_t fuid = sender.GetUID(fe.Current());
            if (fuid != 0xffffffff)
                block.AddEdge(solid_uid, fuid, brepdb::TopoBlock::FaceOfSolid);
        }

        // Face ¡ú Edge
        for (TopExp_Explorer fe(solid_exp.Current(), TopAbs_FACE); fe.More(); fe.Next()) {
            uint32_t fuid = sender.GetUID(fe.Current());
            if (fuid == 0xffffffff) continue;
            for (TopExp_Explorer ee(fe.Current(), TopAbs_EDGE); ee.More(); ee.Next()) {
                uint32_t euid = sender.GetUID(ee.Current());
                if (euid != 0xffffffff)
                    block.AddEdge(fuid, euid, brepdb::TopoBlock::EdgeOfFace);
            }
        }

        // Edge ¡ú Vertex
        for (TopExp_Explorer ee(solid_exp.Current(), TopAbs_EDGE); ee.More(); ee.Next()) {
            uint32_t euid = sender.GetUID(ee.Current());
            if (euid == 0xffffffff) continue;
            TopoDS_Vertex vf, vl;
            TopExp::Vertices(TopoDS::Edge(ee.Current()), vf, vl);
            auto add_v = [&](const TopoDS_Vertex& v) {
                if (!v.IsNull()) {
                    uint32_t vid = sender.GetUID(v);
                    if (vid != 0xffffffff) block.AddEdge(euid, vid, brepdb::TopoBlock::VertexOfEdge);
                }
            };
            add_v(vf); add_v(vl);
        }

        TopTools_IndexedDataMapOfShapeListOfShape edge_face_map;
        TopExp::MapShapesAndAncestors(
            solid_exp.Current(), TopAbs_EDGE, TopAbs_FACE, edge_face_map);

        for (int ei = 1; ei <= edge_face_map.Extent(); ++ei)
        {
            const TopTools_ListOfShape& faces = edge_face_map(ei);
            std::vector<uint32_t> fuids;
            for (auto it = faces.cbegin(); it != faces.cend(); ++it) {
                uint32_t f = sender.GetUID(*it);
                if (f != 0xffffffff) fuids.push_back(f);
            }
            for (size_t i = 0; i < fuids.size(); ++i)
                for (size_t j = i + 1; j < fuids.size(); ++j)
                    block.AddFaceAdjacency(fuids[i], fuids[j]);
        }
    }

    topo.FinalizeAll();

    db->Flush();
}

void w_BrepDB_query()
{
    auto db = ((wrapper::Proxy<brepdb::BrepDB>*)ves_toforeign(0))->obj;
    sm::cube* aabb = (sm::cube*)ves_toforeign(1);

    const double min[] = { aabb->xmin, aabb->ymin, aabb->zmin };
    const double max[] = { aabb->xmax, aabb->ymax, aabb->zmax };
    spatialdb::Region region(min, max);

    auto visitor = std::make_unique<spatialdb::ObjVisitor>();
    db->GetRTree().IntersectsWithQuery(region, *visitor);

    auto pool = std::make_shared<brepdb::GeometryPool>();
    size_t offset = 0;

    auto& items = visitor->GetResults();
    for (auto item : items)
    {
        spatialdb::id_type id = item->GetIdentifier();

        uint32_t len = 0;
        uint8_t* data = nullptr;
        item->GetData(len, &data);

        uint8_t* ptr = data;

        brepdb::GeomHeader header;
        memcpy(&header, ptr, sizeof(brepdb::GeomHeader));
        ptr += sizeof(brepdb::GeomHeader);

        header.param_offset = offset;

        std::vector<double> item_data(header.param_count);
        assert(header.param_count * sizeof(double) == len - sizeof(brepdb::GeomHeader));
        memcpy(item_data.data(), ptr, header.param_count * sizeof(double));

        pool->headers.emplace_back(header);
        std::copy(item_data.begin(), item_data.end(), std::back_inserter(pool->data_pool));

        offset += header.param_count;

        delete[] data;
    }

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("brepdb", "BrepIR");
    auto proxy = (wrapper::Proxy<brepdb::GeometryPool>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<brepdb::GeometryPool>));
    proxy->obj = pool;
    ves_pop(1);
}

void w_VersionTree_allocate()
{
    auto proxy = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(
        ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<brepdb::VersionTree>)));
    proxy->obj = std::make_shared<brepdb::VersionTree>();
}

int w_VersionTree_finalize(void* data)
{
    auto proxy = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(data);
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<brepdb::VersionTree>);
}

static void push_ir_with_pool(const std::shared_ptr<brepdb::GeometryPool>& pool)
{
    ves_pushnil();
    ves_import_class("brepdb", "BrepIR");
    auto proxy = reinterpret_cast<wrapper::Proxy<brepdb::GeometryPool>*>(
        ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<brepdb::GeometryPool>)));
    proxy->obj = pool;
    ves_pop(1);
}

void w_VersionTree_init_pool()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    auto pool = ((wrapper::Proxy<brepdb::GeometryPool>*)ves_toforeign(1))->obj;
    const char* desc = ves_tostring(2);
    vt->Commit(*pool, desc ? desc : "initial");
}

void w_VersionTree_commit()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    auto pool = ((wrapper::Proxy<brepdb::GeometryPool>*)ves_toforeign(1))->obj;
    const char* desc = ves_tostring(2);
    uint32_t id = vt->Commit(*pool, desc ? desc : "");
    ves_set_number(0, id);
}

void w_VersionTree_undo()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    if (!vt->CanUndo()) { ves_set_nil(0); return; }
    auto pool = vt->Undo();
    ves_pop(ves_argnum());
    push_ir_with_pool(pool);
}

void w_VersionTree_redo()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    if (!vt->CanRedo()) { ves_set_nil(0); return; }
    auto pool = vt->Redo();
    ves_pop(ves_argnum());
    push_ir_with_pool(pool);
}

void w_VersionTree_checkout()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    auto pool = vt->Checkout(static_cast<uint32_t>(ves_tonumber(1)));
    ves_pop(ves_argnum());
    push_ir_with_pool(pool);
}

void w_VersionTree_get_current_id() 
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    ves_set_number(0, vt->GetCurrentId());
}

void w_VersionTree_get_node_count() 
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    ves_set_number(0, static_cast<double>(vt->GetNodeCount()));
}

void w_VersionTree_can_undo() 
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    ves_set_boolean(0, vt->CanUndo());
}

void w_VersionTree_can_redo() 
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    ves_set_boolean(0, vt->CanRedo());
}

void w_VersionTree_get_node_desc()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    auto node = vt->GetNode(static_cast<uint32_t>(ves_tonumber(1)));
    if (node) {
        ves_set_lstring(0, node->op_desc.c_str(), node->op_desc.size());
    } else {
        ves_set_nil(0);
    }
}

void w_VersionTree_get_children()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    auto node = vt->GetNode(static_cast<uint32_t>(ves_tonumber(1)));
    if (!node) { ves_set_nil(0); return; }

    int num = static_cast<int>(node->children.size());
    ves_pop(ves_argnum());
    ves_newlist(num);
    for (int i = 0; i < num; ++i)
    {
        ves_pushnumber(node->children[i]);
        ves_seti(-2, i);
        ves_pop(1);
    }
}

void w_VersionTree_save()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    const char* fp = ves_tostring(1);
    ves_set_boolean(0, vt->SaveToFile(fp ? fp : ""));
}

void w_VersionTree_load()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    const char* fp = ves_tostring(1);
    ves_set_boolean(0, vt->LoadFromFile(fp ? fp : ""));
}

void w_VersionTree_clear() {

    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    vt->Clear();
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

    if (strcmp(signature, "BrepDB.build(_)") == 0) return w_BrepDB_build;
    if (strcmp(signature, "BrepDB.query(_)") == 0) return w_BrepDB_query;

    if (strcmp(signature, "VersionTree.init_pool(_,_)") == 0) return w_VersionTree_init_pool;
    if (strcmp(signature, "VersionTree.commit(_,_)") == 0) return w_VersionTree_commit;
    if (strcmp(signature, "VersionTree.undo()") == 0) return w_VersionTree_undo;
    if (strcmp(signature, "VersionTree.redo()") == 0) return w_VersionTree_redo;
    if (strcmp(signature, "VersionTree.checkout(_)") == 0) return w_VersionTree_checkout;
    if (strcmp(signature, "VersionTree.get_current_id()") == 0) return w_VersionTree_get_current_id;
    if (strcmp(signature, "VersionTree.get_node_count()") == 0) return w_VersionTree_get_node_count;
    if (strcmp(signature, "VersionTree.can_undo()") == 0) return w_VersionTree_can_undo;
    if (strcmp(signature, "VersionTree.can_redo()") == 0) return w_VersionTree_can_redo;
    if (strcmp(signature, "VersionTree.get_node_desc(_)") == 0) return w_VersionTree_get_node_desc;
    if (strcmp(signature, "VersionTree.get_children(_)") == 0) return w_VersionTree_get_children;
    if (strcmp(signature, "VersionTree.save(_)") == 0) return w_VersionTree_save;
    if (strcmp(signature, "VersionTree.load(_)") == 0) return w_VersionTree_load;
    if (strcmp(signature, "VersionTree.clear()") == 0) return w_VersionTree_clear;

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

    if (strcmp(class_name, "VersionTree") == 0)
    {
        methods->allocate = w_VersionTree_allocate;
        methods->finalize = w_VersionTree_finalize;
    }
}

}