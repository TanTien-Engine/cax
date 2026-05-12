#include "brepdb_c/wrap_BrepDB.h"
#include "brepdb_c/WorldSender.h"
#include "brepdb_c/WorldReceiver.h"
#include "brepdb_c/WorldFile.h"
#include "brepdb_c/StepFile.h"
#include "brepdb_c/BrepDB.h"
#include "brepdb_c/VersionTree.h"
#include "brepdb_c/VersionGraph.h"

#include <graph/Graph.h>

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

// ============================================================
// BrepWorld foreign class
// ============================================================

void w_BrepWorld_allocate()
{
    auto proxy = (wrapper::Proxy<brepdb::BRepWorld>*)ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<brepdb::BRepWorld>));
    proxy->obj = std::make_shared<brepdb::BRepWorld>();
}

int w_BrepWorld_finalize(void* data)
{
    auto proxy = (wrapper::Proxy<brepdb::BRepWorld>*)(data);
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<brepdb::BRepWorld>);
}

void w_BrepWorld_serialize()
{
    auto world = ((wrapper::Proxy<brepdb::BRepWorld>*)ves_toforeign(0))->obj;
    auto shape = ((wrapper::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    const TopoDS_Shape& tshape = shape->GetShape();

    brepdb::WorldSender sender(partgraph::GlobalConfig::Instance()->GetTopoNaming());
    sender.Serialize(tshape, *world);
}

void w_BrepWorld_save()
{
    auto world = ((wrapper::Proxy<brepdb::BRepWorld>*)ves_toforeign(0))->obj;
    std::string filepath = ves_tostring(1);
    brepdb::WorldFile::Save(filepath, *world);
}

void w_BrepWorld_load()
{
    auto world = ((wrapper::Proxy<brepdb::BRepWorld>*)ves_toforeign(0))->obj;
    std::string filepath = ves_tostring(1);
    brepdb::WorldFile::Load(filepath, *world);
}

void w_BrepWorld_deserialize()
{
    auto world = ((wrapper::Proxy<brepdb::BRepWorld>*)ves_toforeign(0))->obj;

    brepdb::WorldReceiver receiver(*world);
    TopoDS_Shape compound = receiver.GetAll();

    auto shape = std::make_shared<partgraph::TopoShape>(compound);
    partgraph::return_topo_shape(shape);
}

void w_BrepWorld_entity_count()
{
    auto world = ((wrapper::Proxy<brepdb::BRepWorld>*)ves_toforeign(0))->obj;
    ves_set_number(0, static_cast<double>(world->EntityCount()));
}

void w_BrepWorld_export_step()
{
    auto world = ((wrapper::Proxy<brepdb::BRepWorld>*)ves_toforeign(0))->obj;
    std::string filepath = ves_tostring(1);
    bool ok = brepdb::StepFile::Export(filepath, *world);
    ves_set_boolean(0, ok);
}

void w_BrepWorld_import_step()
{
    auto world = ((wrapper::Proxy<brepdb::BRepWorld>*)ves_toforeign(0))->obj;
    std::string filepath = ves_tostring(1);
    auto tn = partgraph::GlobalConfig::Instance()->GetTopoNaming();
    bool ok = brepdb::StepFile::Import(filepath, *world, tn);
    ves_set_boolean(0, ok);
}

// ============================================================

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

void w_BrepDB_build()
{
    auto db = ((wrapper::Proxy<brepdb::BrepDB>*)ves_toforeign(0))->obj;
    auto shape = ((wrapper::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    auto root = shape->GetShape();

    brepdb::WorldSender sender(partgraph::GlobalConfig::Instance()->GetTopoNaming());
    brepdb::BRepWorld world;
    sender.Serialize(root, world);

    db->ImportWorld(world);

    brepdb::TopoGraph& topo = db->GetTopoGraph();

    for (TopExp_Explorer solid_exp(root, TopAbs_SOLID); solid_exp.More(); solid_exp.Next())
    {
        uint32_t solid_uid = sender.GetUID(solid_exp.Current());
        if (solid_uid == 0xffffffff) continue;

        brepdb::TopoBlock& block = topo.CreateBlock(solid_uid);

        // Solid �� Face
        for (TopExp_Explorer fe(solid_exp.Current(), TopAbs_FACE); fe.More(); fe.Next()) {
            uint32_t fuid = sender.GetUID(fe.Current());
            if (fuid != 0xffffffff)
                block.AddEdge(solid_uid, fuid, brepdb::TopoBlock::FaceOfSolid);
        }

        // Face �� Edge
        for (TopExp_Explorer fe(solid_exp.Current(), TopAbs_FACE); fe.More(); fe.Next()) {
            uint32_t fuid = sender.GetUID(fe.Current());
            if (fuid == 0xffffffff) continue;
            for (TopExp_Explorer ee(fe.Current(), TopAbs_EDGE); ee.More(); ee.Next()) {
                uint32_t euid = sender.GetUID(ee.Current());
                if (euid != 0xffffffff)
                    block.AddEdge(fuid, euid, brepdb::TopoBlock::EdgeOfFace);
            }
        }

        // Edge �� Vertex
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

    auto world = std::make_shared<brepdb::BRepWorld>();

    auto& items = visitor->GetResults();
    for (auto item : items)
    {
        uint32_t entity_id = static_cast<uint32_t>(item->GetIdentifier());

        uint32_t len = 0;
        uint8_t* data = nullptr;
        item->GetData(len, &data);

        const uint8_t* ptr = data;

        // Blob layout: type(1) + pid(4) + min_pt(24) + max_pt(24) + param_count(4) + params
        brepdb::Type type = static_cast<brepdb::Type>(*ptr); ptr += 1;
        uint32_t pid; std::memcpy(&pid, ptr, 4); ptr += 4;

        brepdb::AabbComp aabb_comp;
        std::memcpy(aabb_comp.min_pt, ptr, 24); ptr += 24;
        std::memcpy(aabb_comp.max_pt, ptr, 24); ptr += 24;

        uint32_t param_count; std::memcpy(&param_count, ptr, 4); ptr += 4;
        std::vector<double> params(param_count);
        if (param_count > 0)
            std::memcpy(params.data(), ptr, param_count * sizeof(double));

        world->RegisterEntity(entity_id);
        world->Types().Set(entity_id, type);
        world->Aabbs().Set(entity_id, aabb_comp);

        if (!params.empty())
        {
            brepdb::ParamsComp pc;
            pc.data = std::move(params);
            world->Params().Set(entity_id, pc);
        }

        delete[] data;
    }

    // Rebuild typed components from flat params
    world->RebuildTypedFromParams();

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("brepdb", "BrepWorld");
    auto proxy = (wrapper::Proxy<brepdb::BRepWorld>*)ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<brepdb::BRepWorld>));
    proxy->obj = world;
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

static void push_world(const brepdb::WorldPtr& world)
{
    ves_pushnil();
    ves_import_class("brepdb", "BrepWorld");
    auto proxy = reinterpret_cast<wrapper::Proxy<brepdb::BRepWorld>*>(
        ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<brepdb::BRepWorld>)));
    proxy->obj = world;
    ves_pop(1);
}

void w_VersionTree_add_root()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    auto world = ((wrapper::Proxy<brepdb::BRepWorld>*)ves_toforeign(1))->obj;
    const char* desc = ves_tostring(2);
    uint32_t id = vt->AddRoot(*world, desc ? desc : "initial");
    ves_set_number(0, id);
}

void w_VersionTree_commit()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    uint32_t root_id = static_cast<uint32_t>(ves_tonumber(1));
    auto world = ((wrapper::Proxy<brepdb::BRepWorld>*)ves_toforeign(2))->obj;
    const char* desc = ves_tostring(3);
    uint32_t id = vt->Commit(root_id, *world, desc ? desc : "");
    ves_set_number(0, id);
}

void w_VersionTree_undo()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    uint32_t root_id = static_cast<uint32_t>(ves_tonumber(1));
    if (!vt->CanUndo(root_id)) { ves_set_nil(0); return; }
    auto world = vt->Undo(root_id);
    ves_pop(ves_argnum());
    push_world(world);
}

void w_VersionTree_redo()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    uint32_t root_id = static_cast<uint32_t>(ves_tonumber(1));
    if (!vt->CanRedo(root_id)) { ves_set_nil(0); return; }
    auto world = vt->Redo(root_id);
    ves_pop(ves_argnum());
    push_world(world);
}

void w_VersionTree_checkout()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    uint32_t root_id = static_cast<uint32_t>(ves_tonumber(1));
    uint32_t node_id = static_cast<uint32_t>(ves_tonumber(2));
    auto world = vt->Checkout(root_id, node_id);
    ves_pop(ves_argnum());
    push_world(world);
}

void w_VersionTree_get_current_id()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    uint32_t root_id = static_cast<uint32_t>(ves_tonumber(1));
    ves_set_number(0, vt->GetCurrentId(root_id));
}

void w_VersionTree_get_node_count()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    ves_set_number(0, static_cast<double>(vt->GetNodeCount()));
}

void w_VersionTree_can_undo()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    uint32_t root_id = static_cast<uint32_t>(ves_tonumber(1));
    ves_set_boolean(0, vt->CanUndo(root_id));
}

void w_VersionTree_can_redo()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    uint32_t root_id = static_cast<uint32_t>(ves_tonumber(1));
    ves_set_boolean(0, vt->CanRedo(root_id));
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

void w_VersionTree_find_root_of()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    uint32_t node_id = static_cast<uint32_t>(ves_tonumber(1));
    ves_set_number(0, vt->FindRootOf(node_id));
}

void w_VersionTree_clear() {

    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    vt->Clear();
}

void w_VersionGraph_allocate()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(1))->obj;
    auto proxy = reinterpret_cast<wrapper::Proxy<brepdb::VersionGraph>*>(
        ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<brepdb::VersionGraph>)));
    proxy->obj = std::make_shared<brepdb::VersionGraph>(*vt);
}

int w_VersionGraph_finalize(void* data)
{
    auto proxy = reinterpret_cast<wrapper::Proxy<brepdb::VersionGraph>*>(data);
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<brepdb::VersionGraph>);
}

void w_VersionGraph_get_graph()
{
    auto vg = reinterpret_cast<wrapper::Proxy<brepdb::VersionGraph>*>(ves_toforeign(0))->obj;

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("graph", "Graph");
    auto proxy = reinterpret_cast<wrapper::Proxy<graph::Graph>*>(
        ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<graph::Graph>)));
    proxy->obj = vg->GetGraph();
    ves_pop(1);
}

}

namespace brepdb
{

VesselForeignMethodFn BrepDBBindMethod(const char* signature)
{
    if (strcmp(signature, "BrepWorld.serialize(_)") == 0) return w_BrepWorld_serialize;
    if (strcmp(signature, "BrepWorld.deserialize()") == 0) return w_BrepWorld_deserialize;
    if (strcmp(signature, "BrepWorld.save(_)") == 0) return w_BrepWorld_save;
    if (strcmp(signature, "BrepWorld.load(_)") == 0) return w_BrepWorld_load;
    if (strcmp(signature, "BrepWorld.entity_count()") == 0) return w_BrepWorld_entity_count;
    if (strcmp(signature, "BrepWorld.export_step(_)") == 0) return w_BrepWorld_export_step;
    if (strcmp(signature, "BrepWorld.import_step(_)") == 0) return w_BrepWorld_import_step;

    if (strcmp(signature, "BrepDB.build(_)") == 0) return w_BrepDB_build;
    if (strcmp(signature, "BrepDB.query(_)") == 0) return w_BrepDB_query;

    if (strcmp(signature, "VersionTree.add_root(_,_)") == 0) return w_VersionTree_add_root;
    if (strcmp(signature, "VersionTree.commit(_,_,_)") == 0) return w_VersionTree_commit;
    if (strcmp(signature, "VersionTree.undo(_)") == 0) return w_VersionTree_undo;
    if (strcmp(signature, "VersionTree.redo(_)") == 0) return w_VersionTree_redo;
    if (strcmp(signature, "VersionTree.checkout(_,_)") == 0) return w_VersionTree_checkout;
    if (strcmp(signature, "VersionTree.get_current_id(_)") == 0) return w_VersionTree_get_current_id;
    if (strcmp(signature, "VersionTree.get_node_count()") == 0) return w_VersionTree_get_node_count;
    if (strcmp(signature, "VersionTree.can_undo(_)") == 0) return w_VersionTree_can_undo;
    if (strcmp(signature, "VersionTree.can_redo(_)") == 0) return w_VersionTree_can_redo;
    if (strcmp(signature, "VersionTree.get_node_desc(_)") == 0) return w_VersionTree_get_node_desc;
    if (strcmp(signature, "VersionTree.get_children(_)") == 0) return w_VersionTree_get_children;
    if (strcmp(signature, "VersionTree.find_root_of(_)") == 0) return w_VersionTree_find_root_of;
    if (strcmp(signature, "VersionTree.save(_)") == 0) return w_VersionTree_save;
    if (strcmp(signature, "VersionTree.load(_)") == 0) return w_VersionTree_load;
    if (strcmp(signature, "VersionTree.clear()") == 0) return w_VersionTree_clear;

    if (strcmp(signature, "VersionGraph.get_graph()") == 0) return w_VersionGraph_get_graph;

    return nullptr;
}

void BrepDBBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
    if (strcmp(class_name, "BrepWorld") == 0)
    {
        methods->allocate = w_BrepWorld_allocate;
        methods->finalize = w_BrepWorld_finalize;
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
        return;
    }

    if (strcmp(class_name, "VersionGraph") == 0)
    {
        methods->allocate = w_VersionGraph_allocate;
        methods->finalize = w_VersionGraph_finalize;
    }
}

}