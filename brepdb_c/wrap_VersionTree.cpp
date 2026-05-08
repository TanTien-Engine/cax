#include "wrap_VersionTree.h"
#include "VersionTree.h"
#include "BrepDB.h"
#include "GeomPool.h"

#include <wrapper/Proxy.h>

#include <cstring>
#include <memory>

// Integration notes:
//   1. BrepIR is assumed to expose GetPool() -> GeometryPool&.
//      Adjust get_pool_from_ir() and push_ir_with_pool() if the interface differs.
//   2. Register VersionTreeBindMethod / VersionTreeBindClass alongside
//      the existing BrepDB bindings in your module init function.
//   3. Append the VersionTree foreign class block to brepdb.ves.inc.

namespace
{

// ---- allocate / finalize ----

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

// ---- pool access helpers ----

static brepdb::GeometryPool& get_pool_from_ir(int slot)
{
    auto ir = reinterpret_cast<wrapper::Proxy<brepdb::BrepIR>*>(ves_toforeign(slot))->obj;
    return ir->GetPool();
}

static void push_ir_with_pool(const brepdb::GeometryPool& pool)
{
    ves_pushnil();
    ves_import_class("brepdb", "BrepIR");
    auto proxy = reinterpret_cast<wrapper::Proxy<brepdb::BrepIR>*>(
        ves_set_newforeign(0, 1, sizeof(wrapper::Proxy<brepdb::BrepIR>)));
    proxy->obj = std::make_shared<brepdb::BrepIR>();
    proxy->obj->GetPool() = pool;
    ves_pop(1);
}

// ---- init_pool(ir, desc) ----

void w_VersionTree_init_pool()
{
    auto vt   = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    auto& pool = get_pool_from_ir(1);
    const char* desc = ves_tostring(2);
    vt->Init(pool, desc ? desc : "initial");
}

// ---- commit(ir, desc) -> node_id ----

void w_VersionTree_commit()
{
    auto vt    = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    auto& pool = get_pool_from_ir(1);
    const char* desc = ves_tostring(2);
    uint32_t id = vt->Commit(pool, desc ? desc : "");
    ves_set_number(0, id);
}

// ---- undo() -> BrepIR ----

void w_VersionTree_undo()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    if (!vt->CanUndo()) { ves_set_nil(0); return; }
    auto pool = vt->Undo();
    ves_pop(ves_argnum());
    push_ir_with_pool(pool);
}

// ---- redo() -> BrepIR ----

void w_VersionTree_redo()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    if (!vt->CanRedo()) { ves_set_nil(0); return; }
    auto pool = vt->Redo();
    ves_pop(ves_argnum());
    push_ir_with_pool(pool);
}

// ---- checkout(node_id) -> BrepIR ----

void w_VersionTree_checkout()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    auto pool = vt->Checkout(static_cast<uint32_t>(ves_tonumber(1)));
    ves_pop(ves_argnum());
    push_ir_with_pool(pool);
}

// ---- get_current_id() ----

void w_VersionTree_get_current_id() {
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    ves_set_number(0, vt->GetCurrentId());
}

// ---- get_node_count() ----

void w_VersionTree_get_node_count() {
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    ves_set_number(0, static_cast<double>(vt->GetNodeCount()));
}

// ---- can_undo() ----

void w_VersionTree_can_undo() {
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    ves_set_boolean(0, vt->CanUndo());
}

// ---- can_redo() ----

void w_VersionTree_can_redo() {
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    ves_set_boolean(0, vt->CanRedo());
}

// ---- get_node_desc(node_id) -> string ----

void w_VersionTree_get_node_desc()
{
    auto vt  = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    auto node = vt->GetNode(static_cast<uint32_t>(ves_tonumber(1)));
    if (node) {
        ves_set_lstring(0, node->op_desc.c_str(), node->op_desc.size());
    } else {
        ves_set_nil(0);
    }
}

// ---- get_children(node_id) -> list ----

void w_VersionTree_get_children()
{
    auto vt  = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
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

// ---- save(filepath) -> bool ----

void w_VersionTree_save()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    const char* fp = ves_tostring(1);
    ves_set_boolean(0, vt->SaveToFile(fp ? fp : ""));
}

// ---- load(filepath) -> bool ----

void w_VersionTree_load()
{
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    const char* fp = ves_tostring(1);
    ves_set_boolean(0, vt->LoadFromFile(fp ? fp : ""));
}

// ---- clear() ----

void w_VersionTree_clear() {
    auto vt = reinterpret_cast<wrapper::Proxy<brepdb::VersionTree>*>(ves_toforeign(0))->obj;
    vt->Clear();
}

} // anonymous namespace


namespace brepdb
{

VesselForeignMethodFn VersionTreeBindMethod(const char* signature)
{
    if (strcmp(signature, "VersionTree.init_pool(_,_)")    == 0) return w_VersionTree_init_pool;
    if (strcmp(signature, "VersionTree.commit(_,_)")       == 0) return w_VersionTree_commit;
    if (strcmp(signature, "VersionTree.undo()")            == 0) return w_VersionTree_undo;
    if (strcmp(signature, "VersionTree.redo()")            == 0) return w_VersionTree_redo;
    if (strcmp(signature, "VersionTree.checkout(_)")       == 0) return w_VersionTree_checkout;
    if (strcmp(signature, "VersionTree.get_current_id()") == 0) return w_VersionTree_get_current_id;
    if (strcmp(signature, "VersionTree.get_node_count()") == 0) return w_VersionTree_get_node_count;
    if (strcmp(signature, "VersionTree.can_undo()")       == 0) return w_VersionTree_can_undo;
    if (strcmp(signature, "VersionTree.can_redo()")       == 0) return w_VersionTree_can_redo;
    if (strcmp(signature, "VersionTree.get_node_desc(_)") == 0) return w_VersionTree_get_node_desc;
    if (strcmp(signature, "VersionTree.get_children(_)")  == 0) return w_VersionTree_get_children;
    if (strcmp(signature, "VersionTree.save(_)")          == 0) return w_VersionTree_save;
    if (strcmp(signature, "VersionTree.load(_)")          == 0) return w_VersionTree_load;
    if (strcmp(signature, "VersionTree.clear()")          == 0) return w_VersionTree_clear;
    return nullptr;
}

void VersionTreeBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
    if (strcmp(class_name, "VersionTree") == 0)
    {
        methods->allocate = w_VersionTree_allocate;
        methods->finalize = w_VersionTree_finalize;
    }
}

} // namespace brepdb
