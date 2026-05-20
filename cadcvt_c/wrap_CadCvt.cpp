#include "cadcvt_c/wrap_CadCvt.h"
#include "cadcvt_c/reader/FreeCadReader.h"
#include "cadapp_c/emitter/Replayer.h"

#include "brepkit_c/TransHelper.h"
#include "brepkit_c/TopoShape.h"

#include <wrapper/Proxy.h>

#include <cstring>
#include <memory>
#include <string>

// ============================================================
// wrap_CadCvt.cpp
//
// Vessel foreign-method bindings for the cadcvt module.
//
// Right now this exposes a single helper class FreeCadLoader,
// which combines a FreeCadReader + Replayer call. The caller gets
// back a brepkit::TopoShape ready to plug into the rest of the
// node graph.
// ============================================================

namespace cadcvt
{

// ============================================================
// FreeCadLoader foreign class
// ============================================================
//
// Foreign objects store a pointer to this state struct via the
// vessel allocator/finalizer pair. The reader/replayer themselves
// are kept here so options set before load() (unit scale, strict)
// stick to the same instance.

namespace
{

struct FreeCadLoaderState
{
    FreeCadReader      reader;       // cadcvt::FreeCadReader
    cadapp::Replayer   replayer;
    std::string        last_error;
};


// ---- allocator / finalizer ----

void w_FreeCadLoader_allocate()
{
    auto* proxy = (wrapper::Proxy<FreeCadLoaderState>*)ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<FreeCadLoaderState>));
    proxy->obj = std::make_shared<FreeCadLoaderState>();
}

int w_FreeCadLoader_finalize(void* data)
{
    auto* proxy = (wrapper::Proxy<FreeCadLoaderState>*)data;
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<FreeCadLoaderState>);
}


// ---- methods ----

FreeCadLoaderState* GetState(int slot)
{
    auto* p = (wrapper::Proxy<FreeCadLoaderState>*)ves_toforeign(slot);
    return p ? p->obj.get() : nullptr;
}

void w_FreeCadLoader_set_unit_scale()
{
    auto* st = GetState(0);
    if (!st) {
        return;
    }
    double s = ves_tonumber(1);
    st->reader.SetUnitScale(s);
}

void w_FreeCadLoader_set_strict()
{
    auto* st = GetState(0);
    if (!st) {
        return;
    }
    bool b = ves_toboolean(1);
    st->reader.SetStrict(b);
}

void w_FreeCadLoader_load()
{
    auto* st = GetState(0);
    if (!st)
    {
        ves_set_nil(0);
        return;
    }

    const char* path = ves_tostring(1);
    if (!path || !*path)
    {
        st->last_error = "FreeCadLoader.load: empty path";
        ves_set_nil(0);
        return;
    }

    // Step 1: read FreeCAD document into DocumentIR.
    cadapp::DocumentIR doc;
    std::string err;
    if (!st->reader.ReadFile(path, doc, &err))
    {
        st->last_error = err.empty() ? "ReadFile failed" : err;
        ves_set_nil(0);
        return;
    }

    // Step 2: replay onto OCCT via brepkit.
    cadapp::ReplayOptions opt;
    opt.write_back_resolved = false;  // no need to mutate doc for one-shot load
    opt.commit_versions     = false;  // host script decides versioning policy

    cadapp::ReplayResult res;
    if (!st->replayer.Replay(doc, opt, res))
    {
        st->last_error = res.err_msg.empty() ? "Replay failed" : res.err_msg;
        ves_set_nil(0);
        return;
    }

    // Some features may have been skipped (Replayer keeps going and
    // collects diagnostics in err_msg). Preserve them for inspection
    // but still return the shape.
    if (!res.err_msg.empty()) {
        st->last_error = res.err_msg;
    }
    else {
        st->last_error.clear();
    }

    brepkit::return_topo_shape(res.shape);
}

void w_FreeCadLoader_last_error()
{
    auto* st = GetState(0);
    if (!st || st->last_error.empty())
    {
        ves_set_nil(0);
        return;
    }
    ves_set_lstring(0, st->last_error.data(), st->last_error.size());
}

} // anonymous namespace


// ============================================================
// Bind entry points
// ============================================================

VesselForeignMethodFn CadCvtBindMethod(const char* signature)
{
    if (std::strcmp(signature, "FreeCadLoader.set_unit_scale(_)") == 0) {
        return w_FreeCadLoader_set_unit_scale;
    }
    if (std::strcmp(signature, "FreeCadLoader.set_strict(_)") == 0) {
        return w_FreeCadLoader_set_strict;
    }
    if (std::strcmp(signature, "FreeCadLoader.load(_)") == 0) {
        return w_FreeCadLoader_load;
    }
    if (std::strcmp(signature, "FreeCadLoader.last_error()") == 0) {
        return w_FreeCadLoader_last_error;
    }
    return nullptr;
}

void CadCvtBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
    if (std::strcmp(class_name, "FreeCadLoader") == 0)
    {
        methods->allocate = w_FreeCadLoader_allocate;
        methods->finalize = w_FreeCadLoader_finalize;
        return;
    }
}

} // namespace cadcvt
