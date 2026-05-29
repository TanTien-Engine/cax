#include "cadcvt_c/wrap_CadCvt.h"
#include "cadcvt_c/reader/FreeCadReader.h"
#include "cadapp_c/emitter/Replayer.h"

#include "brepkit_c/TransHelper.h"
#include "brepkit_c/TopoShape.h"

#include <wrapper/Proxy.h>

#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>

#include <cstring>
#include <memory>
#include <string>
#include <vector>

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

    // Per-part split of the last load(), with source appearance.
    // Populated from ReplayResult::parts so the host can render each
    // part with its own transparency (FreeCAD-style see-through
    // assemblies). load() still returns the merged shape for
    // back-compat; part_count / part_shape / part_transparency
    // expose this vector.
    std::vector<cadapp::ReplayPart> parts;
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

    // One-shot load: replay each top-level part independently and in
    // parallel. ReplayParts splits the document per part, replays each
    // through its own isolated CalcGraph/TopoNaming on a thread pool, and
    // merges the shapes -- geometry-identical to a serial Replay but
    // spread across cores (multi-instance assemblies see a large speedup).
    // It auto-falls back to a whole-document serial Replay when the parts
    // are not independent (patterns / clones that cross-reference). Safe
    // here precisely because write_back / commit are off: per-part naming
    // is transient and never persisted.
    cadapp::ReplayResult res;
    if (!st->replayer.ReplayParts(doc, opt, res, /*parallel*/ true))
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

    // Stash the per-part split so part_count / part_shape /
    // part_transparency can serve it after this load returns.
    st->parts = std::move(res.parts);

    brepkit::return_topo_shape(res.shape);
}

// Number of parts produced by the last load(). 0 before any load
// or when the load produced no shape.
void w_FreeCadLoader_part_count()
{
    auto* st = GetState(0);
    ves_set_number(0, st ? (double)st->parts.size() : 0.0);
}

// i-th part's shape (TopoShape) from the last load(). nil on a bad
// index so the host can stop iterating defensively.
void w_FreeCadLoader_part_shape()
{
    auto* st = GetState(0);
    if (!st) {
        ves_set_nil(0);
        return;
    }
    int i = (int)ves_tonumber(1);
    if (i < 0 || i >= (int)st->parts.size() || !st->parts[i].shape) {
        ves_set_nil(0);
        return;
    }
    brepkit::return_topo_shape(st->parts[i].shape);
}

// i-th part's transparency (0 opaque .. 1 fully transparent) from
// the last load(). 0 on a bad index.
void w_FreeCadLoader_part_transparency()
{
    auto* st = GetState(0);
    if (!st) {
        ves_set_number(0, 0.0);
        return;
    }
    int i = (int)ves_tonumber(1);
    double t = (i >= 0 && i < (int)st->parts.size())
                   ? st->parts[i].transparency
                   : 0.0;
    ves_set_number(0, t);
}

// Merge the last load()'s parts into a single TopoShape, selecting
// either the opaque (transparency ~ 0) or the transparent
// (transparency > 0) subset. Returns nil when the subset is empty
// so the host can skip wiring an empty draw. Used by the two-pass
// renderer: opaque subset draws with depth writes + edges, the
// transparent subset draws after with blend + depth writes off so
// interiors show through.
std::shared_ptr<brepkit::TopoShape>
MergePartsSubset(const std::vector<cadapp::ReplayPart>& parts,
                 bool                                  want_transparent)
{
    const double k_eps = 1e-6;

    BRep_Builder    bb;
    TopoDS_Compound comp;
    bb.MakeCompound(comp);

    int                                 added = 0;
    std::shared_ptr<brepkit::TopoShape> solo;
    for (const auto& p : parts)
    {
        bool is_transparent = p.transparency > k_eps;
        if (is_transparent != want_transparent) {
            continue;
        }
        if (!p.shape) {
            continue;
        }
        const TopoDS_Shape& s = p.shape->GetShape();
        if (s.IsNull()) {
            continue;
        }
        bb.Add(comp, s);
        if (added == 0) {
            solo = p.shape;
        }
        ++added;
    }

    if (added == 0) {
        return nullptr;
    }
    if (added == 1) {
        return solo;            // avoid a 1-child compound wrapper
    }
    return std::make_shared<brepkit::TopoShape>(comp);
}

// Merged opaque subset of the last load(); nil when there are none.
void w_FreeCadLoader_merged_opaque()
{
    auto* st = GetState(0);
    if (!st) {
        ves_set_nil(0);
        return;
    }
    auto sh = MergePartsSubset(st->parts, /*want_transparent*/ false);
    if (!sh) {
        ves_set_nil(0);
        return;
    }
    brepkit::return_topo_shape(sh);
}

// Merged transparent subset of the last load(); nil when there are
// none (e.g. a model with no transparency authored -> host draws
// only the opaque pass).
void w_FreeCadLoader_merged_transparent()
{
    auto* st = GetState(0);
    if (!st) {
        ves_set_nil(0);
        return;
    }
    auto sh = MergePartsSubset(st->parts, /*want_transparent*/ true);
    if (!sh) {
        ves_set_nil(0);
        return;
    }
    brepkit::return_topo_shape(sh);
}

// Transparent parts as a LIST of TopoShapes (NOT merged), so the host
// can build a per-part mesh and bake each part's own alpha. Parallel
// to transparent_alphas(). Empty list when there are none.
void w_FreeCadLoader_transparent_parts()
{
    const double k_eps = 1e-6;
    std::vector<std::shared_ptr<brepkit::TopoShape>> list;

    auto* st = GetState(0);
    if (st)
    {
        for (const auto& p : st->parts)
        {
            if (p.transparency > k_eps && p.shape) {
                list.push_back(p.shape);
            }
        }
    }
    brepkit::return_topo_shape_list(list);
}

// Per-part alpha (1 - transparency) for the transparent parts, in the
// same order as transparent_parts(). A number list the host zips with
// the shape list when building meshes.
void w_FreeCadLoader_transparent_alphas()
{
    const double k_eps = 1e-6;
    std::vector<double> alphas;

    auto* st = GetState(0);
    if (st)
    {
        for (const auto& p : st->parts)
        {
            if (p.transparency > k_eps && p.shape) {
                alphas.push_back(1.0 - p.transparency);
            }
        }
    }

    ves_pop(ves_argnum());
    ves_newlist((int)alphas.size());
    for (int i = 0; i < (int)alphas.size(); ++i)
    {
        ves_pushnumber(alphas[i]);
        ves_seti(-2, i);
        ves_pop(1);
    }
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
    if (std::strcmp(signature, "FreeCadLoader.part_count()") == 0) {
        return w_FreeCadLoader_part_count;
    }
    if (std::strcmp(signature, "FreeCadLoader.part_shape(_)") == 0) {
        return w_FreeCadLoader_part_shape;
    }
    if (std::strcmp(signature, "FreeCadLoader.part_transparency(_)") == 0) {
        return w_FreeCadLoader_part_transparency;
    }
    if (std::strcmp(signature, "FreeCadLoader.merged_opaque()") == 0) {
        return w_FreeCadLoader_merged_opaque;
    }
    if (std::strcmp(signature, "FreeCadLoader.merged_transparent()") == 0) {
        return w_FreeCadLoader_merged_transparent;
    }
    if (std::strcmp(signature, "FreeCadLoader.transparent_parts()") == 0) {
        return w_FreeCadLoader_transparent_parts;
    }
    if (std::strcmp(signature, "FreeCadLoader.transparent_alphas()") == 0) {
        return w_FreeCadLoader_transparent_alphas;
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
