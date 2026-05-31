#include "cadcvt_c/wrap_CadCvt.h"
#include "cadcvt_c/reader/FreeCadReader.h"
#include "cadapp_c/emitter/Replayer.h"
#include "cadapp_c/ir/SketchIR.h"

#include "brepkit_c/TransHelper.h"
#include "brepkit_c/TopoShape.h"

#include "brepgraph_c/computation/CalcGraph.h"

#ifdef CAX_ASMSOLVER_OK
#include "cadcvt_c/AsmSession.h"
#endif

#include <wrapper/Proxy.h>

#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
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

    // Construction calc graph the replayer built for the last load()
    // (sketch / pad / pocket / fillet / ... ops). Lets a CalcGraph node
    // draw the model's real construction history instead of one opaque
    // const-shape leaf. Only populated when the load took the serial
    // Replay path (single-part docs, which is where ReplayParts falls
    // back to it); nullptr for the parallel multi-part assembly path,
    // whose per-part graphs are isolated and never merged.
    std::shared_ptr<brepgraph::CalcGraph> calc_graph;
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

    // Drop the previous load's construction graph up front so a failed
    // reload can't leave a CalcGraph node drawing a stale graph.
    st->calc_graph.reset();

    const char* path = ves_tostring(1);
    if (!path || !*path)
    {
        st->last_error = "FreeCadLoader.load: empty path";
        ves_set_nil(0);
        return;
    }

    // Optional one-shot load timing (CAX_TIME env). Prints the
    // ReadFile vs ReplayParts split once so a "still slow" report can be
    // localised to parsing, replay, or (if both are fast) host-side
    // meshing / rendering downstream of load().
    const bool t_on = std::getenv("CAX_TIME") != nullptr;
    auto t_read0 = std::chrono::steady_clock::now();

    // Step 1: read FreeCAD document into DocumentIR.
    cadapp::DocumentIR doc;
    std::string err;
    if (!st->reader.ReadFile(path, doc, &err))
    {
        st->last_error = err.empty() ? "ReadFile failed" : err;
        ves_set_nil(0);
        return;
    }
    auto t_read1 = std::chrono::steady_clock::now();

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
    // CAX_REPLAY_SERIAL=1 forces the whole-document serial path (no
    // thread pool) so a "parallel isn't helping" hunch can be A/B'd on
    // platforms where it underperforms (low VM core count, or heap-lock
    // contention serialising concurrent OCCT allocation on Windows).
    const bool parallel = std::getenv("CAX_REPLAY_SERIAL") == nullptr;
    cadapp::ReplayResult res;
    if (!st->replayer.ReplayParts(doc, opt, res, parallel))
    {
        st->last_error = res.err_msg.empty() ? "Replay failed" : res.err_msg;
        ves_set_nil(0);
        return;
    }
    if (t_on) {
        auto t_replay1 = std::chrono::steady_clock::now();
        auto ms = [](auto a, auto b) {
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        std::fprintf(stderr,
                     "[CAX_TIME] FreeCadLoader.load: ReadFile=%.1fms "
                     "ReplayParts=%.1fms parts=%zu parallel=%d cores=%u\n",
                     ms(t_read0, t_read1), ms(t_read1, t_replay1),
                     res.parts.size(), parallel ? 1 : 0,
                     std::thread::hardware_concurrency());
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

    // Keep the construction graph (if the serial Replay path built one)
    // so calc_graph() can hand it to a CalcGraph node for drawing.
    st->calc_graph = res.calc_graph;

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

// Construction calc graph from the last load() (nil when none was kept
// -- e.g. the parallel multi-part assembly path). The host wraps it in a
// CalcGraph foreign and a CalcGraph node draws its full op history.
void w_FreeCadLoader_calc_graph()
{
    auto* st = GetState(0);
    if (!st || !st->calc_graph)
    {
        ves_set_nil(0);
        return;
    }

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("brepgraph", "CalcGraph");
    auto proxy = (wrapper::Proxy<brepgraph::CalcGraph>*)ves_set_newforeign(
        0, 1, sizeof(wrapper::Proxy<brepgraph::CalcGraph>));
    proxy->obj = st->calc_graph;
    ves_pop(1);
}


// ---- sketch geometry exposure (for rebuilding editable sketches) ----
//
// A FreeCAD sketch lives in the construction calc graph as a $sketch const
// (an opaque SketchIR handle) feeding a sketch_face / sketch_wire op. To
// rebuild it as an editable sketchgraph sub-graph, the host needs the raw
// 2D geometry. These read it straight off the const at `step` (the sketch
// input of a sketch_face step), so there is no fragile sketch<->step
// mapping. cadcvt_c can include both brepgraph (CalcGraph) and cadapp
// (SketchIR) without inverting the module dependency, which brepgraph_c
// could not.

const cadapp::SketchIR* SketchAtStep(FreeCadLoaderState* st, int step)
{
    if (!st || !st->calc_graph) {
        return nullptr;
    }
    const brepgraph::OpStep* s = st->calc_graph->GetHistory().Get(step);
    if (!s || s->op_name != "$sketch") {
        return nullptr;
    }
    const auto* sv = std::get_if<brepgraph::SketchVal>(&s->imm);
    if (!sv || !sv->handle) {
        return nullptr;
    }
    return static_cast<const cadapp::SketchIR*>(sv->handle.get());
}

// Plane of the sketch const at `step` as a flat 9-number list:
// [ox,oy,oz, xx,xy,xz, nx,ny,nz] (origin, x_dir, normal). nil if `step`
// is not a sketch const.
void w_FreeCadLoader_sketch_plane_at()
{
    auto* st = GetState(0);
    int step = (int)ves_tonumber(1);
    const cadapp::SketchIR* sk = SketchAtStep(st, step);
    if (!sk) {
        ves_set_nil(0);
        return;
    }
    double v[9] = {
        sk->plane_origin[0], sk->plane_origin[1], sk->plane_origin[2],
        sk->plane_x_dir[0],  sk->plane_x_dir[1],  sk->plane_x_dir[2],
        sk->plane_normal[0], sk->plane_normal[1], sk->plane_normal[2],
    };
    ves_pop(ves_argnum());
    ves_newlist(9);
    for (int i = 0; i < 9; ++i) {
        ves_pushnumber(v[i]);
        ves_seti(-2, i);
        ves_pop(1);
    }
}

// Number of 2D geometries in the sketch const at `step` (0 if not a sketch).
void w_FreeCadLoader_sketch_geo_count_at()
{
    auto* st = GetState(0);
    int step = (int)ves_tonumber(1);
    const cadapp::SketchIR* sk = SketchAtStep(st, step);
    ves_set_number(0, sk ? (double)sk->geos.size() : 0.0);
}

// The gi-th geometry of the sketch const at `step` as [type, params...],
// where type is the SkGeoType code (1=Point 2=Line 3=Arc 4=Circle
// 5=Ellipse 6=Spline) and params follow SkGeoIR's per-type layout. nil on
// a bad index.
void w_FreeCadLoader_sketch_geo_at()
{
    auto* st = GetState(0);
    int step = (int)ves_tonumber(1);
    int gi   = (int)ves_tonumber(2);
    const cadapp::SketchIR* sk = SketchAtStep(st, step);
    if (!sk || gi < 0 || gi >= (int)sk->geos.size()) {
        ves_set_nil(0);
        return;
    }
    const cadapp::SkGeoIR& g = sk->geos[gi];
    int n = (int)g.params.size();
    ves_pop(ves_argnum());
    ves_newlist(n + 1);
    ves_pushnumber((double)(int)g.type);
    ves_seti(-2, 0);
    ves_pop(1);
    for (int i = 0; i < n; ++i) {
        ves_pushnumber(g.params[i]);
        ves_seti(-2, i + 1);
        ves_pop(1);
    }
}


// ============================================================
// AsmSession foreign class -- interactive pick + drag editing
// ============================================================
//
// Stateful: load() builds the constraint graph once, then pick()/drag()
// move parts and the host re-meshes body_shape(i). Only poses change; the
// source file is never written. When the asmsolver (Ceres submodule) is not
// built, the class still binds but every call reports a not-built error, so
// the .ves node degrades gracefully instead of failing to instantiate.

struct AsmSessionState
{
#ifdef CAX_ASMSOLVER_OK
    cadcvt::AsmSession session;
#endif
    std::string err = "asmsolver not built (Ceres submodule absent)";
};

void w_AsmSession_allocate()
{
    auto* proxy = (wrapper::Proxy<AsmSessionState>*)ves_set_newforeign(
        0, 0, sizeof(wrapper::Proxy<AsmSessionState>));
    proxy->obj = std::make_shared<AsmSessionState>();
}

int w_AsmSession_finalize(void* data)
{
    auto* proxy = (wrapper::Proxy<AsmSessionState>*)data;
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<AsmSessionState>);
}

AsmSessionState* GetAsmState(int slot)
{
    auto* proxy = (wrapper::Proxy<AsmSessionState>*)ves_toforeign(slot);
    return proxy ? proxy->obj.get() : nullptr;
}

// load(path, unit_scale, strict) -> bool
void w_AsmSession_load()
{
    auto* st = GetAsmState(0);
    if (!st) { ves_set_boolean(0, false); return; }
#ifdef CAX_ASMSOLVER_OK
    const char* path = ves_tostring(1);
    double      us   = ves_tonumber(2);
    bool        strict = ves_toboolean(3);
    bool ok = st->session.Load(path ? path : "", us, strict);
    if (!ok) st->err = st->session.last_error();
    ves_set_boolean(0, ok);
#else
    ves_set_boolean(0, false);
#endif
}

void w_AsmSession_last_error()
{
#ifdef CAX_ASMSOLVER_OK
    auto* st = GetAsmState(0);
    std::string e = st ? st->session.last_error() : std::string{};
    if (e.empty()) { ves_set_nil(0); return; }
    ves_set_lstring(0, e.data(), e.size());
#else
    auto* st = GetAsmState(0);
    if (st) ves_set_lstring(0, st->err.data(), st->err.size());
    else    ves_set_nil(0);
#endif
}

void w_AsmSession_body_count()
{
#ifdef CAX_ASMSOLVER_OK
    auto* st = GetAsmState(0);
    ves_set_number(0, st ? (double)st->session.body_count() : 0.0);
#else
    ves_set_number(0, 0.0);
#endif
}

void w_AsmSession_body_name()
{
#ifdef CAX_ASMSOLVER_OK
    auto* st = GetAsmState(0);
    int i = (int)ves_tonumber(1);
    std::string n = st ? st->session.body_name(i) : std::string{};
    ves_set_lstring(0, n.data(), n.size());
#else
    ves_set_nil(0);
#endif
}

// body_shape(i) -> TopoShape at the current solved pose (nil on bad index).
void w_AsmSession_body_shape()
{
#ifdef CAX_ASMSOLVER_OK
    auto* st = GetAsmState(0);
    if (!st) { ves_set_nil(0); return; }
    int i = (int)ves_tonumber(1);
    auto sh = st->session.body_shape(i);
    if (!sh) { ves_set_nil(0); return; }
    brepkit::return_topo_shape(sh);
#else
    ves_set_nil(0);
#endif
}

// pick(ox,oy,oz, dx,dy,dz) -> body index (-1 on miss). Stores the hit point
// for hit() and the grab anchor for the next drag().
void w_AsmSession_pick()
{
#ifdef CAX_ASMSOLVER_OK
    auto* st = GetAsmState(0);
    if (!st) { ves_set_number(0, -1.0); return; }
    double ox = ves_tonumber(1), oy = ves_tonumber(2), oz = ves_tonumber(3);
    double dx = ves_tonumber(4), dy = ves_tonumber(5), dz = ves_tonumber(6);
    ves_set_number(0, (double)st->session.Pick(ox, oy, oz, dx, dy, dz));
#else
    ves_set_number(0, -1.0);
#endif
}

// hit() -> [x,y,z] world hit point of the last pick.
void w_AsmSession_hit()
{
    double h[3] = {0, 0, 0};
#ifdef CAX_ASMSOLVER_OK
    auto* st = GetAsmState(0);
    if (st) st->session.last_hit(h);
#endif
    ves_pop(ves_argnum());
    ves_newlist(3);
    for (int i = 0; i < 3; ++i) {
        ves_pushnumber(h[i]);
        ves_seti(-2, i);
        ves_pop(1);
    }
}

// drag(body, tx,ty,tz, weight) -> post-solve joint residual.
void w_AsmSession_drag()
{
#ifdef CAX_ASMSOLVER_OK
    auto* st = GetAsmState(0);
    if (!st) { ves_set_number(0, -1.0); return; }
    int    body = (int)ves_tonumber(1);
    double tx = ves_tonumber(2), ty = ves_tonumber(3), tz = ves_tonumber(4);
    double w  = ves_tonumber(5);
    ves_set_number(0, st->session.Drag(body, tx, ty, tz, w));
#else
    ves_set_number(0, -1.0);
#endif
}

void w_AsmSession_drag_rot()
{
#ifdef CAX_ASMSOLVER_OK
    auto* st = GetAsmState(0);
    if (!st) { ves_set_number(0, -1.0); return; }
    int    body  = (int)ves_tonumber(1);
    double ax = ves_tonumber(2), ay = ves_tonumber(3), az = ves_tonumber(4);
    double angle = ves_tonumber(5), w = ves_tonumber(6);
    ves_set_number(0, st->session.DragRot(body, ax, ay, az, angle, w));
#else
    ves_set_number(0, -1.0);
#endif
}

void w_AsmSession_drive()
{
#ifdef CAX_ASMSOLVER_OK
    auto* st = GetAsmState(0);
    if (!st) { ves_set_number(0, -1.0); return; }
    int    body  = (int)ves_tonumber(1);
    double ax = ves_tonumber(2), ay = ves_tonumber(3), az = ves_tonumber(4);
    double value = ves_tonumber(5), w = ves_tonumber(6);
    ves_set_number(0, st->session.Drive(body, ax, ay, az, value, w));
#else
    ves_set_number(0, -1.0);
#endif
}

void w_AsmSession_snap()
{
#ifdef CAX_ASMSOLVER_OK
    auto* st = GetAsmState(0);
    if (!st) { ves_set_number(0, -1.0); return; }
    ves_set_number(0, st->session.Snap());
#else
    ves_set_number(0, -1.0);
#endif
}

void w_AsmSession_save_back()
{
#ifdef CAX_ASMSOLVER_OK
    auto* st = GetAsmState(0);
    if (!st) { ves_set_boolean(0, false); return; }
    ves_set_boolean(0, st->session.SaveBack());
#else
    ves_set_boolean(0, false);
#endif
}

void w_AsmSession_save()
{
#ifdef CAX_ASMSOLVER_OK
    auto* st = GetAsmState(0);
    if (!st) { ves_set_boolean(0, false); return; }
    const char* p = ves_tostring(1);
    ves_set_boolean(0, p ? st->session.Save(p) : false);
#else
    ves_set_boolean(0, false);
#endif
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
    if (std::strcmp(signature, "FreeCadLoader.calc_graph()") == 0) {
        return w_FreeCadLoader_calc_graph;
    }
    if (std::strcmp(signature, "FreeCadLoader.sketch_plane_at(_)") == 0) {
        return w_FreeCadLoader_sketch_plane_at;
    }
    if (std::strcmp(signature, "FreeCadLoader.sketch_geo_count_at(_)") == 0) {
        return w_FreeCadLoader_sketch_geo_count_at;
    }
    if (std::strcmp(signature, "FreeCadLoader.sketch_geo_at(_,_)") == 0) {
        return w_FreeCadLoader_sketch_geo_at;
    }

    if (std::strcmp(signature, "AsmSession.load(_,_,_)") == 0) {
        return w_AsmSession_load;
    }
    if (std::strcmp(signature, "AsmSession.last_error()") == 0) {
        return w_AsmSession_last_error;
    }
    if (std::strcmp(signature, "AsmSession.body_count()") == 0) {
        return w_AsmSession_body_count;
    }
    if (std::strcmp(signature, "AsmSession.body_name(_)") == 0) {
        return w_AsmSession_body_name;
    }
    if (std::strcmp(signature, "AsmSession.body_shape(_)") == 0) {
        return w_AsmSession_body_shape;
    }
    if (std::strcmp(signature, "AsmSession.pick(_,_,_,_,_,_)") == 0) {
        return w_AsmSession_pick;
    }
    if (std::strcmp(signature, "AsmSession.hit()") == 0) {
        return w_AsmSession_hit;
    }
    if (std::strcmp(signature, "AsmSession.drag(_,_,_,_,_)") == 0) {
        return w_AsmSession_drag;
    }
    if (std::strcmp(signature, "AsmSession.drag_rot(_,_,_,_,_,_)") == 0) {
        return w_AsmSession_drag_rot;
    }
    if (std::strcmp(signature, "AsmSession.drive(_,_,_,_,_,_)") == 0) {
        return w_AsmSession_drive;
    }
    if (std::strcmp(signature, "AsmSession.snap()") == 0) {
        return w_AsmSession_snap;
    }
    if (std::strcmp(signature, "AsmSession.save_back()") == 0) {
        return w_AsmSession_save_back;
    }
    if (std::strcmp(signature, "AsmSession.save(_)") == 0) {
        return w_AsmSession_save;
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
    if (std::strcmp(class_name, "AsmSession") == 0)
    {
        methods->allocate = w_AsmSession_allocate;
        methods->finalize = w_AsmSession_finalize;
        return;
    }
}

} // namespace cadcvt
