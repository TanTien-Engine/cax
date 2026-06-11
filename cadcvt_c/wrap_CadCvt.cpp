#include "cadcvt_c/wrap_CadCvt.h"
#include "cadcvt_c/reader/FreeCadReader.h"
#include "cadcvt_c/reader/SwReader.h"
#include "cadcvt_c/reader/ZwReader.h"
#include "cadapp_c/emitter/Replayer.h"
#include "cadapp_c/ir/SketchIR.h"
#include "cadapp_c/ops/sketch_ops.h"

#include "brepkit_c/TransHelper.h"
#include "brepkit_c/TopoShape.h"
#include "brepkit_c/GlobalConfig.h"
#include "brepkit_c/MemProbe.h"

#include "brepgraph_c/computation/CalcGraph.h"
#include "brepdb_c/WorldSender.h"
#include "brepdb_c/VersionTree.h"

#ifdef CAX_ASMSOLVER_OK
#include "cadcvt_c/AsmSession.h"
#endif

#include <wrapper/Proxy.h>

#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>
#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Trsf.hxx>
#include <gp_Pnt.hxx>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
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

// Re-encode a narrow file path for OCCT, whose OSD_OpenFile decodes
// char* as UTF-8 on Windows. Paths arriving here are EITHER UTF-8 (an
// editor .ves scene string) or system-ANSI (argv / ZW3D file APIs on a
// Chinese locale, i.e. GBK): valid UTF-8 passes through untouched,
// anything else is decoded from the ANSI code page by MSVC's narrow
// fs::path constructor and re-encoded as UTF-8. Without this, a GBK
// path reaches OCCT as invalid UTF-8 and the read fails -- the editor
// silently skipped every authored per-feature STEP of a Chinese-named
// ZW3D part.
inline bool PathIsValidUtf8(const std::string& s)
{
    const auto* p   = reinterpret_cast<const unsigned char*>(s.data());
    const auto* end = p + s.size();
    while (p < end)
    {
        if (*p < 0x80) { ++p; continue; }
        int n = 0;
        if      ((*p & 0xE0) == 0xC0) { n = 1; }
        else if ((*p & 0xF0) == 0xE0) { n = 2; }
        else if ((*p & 0xF8) == 0xF0) { n = 3; }
        else { return false; }
        if (end - p <= n) { return false; }
        for (int k = 1; k <= n; ++k) {
            if ((p[k] & 0xC0) != 0x80) { return false; }
        }
        p += n + 1;
    }
    return true;
}

inline std::string PathForOcct(const std::string& p)
{
#ifdef _WIN32
    if (PathIsValidUtf8(p)) {
        return p;
    }
    // u8string() returns std::u8string under C++20 but std::string under
    // C++17; rebuild through iterators so this TU compiles as either.
    const auto u8 = std::filesystem::path(p).u8string();
    return std::string(u8.begin(), u8.end());
#else
    return p;
#endif
}

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

// Number of NON-construction 2D geometries in the sketch const at `step`
// (0 if not a sketch). Construction geometry (reference lines / aux circles)
// is skipped -- it is never part of the profile face the rebuilt sketch
// produces, and including it would break the wire/face assembly.
void w_FreeCadLoader_sketch_geo_count_at()
{
    auto* st = GetState(0);
    int step = (int)ves_tonumber(1);
    const cadapp::SketchIR* sk = SketchAtStep(st, step);
    int cnt = 0;
    if (sk) {
        for (const auto& g : sk->geos) {
            if (!g.construction) ++cnt;
        }
    }
    ves_set_number(0, (double)cnt);
}

// The gi-th NON-construction geometry of the sketch const at `step` as
// [type, params...], where type is the SkGeoType code (1=Point 2=Line 3=Arc
// 4=Circle 5=Ellipse 6=Spline) and params follow SkGeoIR's per-type layout.
// nil on a bad index. (Indexes over non-construction geos to match
// sketch_geo_count_at.)
void w_FreeCadLoader_sketch_geo_at()
{
    auto* st = GetState(0);
    int step = (int)ves_tonumber(1);
    int gi   = (int)ves_tonumber(2);
    const cadapp::SketchIR* sk = SketchAtStep(st, step);
    const cadapp::SkGeoIR* g = nullptr;
    if (sk && gi >= 0) {
        int k = 0;
        for (const auto& cand : sk->geos) {
            if (cand.construction) continue;
            if (k == gi) { g = &cand; break; }
            ++k;
        }
    }
    if (!g) {
        ves_set_nil(0);
        return;
    }
    int n = (int)g->params.size();
    ves_pop(ves_argnum());
    ves_newlist(n + 1);
    ves_pushnumber((double)(int)g->type);
    ves_seti(-2, 0);
    ves_pop(1);
    for (int i = 0; i < n; ++i) {
        ves_pushnumber(g->params[i]);
        ves_seti(-2, i + 1);
        ves_pop(1);
    }
}

// Map a constraint's geo reference (SkGeoIR id) to the index it has among
// the sketch's NON-construction geometry -- the same indexing sketch_geo_at
// and the rebuilt geo nodes use. Returns -1 when the ref is unset (axis /
// origin -> geo_id 0xFFFFFFFF) or points at construction geometry (which the
// rebuilt sub-graph does not materialise), so the host can skip any
// constraint it cannot wire.
static int SketchNonConstrIndexOf(const cadapp::SketchIR* sk, uint32_t geo_id)
{
    if (geo_id == 0xFFFFFFFFu) {
        return -1;
    }
    int k = 0;
    for (const auto& g : sk->geos) {
        if (g.construction) {
            continue;
        }
        if (g.id == geo_id) {
            return k;
        }
        ++k;
    }
    return -1;
}

// Number of constraints on the sketch const at `step` (0 if not a sketch).
// Every entry is one the reader recognised (SkConsType != None); the host
// still drops those it cannot wire (see sketch_cons_at).
void w_FreeCadLoader_sketch_cons_count_at()
{
    auto* st = GetState(0);
    int step = (int)ves_tonumber(1);
    const cadapp::SketchIR* sk = SketchAtStep(st, step);
    int cnt = sk ? (int)sk->cons.size() : 0;
    ves_set_number(0, (double)cnt);
}

// The i-th constraint of the sketch const at `step` as a flat list
//   [cons_type, a_geo, a_pos, b_geo, b_pos, value, driving]
// cons_type is the SkConsType code; a_geo / b_geo index the sketch's
// NON-construction geometry (matching sketch_geo_at) or -1 when the
// reference is an axis / origin / construction geo; a_pos / b_pos are
// SkPointPos (0 none, 1 start, 2 mid, 3 end); driving is 1 / 0. nil on a
// bad index.
void w_FreeCadLoader_sketch_cons_at()
{
    auto* st = GetState(0);
    int step = (int)ves_tonumber(1);
    int ci   = (int)ves_tonumber(2);
    const cadapp::SketchIR* sk = SketchAtStep(st, step);
    if (!sk || ci < 0 || ci >= (int)sk->cons.size()) {
        ves_set_nil(0);
        return;
    }
    const cadapp::SkConsIR& c = sk->cons[(size_t)ci];
    double v[7] = {
        (double)(int)c.type,
        (double)SketchNonConstrIndexOf(sk, c.a.geo_id),
        (double)(int)c.a.point_pos,
        (double)SketchNonConstrIndexOf(sk, c.b.geo_id),
        (double)(int)c.b.point_pos,
        c.value,
        c.driving ? 1.0 : 0.0,
    };
    ves_pop(ves_argnum());
    ves_newlist(7);
    for (int i = 0; i < 7; ++i) {
        ves_pushnumber(v[i]);
        ves_seti(-2, i);
        ves_pop(1);
    }
}

// Build a B-rep face from a flat list of 2D sketch geometry on a plane,
// reusing the proven sketch_face machinery (solver + ConnectEdgesToWires +
// hole-nesting fill). This is what a reconstructed, EDITABLE sketch sub-graph
// feeds through so the face survives holes / multiple loops / arbitrary edge
// order -- things the host-side single-wire build cannot handle.
//
//   geos   : flat [type, nparams, p0, p1, ..., type, nparams, ...] where
//            type is the SkGeoType code and the params follow SkGeoIR layout
//            (Line=[x0,y0,x1,y1] Arc=[cx,cy,r,sa,ea] Circle=[cx,cy,r]).
//   origin / x_dir / normal : the sketch plane, each a [x,y,z] list.
// Returns a TopoShape (face/compound) or nil. Stateless (ignores receiver).
void w_FreeCadLoader_face_from_geos()
{
    std::vector<double> flat;
    int len = ves_len(1);
    flat.reserve(len);
    for (int i = 0; i < len; ++i) {
        ves_geti(1, i);
        flat.push_back(ves_tonumber(-1));
        ves_pop(1);
    }

    double origin[3], x_dir[3], normal[3];
    double* vecs[3] = { origin, x_dir, normal };
    for (int v = 0; v < 3; ++v) {
        for (int i = 0; i < 3; ++i) {
            ves_geti(2 + v, i);
            vecs[v][i] = ves_tonumber(-1);
            ves_pop(1);
        }
    }

    cadapp::SketchIR sk;
    for (int i = 0; i < 3; ++i) {
        sk.plane_origin[i] = origin[i];
        sk.plane_x_dir[i]  = x_dir[i];
        sk.plane_normal[i] = normal[i];
    }

    uint32_t next_id = 1;
    size_t i = 0;
    while (i + 1 < flat.size()) {
        int type = (int)flat[i];
        int np   = (int)flat[i + 1];
        i += 2;
        if (i + (size_t)np > flat.size()) break;
        cadapp::SkGeoIR g;
        g.id   = next_id++;
        g.type = (cadapp::SkGeoType)type;
        g.params.assign(flat.begin() + i, flat.begin() + i + np);
        sk.geos.push_back(std::move(g));
        i += np;
    }

    auto face = cadapp::BuildSketchFace(sk, origin, x_dir, normal);
    if (!face) {
        ves_set_nil(0);
        return;
    }
    brepkit::return_topo_shape(face);
}


// ============================================================
// SwLoader foreign class -- SolidWorks .SLDPRT / .SLDASM
// ============================================================
//
// Same shape as FreeCadLoader: a (SwReader + Replayer) pair kept alive
// across calls so options set before load() stick, plus the per-part
// split for two-pass transparent rendering. The load() body is the
// FreeCadLoader load() with the reader swapped -- ReplayParts is
// reader-agnostic, it consumes the DocumentIR either reader emits.
//
// Note: SwReader.ReadFile drives the installed SolidWorks via COM, so
// the first load() launches SLDWORKS.exe (slow); later loads reuse it.

struct SwLoaderState
{
    cadcvt::SwReader              reader;       // unit scale defaults to 1.0
    cadapp::Replayer             replayer;
    std::string                  last_error;
    std::vector<cadapp::ReplayPart> parts;
    // Construction graph kept from the last load() (single-part serial
    // Replay path), so calc_graph() can hand it to a CalcGraph node and the
    // history-tree rebuild can decompile it -- same as FreeCadLoaderState.
    std::shared_ptr<brepgraph::CalcGraph> calc_graph;
};

void w_SwLoader_allocate()
{
    auto* proxy = (wrapper::Proxy<SwLoaderState>*)ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<SwLoaderState>));
    proxy->obj = std::make_shared<SwLoaderState>();
}

int w_SwLoader_finalize(void* data)
{
    auto* proxy = (wrapper::Proxy<SwLoaderState>*)data;
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<SwLoaderState>);
}

SwLoaderState* GetSwState(int slot)
{
    auto* p = (wrapper::Proxy<SwLoaderState>*)ves_toforeign(slot);
    return p ? p->obj.get() : nullptr;
}

void w_SwLoader_set_unit_scale()
{
    auto* st = GetSwState(0);
    if (st) st->reader.SetUnitScale(ves_tonumber(1));
}

void w_SwLoader_set_strict()
{
    auto* st = GetSwState(0);
    if (st) st->reader.SetStrict(ves_toboolean(1));
}

void w_SwLoader_load()
{
    auto* st = GetSwState(0);
    if (!st) { ves_set_nil(0); return; }

    // Drop the previous load's construction graph so a failed reload can't
    // leave a stale graph behind the rebuild / CalcGraph path.
    st->calc_graph.reset();

    const char* path = ves_tostring(1);
    if (!path || !*path) {
        st->last_error = "SwLoader.load: empty path";
        ves_set_nil(0);
        return;
    }

    cadapp::DocumentIR doc;
    std::string err;
    if (!st->reader.ReadFile(path, doc, &err)) {
        st->last_error = err.empty() ? "ReadFile failed" : err;
        ves_set_nil(0);
        return;
    }

    cadapp::ReplayOptions opt;
    opt.write_back_resolved = false;
    opt.commit_versions     = false;

    const bool parallel = std::getenv("CAX_REPLAY_SERIAL") == nullptr;
    cadapp::ReplayResult res;
    if (!st->replayer.ReplayParts(doc, opt, res, parallel)) {
        st->last_error = res.err_msg.empty() ? "Replay failed" : res.err_msg;
        ves_set_nil(0);
        return;
    }

    st->last_error = res.err_msg;   // diagnostics, even on success
    st->parts      = std::move(res.parts);
    st->calc_graph = res.calc_graph;  // kept by the single-part serial path
    brepkit::return_topo_shape(res.shape);
}

void w_SwLoader_last_error()
{
    auto* st = GetSwState(0);
    if (!st || st->last_error.empty()) { ves_set_nil(0); return; }
    ves_set_lstring(0, st->last_error.data(), st->last_error.size());
}

void w_SwLoader_part_count()
{
    auto* st = GetSwState(0);
    ves_set_number(0, st ? (double)st->parts.size() : 0.0);
}

void w_SwLoader_part_shape()
{
    auto* st = GetSwState(0);
    if (!st) { ves_set_nil(0); return; }
    int i = (int)ves_tonumber(1);
    if (i < 0 || i >= (int)st->parts.size() || !st->parts[i].shape) {
        ves_set_nil(0);
        return;
    }
    brepkit::return_topo_shape(st->parts[i].shape);
}

void w_SwLoader_part_transparency()
{
    auto* st = GetSwState(0);
    if (!st) { ves_set_number(0, 0.0); return; }
    int i = (int)ves_tonumber(1);
    double t = (i >= 0 && i < (int)st->parts.size()) ? st->parts[i].transparency : 0.0;
    ves_set_number(0, t);
}

void w_SwLoader_merged_opaque()
{
    auto* st = GetSwState(0);
    if (!st) { ves_set_nil(0); return; }
    auto sh = MergePartsSubset(st->parts, /*want_transparent*/ false);
    if (!sh) { ves_set_nil(0); return; }
    brepkit::return_topo_shape(sh);
}

// ---- construction-graph + sketch exposure (history-tree rebuild) -------
//
// Mirror of the FreeCadLoader.calc_graph() / sketch_*_at() family but
// reading SwLoaderState. The construction CalcGraph is reader-agnostic
// (the Replayer builds it from either reader's DocumentIR), so the same
// HistGraphBuilder decompiles an SW import into editable nodes once
// these expose the graph + raw sketch geometry. See the FreeCadLoader
// versions above for the per-function contract.

const cadapp::SketchIR* SketchAtStepSw(SwLoaderState* st, int step)
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

void w_SwLoader_calc_graph()
{
    auto* st = GetSwState(0);
    if (!st || !st->calc_graph) {
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

void w_SwLoader_sketch_plane_at()
{
    auto* st = GetSwState(0);
    int step = (int)ves_tonumber(1);
    const cadapp::SketchIR* sk = SketchAtStepSw(st, step);
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

void w_SwLoader_sketch_geo_count_at()
{
    auto* st = GetSwState(0);
    int step = (int)ves_tonumber(1);
    const cadapp::SketchIR* sk = SketchAtStepSw(st, step);
    int cnt = 0;
    if (sk) {
        for (const auto& g : sk->geos) {
            if (!g.construction) ++cnt;
        }
    }
    ves_set_number(0, (double)cnt);
}

void w_SwLoader_sketch_geo_at()
{
    auto* st = GetSwState(0);
    int step = (int)ves_tonumber(1);
    int gi   = (int)ves_tonumber(2);
    const cadapp::SketchIR* sk = SketchAtStepSw(st, step);
    const cadapp::SkGeoIR* g = nullptr;
    if (sk && gi >= 0) {
        int k = 0;
        for (const auto& cand : sk->geos) {
            if (cand.construction) continue;
            if (k == gi) { g = &cand; break; }
            ++k;
        }
    }
    if (!g) {
        ves_set_nil(0);
        return;
    }
    int n = (int)g->params.size();
    ves_pop(ves_argnum());
    ves_newlist(n + 1);
    ves_pushnumber((double)(int)g->type);
    ves_seti(-2, 0);
    ves_pop(1);
    for (int i = 0; i < n; ++i) {
        ves_pushnumber(g->params[i]);
        ves_seti(-2, i + 1);
        ves_pop(1);
    }
}

void w_SwLoader_sketch_cons_count_at()
{
    auto* st = GetSwState(0);
    int step = (int)ves_tonumber(1);
    const cadapp::SketchIR* sk = SketchAtStepSw(st, step);
    int cnt = sk ? (int)sk->cons.size() : 0;
    ves_set_number(0, (double)cnt);
}

void w_SwLoader_sketch_cons_at()
{
    auto* st = GetSwState(0);
    int step = (int)ves_tonumber(1);
    int ci   = (int)ves_tonumber(2);
    const cadapp::SketchIR* sk = SketchAtStepSw(st, step);
    if (!sk || ci < 0 || ci >= (int)sk->cons.size()) {
        ves_set_nil(0);
        return;
    }
    const cadapp::SkConsIR& c = sk->cons[(size_t)ci];
    double v[7] = {
        (double)(int)c.type,
        (double)SketchNonConstrIndexOf(sk, c.a.geo_id),
        (double)(int)c.a.point_pos,
        (double)SketchNonConstrIndexOf(sk, c.b.geo_id),
        (double)(int)c.b.point_pos,
        c.value,
        c.driving ? 1.0 : 0.0,
    };
    ves_pop(ves_argnum());
    ves_newlist(7);
    for (int i = 0; i < 7; ++i) {
        ves_pushnumber(v[i]);
        ves_seti(-2, i);
        ves_pop(1);
    }
}

void w_SwLoader_merged_transparent()
{
    auto* st = GetSwState(0);
    if (!st) { ves_set_nil(0); return; }
    auto sh = MergePartsSubset(st->parts, /*want_transparent*/ true);
    if (!sh) { ves_set_nil(0); return; }
    brepkit::return_topo_shape(sh);
}

void w_SwLoader_transparent_parts()
{
    const double k_eps = 1e-6;
    std::vector<std::shared_ptr<brepkit::TopoShape>> list;
    auto* st = GetSwState(0);
    if (st) {
        for (const auto& p : st->parts) {
            if (p.transparency > k_eps && p.shape) list.push_back(p.shape);
        }
    }
    brepkit::return_topo_shape_list(list);
}

void w_SwLoader_transparent_alphas()
{
    const double k_eps = 1e-6;
    std::vector<double> alphas;
    auto* st = GetSwState(0);
    if (st) {
        for (const auto& p : st->parts) {
            if (p.transparency > k_eps && p.shape) alphas.push_back(1.0 - p.transparency);
        }
    }
    ves_pop(ves_argnum());
    ves_newlist((int)alphas.size());
    for (int i = 0; i < (int)alphas.size(); ++i) {
        ves_pushnumber(alphas[i]);
        ves_seti(-2, i);
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
// ZwLoader -- ZW3D neutral intermediate (.cax.json) -> shape.
//
// Mirror of SwLoader, but the reader is the SDK-free ZwReader: it parses
// the .cax.json the zw_export plugin emits, so NO ZW3D runtime is needed
// here (unlike SwLoader, which drives an installed SolidWorks via COM).
// load(path) reads + replays in one shot and returns the merged shape;
// per-part access is available afterwards. ReplayParts is reader-agnostic,
// so the replay path is identical to SwLoader / FreeCadLoader.
// ============================================================

struct ZwLoaderState
{
    cadcvt::ZwReader                 reader;     // unit scale defaults to 0.001 (mm)
    cadapp::Replayer                 replayer;
    std::string                     last_error;
    std::vector<cadapp::ReplayPart> parts;
    // Construction graph from the last load (single-part serial path), so
    // calc_graph() can hand it to RebuildHistory / HistGraphBuilder to
    // decompile the import into editable nodes -- same as SwLoaderState.
    std::shared_ptr<brepgraph::CalcGraph> calc_graph;
};

void w_ZwLoader_allocate()
{
    auto* proxy = (wrapper::Proxy<ZwLoaderState>*)ves_set_newforeign(0, 0, sizeof(wrapper::Proxy<ZwLoaderState>));
    proxy->obj = std::make_shared<ZwLoaderState>();
}

int w_ZwLoader_finalize(void* data)
{
    auto* proxy = (wrapper::Proxy<ZwLoaderState>*)data;
    proxy->~Proxy();
    return sizeof(wrapper::Proxy<ZwLoaderState>);
}

ZwLoaderState* GetZwState(int slot)
{
    auto* p = (wrapper::Proxy<ZwLoaderState>*)ves_toforeign(slot);
    return p ? p->obj.get() : nullptr;
}

void w_ZwLoader_set_unit_scale()
{
    auto* st = GetZwState(0);
    if (st) st->reader.SetUnitScale(ves_tonumber(1));
}

void w_ZwLoader_set_strict()
{
    auto* st = GetZwState(0);
    if (st) st->reader.SetStrict(ves_toboolean(1));
}

void w_ZwLoader_load()
{
    auto* st = GetZwState(0);
    if (!st) { ves_set_nil(0); return; }

    // Drop the previous load's construction graph so a failed reload can't
    // leave a stale graph behind the RebuildHistory / CalcGraph path.
    st->calc_graph.reset();

    const char* path = ves_tostring(1);
    if (!path || !*path) {
        st->last_error = "ZwLoader.load: empty path";
        ves_set_nil(0);
        return;
    }

    brepkit::MemProbe("ZwLoad: start (before parse)");

    cadapp::DocumentIR doc;
    std::string err;
    if (!st->reader.ReadFile(path, doc, &err)) {
        st->last_error = err.empty() ? "ReadFile failed" : err;
        ves_set_nil(0);
        return;
    }

    brepkit::MemProbe("ZwLoad: after ReadFile (parse)");

    // Load per-feature authored geometry (CdGeomCopy STEP refs the
    // SDK-free ZwReader could only record as paths) into authored_shapes.
    // The Replayer's BakedShape arm turns each into a body-root node.
    // OCCT lives here in the loader, not in the reader (Reader.h contract).
    for (auto& feat : doc.features)
    {
        auto it = feat.ext_strings.find("zw_geometry");
        if (it == feat.ext_strings.end() || it->second.empty()) {
            continue;
        }
        STEPControl_Reader rd;
        if (rd.ReadFile(PathForOcct(it->second).c_str()) != IFSelect_RetDone) {
            std::fprintf(stderr,
                         "[ZwLoad] authored STEP unreadable (feat %u): %s\n",
                         feat.id, it->second.c_str());
            continue;   // missing/unreadable -> Replayer reports the gap
        }
        rd.TransferRoots();
        TopoDS_Shape shp = rd.OneShape();
        if (shp.IsNull()) {
            continue;
        }
        // OCCT reads ZW3D's STEP in its native unit (mm); scale by the
        // reader's unit_scale (mm -> m) about the origin so the authored
        // base matches the metre-scaled parametric features. Same origin
        // scaling the reader applied to the 2D profile coords.
        const double sc = st->reader.UnitScale();
        if (sc != 1.0) {
            gp_Trsf t;
            t.SetScale(gp_Pnt(0.0, 0.0, 0.0), sc);
            shp = BRepBuilderAPI_Transform(shp, t, true).Shape();
        }
        auto ts = std::make_shared<brepkit::TopoShape>(shp);

        // Register the imported base as a VersionTree root (mirrors
        // PrimMaker::Box). Without a version id, a downstream boolean
        // fusing onto it asserts in VersionTree::Merge (the parent id is
        // unknown) when the result is serialized / meshed. The editable
        // RebuildHistory graph evaluates against GlobalConfig's tree, so
        // register there.
        auto gc = brepkit::GlobalConfig::Instance();
        auto tn = gc ? gc->GetTopoNaming()  : nullptr;
        auto vt = gc ? gc->GetVersionTree() : nullptr;
        if (tn && vt) {
            brepdb::BRepWorld  world;
            brepdb::WorldSender sender(tn);
            sender.Serialize(ts->GetShape(), world);
            ts->SetVersionId(vt->AddRoot(world, "zw_base"));
        }

        doc.authored_shapes[feat.id] = ts;
    }

    brepkit::MemProbe("ZwLoad: after authored STEP load");

    cadapp::ReplayOptions opt;
    opt.write_back_resolved = false;
    opt.commit_versions     = false;

    const bool parallel = std::getenv("CAX_REPLAY_SERIAL") == nullptr;
    cadapp::ReplayResult res;
    if (!st->replayer.ReplayParts(doc, opt, res, parallel)) {
        st->last_error = res.err_msg.empty() ? "Replay failed" : res.err_msg;
        ves_set_nil(0);
        return;
    }

    brepkit::MemProbe("ZwLoad: after ReplayParts (replay)");

    st->last_error = res.err_msg;   // diagnostics, even on success
    st->parts      = std::move(res.parts);
    st->calc_graph = res.calc_graph;  // kept by the single-part serial path
    brepkit::return_topo_shape(res.shape);

    brepkit::MemProbe("ZwLoad: done (shape returned)");
}

void w_ZwLoader_last_error()
{
    auto* st = GetZwState(0);
    if (!st || st->last_error.empty()) { ves_set_nil(0); return; }
    ves_set_lstring(0, st->last_error.data(), st->last_error.size());
}

void w_ZwLoader_part_count()
{
    auto* st = GetZwState(0);
    ves_set_number(0, st ? (double)st->parts.size() : 0.0);
}

void w_ZwLoader_part_shape()
{
    auto* st = GetZwState(0);
    if (!st) { ves_set_nil(0); return; }
    int i = (int)ves_tonumber(1);
    if (i < 0 || i >= (int)st->parts.size() || !st->parts[i].shape) {
        ves_set_nil(0);
        return;
    }
    brepkit::return_topo_shape(st->parts[i].shape);
}

// ---- construction-graph + sketch exposure (history-tree rebuild) -------
//
// Mirror of the SwLoader family: RebuildHistory duck-types on the node's
// get_calc_graph(), then HistGraphBuilder reads raw sketch geometry off
// the construction graph through these. All reader-agnostic -- the graph
// is built by the same Replayer, so the SketchIR lookup is identical.

const cadapp::SketchIR* SketchAtStepZw(ZwLoaderState* st, int step)
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

void w_ZwLoader_calc_graph()
{
    auto* st = GetZwState(0);
    if (!st || !st->calc_graph) {
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

void w_ZwLoader_sketch_plane_at()
{
    auto* st = GetZwState(0);
    int step = (int)ves_tonumber(1);
    const cadapp::SketchIR* sk = SketchAtStepZw(st, step);
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

void w_ZwLoader_sketch_geo_count_at()
{
    auto* st = GetZwState(0);
    int step = (int)ves_tonumber(1);
    const cadapp::SketchIR* sk = SketchAtStepZw(st, step);
    int cnt = 0;
    if (sk) {
        for (const auto& g : sk->geos) {
            if (!g.construction) ++cnt;
        }
    }
    ves_set_number(0, (double)cnt);
}

void w_ZwLoader_sketch_geo_at()
{
    auto* st = GetZwState(0);
    int step = (int)ves_tonumber(1);
    int gi   = (int)ves_tonumber(2);
    const cadapp::SketchIR* sk = SketchAtStepZw(st, step);
    const cadapp::SkGeoIR* g = nullptr;
    if (sk && gi >= 0) {
        int k = 0;
        for (const auto& cand : sk->geos) {
            if (cand.construction) continue;
            if (k == gi) { g = &cand; break; }
            ++k;
        }
    }
    if (!g) {
        ves_set_nil(0);
        return;
    }
    int n = (int)g->params.size();
    ves_pop(ves_argnum());
    ves_newlist(n + 1);
    ves_pushnumber((double)(int)g->type);
    ves_seti(-2, 0);
    ves_pop(1);
    for (int i = 0; i < n; ++i) {
        ves_pushnumber(g->params[i]);
        ves_seti(-2, i + 1);
        ves_pop(1);
    }
}

void w_ZwLoader_sketch_cons_count_at()
{
    auto* st = GetZwState(0);
    int step = (int)ves_tonumber(1);
    const cadapp::SketchIR* sk = SketchAtStepZw(st, step);
    int cnt = sk ? (int)sk->cons.size() : 0;
    ves_set_number(0, (double)cnt);
}

void w_ZwLoader_sketch_cons_at()
{
    auto* st = GetZwState(0);
    int step = (int)ves_tonumber(1);
    int ci   = (int)ves_tonumber(2);
    const cadapp::SketchIR* sk = SketchAtStepZw(st, step);
    if (!sk || ci < 0 || ci >= (int)sk->cons.size()) {
        ves_set_nil(0);
        return;
    }
    const cadapp::SkConsIR& c = sk->cons[(size_t)ci];
    double v[7] = {
        (double)(int)c.type,
        (double)SketchNonConstrIndexOf(sk, c.a.geo_id),
        (double)(int)c.a.point_pos,
        (double)SketchNonConstrIndexOf(sk, c.b.geo_id),
        (double)(int)c.b.point_pos,
        c.value,
        c.driving ? 1.0 : 0.0,
    };
    ves_pop(ves_argnum());
    ves_newlist(7);
    for (int i = 0; i < 7; ++i) {
        ves_pushnumber(v[i]);
        ves_seti(-2, i);
        ves_pop(1);
    }
}


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
    if (std::strcmp(signature, "FreeCadLoader.sketch_cons_count_at(_)") == 0) {
        return w_FreeCadLoader_sketch_cons_count_at;
    }
    if (std::strcmp(signature, "FreeCadLoader.sketch_cons_at(_,_)") == 0) {
        return w_FreeCadLoader_sketch_cons_at;
    }
    if (std::strcmp(signature, "FreeCadLoader.face_from_geos(_,_,_,_)") == 0) {
        return w_FreeCadLoader_face_from_geos;
    }

    if (std::strcmp(signature, "SwLoader.set_unit_scale(_)") == 0) {
        return w_SwLoader_set_unit_scale;
    }
    if (std::strcmp(signature, "SwLoader.set_strict(_)") == 0) {
        return w_SwLoader_set_strict;
    }
    if (std::strcmp(signature, "SwLoader.load(_)") == 0) {
        return w_SwLoader_load;
    }
    if (std::strcmp(signature, "SwLoader.last_error()") == 0) {
        return w_SwLoader_last_error;
    }
    if (std::strcmp(signature, "SwLoader.part_count()") == 0) {
        return w_SwLoader_part_count;
    }
    if (std::strcmp(signature, "SwLoader.part_shape(_)") == 0) {
        return w_SwLoader_part_shape;
    }
    if (std::strcmp(signature, "SwLoader.part_transparency(_)") == 0) {
        return w_SwLoader_part_transparency;
    }
    if (std::strcmp(signature, "SwLoader.merged_opaque()") == 0) {
        return w_SwLoader_merged_opaque;
    }
    if (std::strcmp(signature, "SwLoader.merged_transparent()") == 0) {
        return w_SwLoader_merged_transparent;
    }
    if (std::strcmp(signature, "SwLoader.transparent_parts()") == 0) {
        return w_SwLoader_transparent_parts;
    }
    if (std::strcmp(signature, "SwLoader.transparent_alphas()") == 0) {
        return w_SwLoader_transparent_alphas;
    }
    if (std::strcmp(signature, "SwLoader.calc_graph()") == 0) {
        return w_SwLoader_calc_graph;
    }
    if (std::strcmp(signature, "SwLoader.sketch_plane_at(_)") == 0) {
        return w_SwLoader_sketch_plane_at;
    }
    if (std::strcmp(signature, "SwLoader.sketch_geo_count_at(_)") == 0) {
        return w_SwLoader_sketch_geo_count_at;
    }
    if (std::strcmp(signature, "SwLoader.sketch_geo_at(_,_)") == 0) {
        return w_SwLoader_sketch_geo_at;
    }
    if (std::strcmp(signature, "SwLoader.sketch_cons_count_at(_)") == 0) {
        return w_SwLoader_sketch_cons_count_at;
    }
    if (std::strcmp(signature, "SwLoader.sketch_cons_at(_,_)") == 0) {
        return w_SwLoader_sketch_cons_at;
    }
    if (std::strcmp(signature, "SwLoader.face_from_geos(_,_,_,_)") == 0) {
        return w_FreeCadLoader_face_from_geos;  // stateless, ignores receiver
    }

    if (std::strcmp(signature, "ZwLoader.set_unit_scale(_)") == 0) {
        return w_ZwLoader_set_unit_scale;
    }
    if (std::strcmp(signature, "ZwLoader.set_strict(_)") == 0) {
        return w_ZwLoader_set_strict;
    }
    if (std::strcmp(signature, "ZwLoader.load(_)") == 0) {
        return w_ZwLoader_load;
    }
    if (std::strcmp(signature, "ZwLoader.last_error()") == 0) {
        return w_ZwLoader_last_error;
    }
    if (std::strcmp(signature, "ZwLoader.part_count()") == 0) {
        return w_ZwLoader_part_count;
    }
    if (std::strcmp(signature, "ZwLoader.part_shape(_)") == 0) {
        return w_ZwLoader_part_shape;
    }
    if (std::strcmp(signature, "ZwLoader.calc_graph()") == 0) {
        return w_ZwLoader_calc_graph;
    }
    if (std::strcmp(signature, "ZwLoader.sketch_plane_at(_)") == 0) {
        return w_ZwLoader_sketch_plane_at;
    }
    if (std::strcmp(signature, "ZwLoader.sketch_geo_count_at(_)") == 0) {
        return w_ZwLoader_sketch_geo_count_at;
    }
    if (std::strcmp(signature, "ZwLoader.sketch_geo_at(_,_)") == 0) {
        return w_ZwLoader_sketch_geo_at;
    }
    if (std::strcmp(signature, "ZwLoader.sketch_cons_count_at(_)") == 0) {
        return w_ZwLoader_sketch_cons_count_at;
    }
    if (std::strcmp(signature, "ZwLoader.sketch_cons_at(_,_)") == 0) {
        return w_ZwLoader_sketch_cons_at;
    }
    if (std::strcmp(signature, "ZwLoader.face_from_geos(_,_,_,_)") == 0) {
        return w_FreeCadLoader_face_from_geos;  // stateless, ignores receiver
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
    if (std::strcmp(class_name, "SwLoader") == 0)
    {
        methods->allocate = w_SwLoader_allocate;
        methods->finalize = w_SwLoader_finalize;
        return;
    }
    if (std::strcmp(class_name, "ZwLoader") == 0)
    {
        methods->allocate = w_ZwLoader_allocate;
        methods->finalize = w_ZwLoader_finalize;
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
