#define QUOTE(...) #__VA_ARGS__
static const char* cadcvtModuleSource = QUOTE(

// =========================================================
// Vessel foreign classes for cadcvt.
//
// Currently only the FreeCadLoader entry is exposed; SwLoader and
// SelfLoader will follow the same pattern.
// =========================================================

foreign class FreeCadLoader
{
    init() {}

    // ---- options ----
    foreign set_unit_scale(s)
    foreign set_strict(b)

    // ---- one-shot read + replay ----
    //
    // Returns a TopoShape (or nil on failure). Sets last_error
    // when the call fails so the caller can display the reason.
    foreign load(path)

    foreign last_error()

    // ---- per-part access (valid after load) ----
    //
    // The assembly is also kept split into parts, each with its
    // source GUI appearance, so a host can render per part instead
    // of the single merged shape.
    //   part_count()         number of parts
    //   part_shape(i)        i-th part's TopoShape (nil on bad index)
    //   part_transparency(i) i-th part's transparency (0..1)
    foreign part_count()
    foreign part_shape(i)
    foreign part_transparency(i)

    // Merged opaque / transparent subsets of the last load(), for
    // two-pass see-through rendering. nil when the subset is empty.
    foreign merged_opaque()
    foreign merged_transparent()

    // Transparent parts un-merged, for per-part alpha. transparent_parts()
    // is a list of TopoShape; transparent_alphas() is the parallel list
    // of per-part alpha (1 - transparency). Zip them when building meshes.
    foreign transparent_parts()
    foreign transparent_alphas()

    // Construction calc graph built by the replayer for the last load()
    // (sketch / pad / pocket / fillet / ... ops). Returns a CalcGraph, or
    // nil when none was kept (parallel multi-part assembly path). Feed it
    // to a CalcGraph node to draw the model's real construction history.
    foreign calc_graph()

    // ---- sketch geometry exposure (for rebuilding editable sketches) ----
    //
    // Read the raw 2D geometry of the $sketch const at `step` (the sketch
    // input of a sketch_face / sketch_wire step in calc_graph()), so the
    // host can rebuild it as an editable sketchgraph sub-graph.
    //   sketch_plane_at(step)     -> [ox,oy,oz, xx,xy,xz, nx,ny,nz] or nil
    //   sketch_geo_count_at(step) -> number
    //   sketch_geo_at(step, g)    -> [type, params...] (SkGeoType code +
    //                                per-type params) or nil
    foreign sketch_plane_at(step)
    foreign sketch_geo_count_at(step)
    foreign sketch_geo_at(step, g)
}

// =========================================================
// AsmSession -- interactive assembly editing (pick + drag).
//
// load() reads a FreeCAD native-AssemblyWB file and builds the
// constraint graph once. Then the host pick()s a part by world
// ray, drag()s it toward a target (re-solving the joints), and
// re-meshes body_shape(i). Only poses change -- the source file
// is never written. Requires the asmsolver (Ceres) build; without
// it every call reports last_error() and does nothing.
// =========================================================

foreign class AsmSession
{
    init() {}

    // load(path, unit_scale, strict) -> bool (false on failure;
    // see last_error()). unit_scale: 0.001 = FreeCAD mm -> metres.
    foreign load(path, unit_scale, strict)
    foreign last_error()

    // ---- bodies (valid after a successful load) ----
    foreign body_count()
    foreign body_name(i)
    foreign body_shape(i)        // TopoShape at the current solved pose

    // ---- interaction ----
    //   pick(ox,oy,oz, dx,dy,dz) -> body index (-1 on miss)
    //   hit()                    -> [x,y,z] world hit point of last pick
    //   drag(body, tx,ty,tz, w)  -> post-solve joint residual
    foreign pick(ox, oy, oz, dx, dy, dz)
    foreign hit()
    foreign drag(body, tx, ty, tz, w)
    foreign drag_rot(body, ax, ay, az, angle, w)  // rotate about grab point
    foreign drive(body, ax, ay, az, value, w)     // drive a DOF along an axis
    foreign snap()               // release-snap: handle-free re-solve -> residual

    // ---- write-back ----
    //   save(out_path) -> bool  : write solved poses to a copy at out_path
    //   save_back()     -> bool : overwrite the source in place (+ .bak backup)
    foreign save(out_path)
    foreign save_back()
}

);
