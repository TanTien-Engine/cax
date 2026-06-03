#define QUOTE(...) #__VA_ARGS__
static const char* cadcvtModuleSource = QUOTE(

// =========================================================
// Vessel foreign classes for cadcvt.
//
// FreeCadLoader and SwLoader expose the same load surface; SelfLoader
// will follow the same pattern.
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
    //   sketch_plane_at(step)      -> [ox,oy,oz, xx,xy,xz, nx,ny,nz] or nil
    //   sketch_geo_count_at(step)  -> number
    //   sketch_geo_at(step, g)     -> [type, params...] (SkGeoType code +
    //                                 per-type params) or nil
    //   sketch_cons_count_at(step) -> number
    //   sketch_cons_at(step, i)    -> [cons_type, a_geo, a_pos, b_geo, b_pos,
    //                                 value, driving] or nil. a_geo / b_geo are
    //                                 indices into the NON-construction geometry
    //                                 (same order as sketch_geo_at), or -1 when
    //                                 the ref is an axis / origin / construction
    //                                 geo; a_pos / b_pos are SkPointPos (0 none,
    //                                 1 start, 2 mid, 3 end); driving is 1 / 0.
    foreign sketch_plane_at(step)
    foreign sketch_geo_count_at(step)
    foreign sketch_geo_at(step, g)
    foreign sketch_cons_count_at(step)
    foreign sketch_cons_at(step, i)

    // Build a B-rep face from a flat list of 2D geometry on a plane, reusing
    // the robust sketch_face machinery (solver + ConnectEdgesToWires + hole
    // fill). Stateless. A reconstructed editable sketch sub-graph feeds its
    // (edited) geometry through here so the face survives holes / multiple
    // loops / arbitrary edge order.
    //   geos   : flat [type, nparams, p..., type, nparams, p..., ...]
    //   origin / x_dir / normal : the sketch plane, each [x, y, z]
    //   -> TopoShape (face/compound) or nil
    foreign face_from_geos(geos, origin, x_dir, normal)
}

// =========================================================
// SwLoader -- SolidWorks .SLDPRT / .SLDASM, via COM automation of
// the installed SolidWorks. Same load surface as FreeCadLoader (the
// replay path is reader-agnostic). unit_scale defaults to 1.0 because
// the SolidWorks API already reports SI metres. The first load()
// launches SLDWORKS.exe (slow); later loads reuse the instance.
// =========================================================

foreign class SwLoader
{
    init() {}

    // ---- options ----
    foreign set_unit_scale(s)
    foreign set_strict(b)

    // ---- one-shot read + replay ----
    // Returns a TopoShape (or nil on failure; see last_error()).
    foreign load(path)
    foreign last_error()

    // ---- per-part access (valid after load) ----
    foreign part_count()
    foreign part_shape(i)
    foreign part_transparency(i)

    // Merged opaque / transparent subsets for two-pass see-through
    // rendering. nil when the subset is empty.
    foreign merged_opaque()
    foreign merged_transparent()

    // Transparent parts un-merged + parallel per-part alpha list.
    foreign transparent_parts()
    foreign transparent_alphas()

    // ---- construction graph + sketch exposure (history-tree rebuild) ----
    // Same contract as FreeCadLoader: calc_graph() returns the replay
    // CalcGraph (nil for the multi-part parallel path); the sketch_*_at
    // family reads raw 2D geometry / constraints off a $sketch const so
    // HistGraphBuilder can decompile the import into editable nodes.
    foreign calc_graph()
    foreign sketch_plane_at(step)
    foreign sketch_geo_count_at(step)
    foreign sketch_geo_at(step, g)
    foreign sketch_cons_count_at(step)
    foreign sketch_cons_at(step, i)

    // Stateless face builder (a rebuilt sketch sub-graph feeds its edited
    // 2D geometry through here); same contract as FreeCadLoader.
    foreign face_from_geos(geos, origin, x_dir, normal)
}

// =========================================================
// ZwLoader -- ZW3D neutral intermediate (.cax.json) -> shape.
//
// The .cax.json is produced by the zw_export plugin running inside
// ZW3D; this loader is SDK-free (parses JSON + replays via the
// reader-agnostic Replayer), so no ZW3D runtime is needed. Same load
// surface as SwLoader, minus the SW-only COM / transparency extras.
// =========================================================

foreign class ZwLoader
{
    init() {}

    // ---- options ----
    foreign set_unit_scale(s)   // length multiplier; default 0.001 (mm -> m)
    foreign set_strict(b)       // fail on unknown feature kinds

    // ---- one-shot read + replay ----
    // Returns a TopoShape (or nil on failure; see last_error()).
    foreign load(path)
    foreign last_error()

    // ---- per-part access (valid after load) ----
    foreign part_count()
    foreign part_shape(i)

    // ---- construction graph + sketch exposure (history-tree rebuild) ----
    // Same contract as SwLoader / FreeCadLoader: RebuildHistory duck-types
    // on get_calc_graph() (below) + reads raw 2D geometry / constraints off
    // a $sketch const so HistGraphBuilder can decompile the import.
    foreign calc_graph()
    foreign sketch_plane_at(step)
    foreign sketch_geo_count_at(step)
    foreign sketch_geo_at(step, g)
    foreign sketch_cons_count_at(step)
    foreign sketch_cons_at(step, i)

    // Stateless face builder (a rebuilt sketch sub-graph feeds its edited
    // 2D geometry through here); same contract as SwLoader.
    foreign face_from_geos(geos, origin, x_dir, normal)
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
