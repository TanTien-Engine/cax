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
}

);
