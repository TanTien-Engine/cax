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
}

);
