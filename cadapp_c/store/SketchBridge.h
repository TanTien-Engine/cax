#pragma once

#include "cadapp_c/ir/SketchIR.h"
#include "cadapp_c/store/SketchStore.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

// ============================================================
// cadapp/store/SketchBridge.h
//
// SketchStore / SketchIR <-> sketchlib::Scene two-way bridge.
//
//   ImportToScene   : SketchStore (or SketchIR) -> sketchlib::Scene
//                     plus a (geo_id, gs::Shape2D) list needed by
//                     Scene::Solve.
//
//   ExportFromScene : after solving, write back the new 2D coords
//                     to the SketchStore.
//
//   EmitVes         : SketchStore -> sketchgraph .ves script for
//                     the sketcheditor. Visual layer only, not on
//                     the import critical path.
//
// The .cpp pulls in sketchlib + geoshape; this header stays light.
// ============================================================

namespace gs
{
class Shape2D;
}
namespace sketchlib
{
class Scene;
}

namespace cadapp
{

class SketchBridge
{
public:
    using GeoShape  = std::shared_ptr<gs::Shape2D>;
    using GeoShapes = std::vector<std::pair<int /*GeoID*/, GeoShape>>;

    // ---- SketchStore -> Scene ----
    // Import the sketch at sketch_idx into scene. The returned
    // out_geos is fed directly to sketchlib::Scene::Solve.
    static bool ImportToScene(const SketchStore& store,
                              uint32_t           sketch_idx,
                              sketchlib::Scene&  out_scene,
                              GeoShapes&         out_geos);

    // Same, but with a raw SketchIR (useful when a reader hasn't
    // yet committed into a store).
    static bool ImportToScene(const SketchIR&   sketch,
                              sketchlib::Scene& out_scene,
                              GeoShapes&        out_geos);

    // ---- Scene -> SketchStore ----
    // After solving, copy the resulting shape coordinates back
    // into the sketch at sketch_idx. Preserves cons / type /
    // construction flags; only param values are updated.
    static bool ExportFromScene(const GeoShapes& solved_geos,
                                SketchStore&     store,
                                uint32_t         sketch_idx);

    // ---- SketchStore -> .ves ----
    // Emit a ves script equivalent to sketchgraph/nodes/{line,
    // arc, circle, ...}. Canvas coords are skipped and laid out
    // mechanically; the editor can re-layout.
    static std::string EmitVes(const SketchStore& store, uint32_t sketch_idx);
    static std::string EmitVes(const SketchIR&    sketch);
};

} // namespace cadapp
