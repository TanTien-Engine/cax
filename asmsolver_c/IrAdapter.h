#pragma once

// Adapter: build an asmsolver::Assembly from a cadapp::DocumentIR.
//
// Reads the FreeCAD-reader output -- placed PartDesign::Body parts (their
// world placement stashed as `asm_*` ext_params on each body's tip
// feature, tagged with `freecad_body`) and FeatType::Joint features
// (joint_kind, connector poses joint_p1/p2_*, grounding joint_ground_*,
// part refs joint_ref{1,2}_part, joint_distance) -- into the solver's
// body+joint graph.
//
// Pulls only cadapp IR *headers* (FeatureIR has no OCCT in its public
// surface -- brepkit::TopoShape is forward-declared), so this stays free
// of the OCCT / OGDF link chain.

#include "asmsolver_c/AsmSolver.h"

#include <map>
#include <string>
#include <vector>

namespace cadapp { struct DocumentIR; }

namespace asmsolver {

struct ImportResult {
    Assembly                  assembly;
    std::vector<std::string>  body_names;     // index -> FreeCAD body name
    std::map<std::string,int> body_index;     // name -> index
    int  joints_built      = 0;
    int  joints_skipped    = 0;               // unknown kind / unresolved part
    int  cylinder_distance = 0;               // Distance joints needing a BREP
                                              // radius (left at radius=0 here;
                                              // filled by a later increment)
};

// Build the assembly. Body initial poses come from each body's imported
// world placement (the FreeCAD-solved configuration); Solve() from that
// state should leave every (fully modelled) joint residual ~0.
ImportResult BuildAssembly(const cadapp::DocumentIR& doc);

} // namespace asmsolver
