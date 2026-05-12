#pragma once

#include <memory>
#include <string>

namespace breptopo { class TopoNaming; }

namespace brepdb
{

class BRepWorld;

// STEP format import/export for BRepWorld.
// Export reconstructs TopoDS_Shape via WorldReceiver, then writes STEP.
// Import reads STEP into TopoDS_Shape, then serializes via WorldSender.
class StepFile
{
public:
    static bool Export(const std::string& filename, const BRepWorld& world);

    static bool Import(const std::string& filename, BRepWorld& world,
                       const std::shared_ptr<breptopo::TopoNaming>& tn);

}; // StepFile

} // namespace brepdb
