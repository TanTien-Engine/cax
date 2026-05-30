#pragma once

// FreeCadWriter -- write solved body poses back into a FreeCAD .FCStd.
//
// A .FCStd is a zip whose Document.xml holds every object. For native
// Assembly WB each PartDesign::Body carries its assembly pose in its own
// Placement property. This writer rewrites those placements in place: it
// parses Document.xml (pugixml), updates the named bodies' PropertyPlacement
// (position + quaternion + redundant axis-angle), and re-zips (miniz) -- every
// other entry (.brp geometry, GuiDocument.xml, ...) is copied verbatim. The
// write goes to a temp file that is renamed over the target only on success,
// and an optional backup of the source is taken first, so a failed write can
// never corrupt the original.
//
// Exact inverse of the reader: it reads Px/Py/Pz * unit_scale -> metres and
// Q0..Q3 -> quaternion (x,y,z,w); we write Px = px / unit_scale and
// Q0..Q3 = quat. Uses only miniz + pugixml (no OCCT / Ceres).

#include <map>
#include <string>

namespace cadcvt {

// A body's target world placement, in PROJECT units (metres). The writer
// scales translation to FreeCAD mm via 1/unit_scale.
struct BodyPlacement {
    double px = 0, py = 0, pz = 0;        // translation (metres)
    double qx = 0, qy = 0, qz = 0, qw = 1; // orientation quaternion (x,y,z,w)
};

// Rewrite the .FCStd at src_path, replacing each named Body's Placement with
// the given pose, and write the result to out_path (may equal src_path). If
// backup_path is non-empty, src_path is first copied there. unit_scale matches
// the reader (0.001 = FreeCAD mm -> metres). On success returns true and (if
// non-null) *written = number of bodies updated; on failure returns false and
// sets *err.
bool WriteFreeCadPlacements(const std::string& src_path,
                            const std::string& out_path,
                            const std::map<std::string, BodyPlacement>& bodies,
                            double unit_scale,
                            const std::string& backup_path,
                            int* written,
                            std::string* err);

} // namespace cadcvt
