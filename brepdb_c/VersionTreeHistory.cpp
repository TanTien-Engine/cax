#include "VersionTree.h"

#include "partgraph_c/BRepHistory.h"
#include "GeomSender.h"

// This file requires OCCT headers and is compiled only in the full project.
// It converts BRepHistory's shape-level map into a PidMapping and delegates
// to BuildDiffFromPidMapping.

namespace brepdb
{

PoolDiff VersionTree::BuildDiffFromHistory(const GeometryPool&          old_pool,
                                            const GeometryPool&          new_pool,
                                            const partgraph::BRepHistory& hist,
                                            const GeomSender&            sender)
{
    PidMapping pid_map;

    const auto& shape_map = hist.GetIdxMap();
    const auto& old_map   = hist.GetOldMap();
    const auto& new_map   = hist.GetNewMap();

    // BRepHistory uses 1-based OCCT IndexedMap convention internally;
    // GetIdxMap() returns 0-based keys into those maps.
    for (const auto& [old_shape_idx, new_shape_indices] : shape_map)
    {
        const TopoDS_Shape& old_shape = old_map(old_shape_idx + 1);
        uint32_t old_pid = sender.GetUID(old_shape);

        if (new_shape_indices.empty())
        {
            // Shape deleted
            pid_map[old_pid] = {};
            continue;
        }

        std::vector<uint32_t> new_pids;
        new_pids.reserve(new_shape_indices.size());

        for (int new_shape_idx : new_shape_indices)
        {
            const TopoDS_Shape& new_shape = new_map(new_shape_idx + 1);
            new_pids.push_back(sender.GetUID(new_shape));
        }

        pid_map[old_pid] = std::move(new_pids);
    }

    return BuildDiffFromPidMapping(old_pool, new_pool, pid_map);
}

} // namespace brepdb
