#pragma once

#include "GraphData.h"

#include <memory>
#include <vector>

namespace brepkit { class TopoShape; }

namespace deepbrep
{

// Walks an OCCT TopoDS_Shape (wrapped in brepkit::TopoShape) and produces a
// GraphData ready to feed into GNNModel. One node per TopoDS_Face. One
// undirected edge (stored as two directed) per pair of faces that share a
// TopoDS_Edge.
//
// Node and edge feature layouts follow FeatureLabels.h (kNodeFeatDim /
// kEdgeFeatDim). Labels are left empty; populate them externally for
// training.
class BRepGraphBuilder
{
public:
    static GraphData Build(const std::vector<std::shared_ptr<brepkit::TopoShape>>& shapes);

    // Single-shape convenience.
    static GraphData Build(const std::shared_ptr<brepkit::TopoShape>& shape);

private:
    // Internal: append one shape's faces and edges into `g`, offsetting node
    // indices by the current node count.
    static void AppendShape(const std::shared_ptr<brepkit::TopoShape>& shape,
                            GraphData& g,
                            std::vector<std::pair<int, int>>& from_to,
                            std::vector<std::vector<float>>& edge_rows);
};

}
