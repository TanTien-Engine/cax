#pragma once

namespace brepgraph
{
class CompGraph;
class TopoNaming;
}
namespace cadapp
{
class SketchStore;
class FeatureStore;
}

#include "brepdb_c/ShapeIndex.h"
#include "brepdb_c/TopoGraph.h"
#include "brepdb_c/TypedPool.h"
#include "brepdb_c/VersionTree.h"

#include <spatialdb/RTree.h>

namespace brepdb
{

class BrepDB
{
public:
    BrepDB(const std::shared_ptr<spatialdb::IStorageManager>& sm, bool overwrite);
    ~BrepDB();

    void Insert(uint32_t entity_id, const BRepWorld& world);
    void ImportWorld(const BRepWorld& world);

    spatialdb::RTree& GetRTree() { return *m_rtree; }
    ShapeIndex& GetShapeIndex() { return *m_shape_index; }
    TopoGraph& GetTopoGraph() { return m_topo_graph; }
    const TopoGraph& GetTopoGraph() const { return m_topo_graph; }

    void Flush();

    void StoreCompGraph(const brepgraph::CompGraph& cg);
    bool LoadCompGraph(brepgraph::CompGraph& cg);

    void StoreVersionTree(const VersionTree& vt);
    bool LoadVersionTree(VersionTree& vt);

    void StoreTopoNaming(const brepgraph::TopoNaming& tn);
    bool LoadTopoNaming(brepgraph::TopoNaming& tn);

    // ---- cadapp meta pages: IR layer for SW / FreeCAD imports ----
    void StoreSketchStore(const cadapp::SketchStore& ss);
    bool LoadSketchStore(cadapp::SketchStore& ss);
    void StoreFeatureStore(const cadapp::FeatureStore& fs);
    bool LoadFeatureStore(cadapp::FeatureStore& fs);

private:
    void StoreShapeIndex();
    void LoadShapeIndex();
    void StoreTopoGraph();
    void LoadTopoGraph();

private:
    std::shared_ptr<spatialdb::IStorageManager> m_sm;
    std::unique_ptr<spatialdb::RTree> m_rtree;
    std::unique_ptr<ShapeIndex> m_shape_index;
    TopoGraph m_topo_graph;

}; // BrepDB

}
