#pragma once

namespace breptopo { class CompGraph; class TopoNaming; }

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

    void StoreCompGraph(const breptopo::CompGraph& cg);
    bool LoadCompGraph(breptopo::CompGraph& cg);

    void StoreVersionTree(const VersionTree& vt);
    bool LoadVersionTree(VersionTree& vt);

    void StoreTopoNaming(const breptopo::TopoNaming& tn);
    bool LoadTopoNaming(breptopo::TopoNaming& tn);

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
