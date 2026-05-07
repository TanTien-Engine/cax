#pragma once

#include "brepdb_c/ShapeIndex.h"
#include "brepdb_c/TopoGraph.h"

#include <spatialdb/RTree.h>

namespace brepdb
{

struct GeomHeader;
struct GeometryPool;

class BrepDB
{
public:
    BrepDB(const std::shared_ptr<spatialdb::IStorageManager>& sm, bool overwrite);
    ~BrepDB();

    void Insert(const GeomHeader& header, const double* params);
    void ImportPool(const GeometryPool& pool);

    spatialdb::RTree& GetRTree() { return *m_rtree; }
    ShapeIndex& GetShapeIndex() { return *m_shape_index; }
    TopoGraph& GetTopoGraph() { return m_topo_graph; }
    const TopoGraph& GetTopoGraph() const { return m_topo_graph; }

    void Flush();

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