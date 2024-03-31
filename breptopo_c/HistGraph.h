#pragma once

#include <TopoDS_Shape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include <memory>
#include <vector>
#include <map>

namespace partgraph { class TopoShape; class TopoFace; class BRepHistory; }
namespace graph { class Graph; }

namespace breptopo
{

class HistGraph
{
public:
	HistGraph();

	void Update(const std::shared_ptr<partgraph::TopoFace>& from, 
		const std::vector<std::shared_ptr<partgraph::TopoFace>>& to);

	void Update(const partgraph::BRepHistory& hist);

	auto GetGraph() { return m_graph; }

private:
	std::shared_ptr<graph::Graph> m_graph;

	int m_time = 0;

	TopTools_IndexedMapOfShape m_curr_faces;
	std::map<int, int> m_idx2gid;

}; // HistGraph

}