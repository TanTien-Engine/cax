#include "TopoNaming.h"
#include "HistGraph.h"

#include <partgraph_c/BRepHistory.h>

namespace breptopo
{

TopoNaming::TopoNaming()
{
	m_vertex_hg = std::make_shared<HistGraph>();
	m_edge_hg = std::make_shared<HistGraph>();
	m_face_hg = std::make_shared<HistGraph>();
	m_solid_hg = std::make_shared<HistGraph>();
}

void TopoNaming::Update(const partgraph::TopoShape& new_shape, uint16_t op_id)
{
	m_vertex_hg->Update(partgraph::BRepHistory(new_shape), op_id);
	m_edge_hg->Update(partgraph::BRepHistory(new_shape), op_id);
	m_face_hg->Update(partgraph::BRepHistory(new_shape), op_id);
	m_solid_hg->Update(partgraph::BRepHistory(new_shape), op_id);
}

void TopoNaming::Update(BRepBuilderAPI_MakeShape& builder, const partgraph::TopoShape& new_shape, 
	                    const partgraph::TopoShape& old_shape, uint16_t op_id)
{
	m_vertex_hg->Update(partgraph::BRepHistory(builder, TopAbs_VERTEX, new_shape, old_shape), op_id);
	m_edge_hg->Update(partgraph::BRepHistory(builder, TopAbs_EDGE, new_shape, old_shape), op_id);
	m_face_hg->Update(partgraph::BRepHistory(builder, TopAbs_FACE, new_shape, old_shape), op_id);
	m_solid_hg->Update(partgraph::BRepHistory(builder, TopAbs_SOLID, new_shape, old_shape), op_id);
}

void TopoNaming::Update(opencascade::handle<BRepTools_History> hist, const partgraph::TopoShape& new_shape,
	                    const partgraph::TopoShape& old_shape, uint16_t op_id)
{
	m_vertex_hg->Update(partgraph::BRepHistory(hist, TopAbs_VERTEX, new_shape, old_shape), op_id);
	m_edge_hg->Update(partgraph::BRepHistory(hist, TopAbs_EDGE, new_shape, old_shape), op_id);
	m_face_hg->Update(partgraph::BRepHistory(hist, TopAbs_FACE, new_shape, old_shape), op_id);
	m_solid_hg->Update(partgraph::BRepHistory(hist, TopAbs_SOLID, new_shape, old_shape), op_id);
}

uint16_t TopoNaming::NextOpId()
{
	return m_next_op++;
}

}