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

void TopoNaming::Update(const partgraph::TopoShape& new_shape, uint32_t op_id)
{
	m_vertex_hg->Update(partgraph::BRepHistory(new_shape), TopAbs_VERTEX, op_id);
	m_edge_hg->Update(partgraph::BRepHistory(new_shape), TopAbs_EDGE, op_id);
	m_face_hg->Update(partgraph::BRepHistory(new_shape), TopAbs_FACE, op_id);
	m_solid_hg->Update(partgraph::BRepHistory(new_shape), TopAbs_SOLID, op_id);
}

void TopoNaming::Update(BRepBuilderAPI_MakeShape& builder, const partgraph::TopoShape& new_shape, 
	                    const partgraph::TopoShape& old_shape, uint32_t op_id)
{
	m_vertex_hg->Update(partgraph::BRepHistory(builder, TopAbs_VERTEX, new_shape, old_shape), TopAbs_VERTEX, op_id);
	m_edge_hg->Update(partgraph::BRepHistory(builder, TopAbs_EDGE, new_shape, old_shape), TopAbs_EDGE, op_id);
	m_face_hg->Update(partgraph::BRepHistory(builder, TopAbs_FACE, new_shape, old_shape), TopAbs_FACE, op_id);
	m_solid_hg->Update(partgraph::BRepHistory(builder, TopAbs_SOLID, new_shape, old_shape), TopAbs_SOLID, op_id);
}

void TopoNaming::Update(opencascade::handle<BRepTools_History> hist, const partgraph::TopoShape& new_shape,
	                    const partgraph::TopoShape& old_shape, uint32_t op_id)
{
	m_vertex_hg->Update(partgraph::BRepHistory(hist, TopAbs_VERTEX, new_shape, old_shape), TopAbs_VERTEX, op_id);
	m_edge_hg->Update(partgraph::BRepHistory(hist, TopAbs_EDGE, new_shape, old_shape), TopAbs_EDGE, op_id);
	m_face_hg->Update(partgraph::BRepHistory(hist, TopAbs_FACE, new_shape, old_shape), TopAbs_FACE, op_id);
	m_solid_hg->Update(partgraph::BRepHistory(hist, TopAbs_SOLID, new_shape, old_shape), TopAbs_SOLID, op_id);
}

void TopoNaming::Update(const BRepOffset_MakeSimpleOffset& builder, const partgraph::TopoShape& old_shape, uint32_t op_id)
{
	m_vertex_hg->Update(partgraph::BRepHistory(builder, old_shape), TopAbs_VERTEX, op_id);
	m_edge_hg->Update(partgraph::BRepHistory(builder, old_shape), TopAbs_EDGE, op_id);
	m_face_hg->Update(partgraph::BRepHistory(builder, old_shape), TopAbs_FACE, op_id);
	m_solid_hg->Update(partgraph::BRepHistory(builder, old_shape), TopAbs_SOLID, op_id);
}

uint32_t TopoNaming::NextOpId()
{
	return m_next_op++;
}

}