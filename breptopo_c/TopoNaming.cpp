#include "TopoNaming.h"
#include "HistGraph.h"

namespace breptopo
{

TopoNaming::TopoNaming()
{
	//m_edge_hg = std::make_shared<HistGraph>(HistGraph::Type::Edge);
	m_face_hg = std::make_shared<HistGraph>(HistGraph::Type::Face);
	//m_solid_hg = std::make_shared<HistGraph>(HistGraph::Type::Solid);
}

}