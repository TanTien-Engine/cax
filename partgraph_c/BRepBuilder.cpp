#include "BRepBuilder.h"
#include "TopoDataset.h"
#include "occt_adapter.h"

#include "GeomDataset.h"

#include <geoshape/Arc3D.h>
#include <geoshape/Line3D.h>

// OCCT
#include <TopoDS_Edge.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>

namespace partgraph
{

std::shared_ptr<TopoEdge> BRepBuilder::MakeEdge(const gs::Line3D& l)
{
	gp_Pnt p1 = trans_pnt(l.GetStart());
	gp_Pnt p2 = trans_pnt(l.GetEnd());
	TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(p1, p2);
	return std::make_shared<TopoEdge>(edge);
}

std::shared_ptr<TopoEdge> BRepBuilder::MakeEdge(const gs::Arc3D& arc)
{
	sm::vec3 p1(arc.GetStart());
	sm::vec3 p2(arc.GetMiddle());
	sm::vec3 p3(arc.GetEnd());

	Handle(Geom_TrimmedCurve) curve = GC_MakeArcOfCircle(
		trans_pnt(p1), trans_pnt(p2), trans_pnt(p3)
	);

	Geom_TrimmedCurve* c2 = curve.get();

	TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(c2);
	return std::make_shared<TopoEdge>(edge);
}

std::shared_ptr<TopoEdge> BRepBuilder::MakeEdge(const TrimmedCurve& c, const CylindricalSurface& s)
{
	TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(c.GetShape(), s.GetShape());
	return std::make_shared<TopoEdge>(edge);
}

std::shared_ptr<TopoWire> BRepBuilder::MakeWire(const std::vector<std::shared_ptr<TopoEdge>>& edges)
{
	TopoDS_Wire wire;
	if (edges.size() == 1) {
		wire = BRepBuilderAPI_MakeWire(edges[0]->GetEdge());
	} else if (edges.size() == 2) {
		wire = BRepBuilderAPI_MakeWire(edges[0]->GetEdge(), edges[1]->GetEdge());
	} else if (edges.size() == 3) {
		wire = BRepBuilderAPI_MakeWire(edges[0]->GetEdge(), edges[1]->GetEdge(), edges[2]->GetEdge());
	} else if (edges.size() == 4) {
		wire = BRepBuilderAPI_MakeWire(edges[0]->GetEdge(), edges[1]->GetEdge(), edges[2]->GetEdge(), edges[3]->GetEdge());
	} else {
		return nullptr;
	}
	return std::make_shared<TopoWire>(wire);
}

std::shared_ptr<TopoFace> BRepBuilder::MakeFace(const TopoWire& wire)
{
	TopoDS_Face face = BRepBuilderAPI_MakeFace(wire.GetWire());
	return std::make_shared<TopoFace>(face);
}

std::shared_ptr<TopoShell> BRepBuilder::MakeShell(const std::vector<std::shared_ptr<TopoFace>>& faces)
{
	TopoDS_Shell shell;
	BRep_Builder builder;
	builder.MakeShell(shell);
	for (auto& face : faces) {
	    builder.Add(shell, face->GetFace());
	}
	return std::make_shared<TopoShell>(shell);
}

std::shared_ptr<TopoShape> BRepBuilder::MakeCompound(const std::vector<std::shared_ptr<TopoShape>>& shapes)
{
	TopoDS_Compound res;
	BRep_Builder builder;
	builder.MakeCompound(res);
	for (auto& shape : shapes) {
		builder.Add(res, shape->GetShape());
	}
	return std::make_shared<TopoShape>(res);
}

}