#include "TopoAdapter.h"
#include "TopoShape.h"

#include "modules/render/Render.h"

#include <SM_Vector.h>
#include <unirender/Device.h>
#include <unirender/VertexBuffer.h>
#include <unirender/IndexBuffer.h>
#include <unirender/VertexArray.h>
#include <unirender/VertexInputAttribute.h>
#include <geoshape/Line3D.h>
#include <geoshape/Polyline3D.h>

// OCCT
#include <Standard_Handle.hxx>
#include <Poly_Triangulation.hxx>
#include <TopoDS_Face.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include <set>

namespace
{

struct Vertex
{
	sm::vec3 pos;
	sm::vec3 normal;
	//sm::vec2 texcoord;
};

// from FreeCAD part/app/tools
Handle(Poly_Triangulation) TriangulationOfFace(const TopoDS_Face& face)
{
    TopLoc_Location loc;
    Handle (Poly_Triangulation) mesh = BRep_Tool::Triangulation(face, loc);
    if (!mesh.IsNull())
        return mesh;

    // If no triangulation exists then the shape is probably infinite
    BRepAdaptor_Surface adapt(face);
    double u1 = adapt.FirstUParameter();
    double u2 = adapt.LastUParameter();
    double v1 = adapt.FirstVParameter();
    double v2 = adapt.LastVParameter();

    auto selectRange = [](double& p1, double& p2) {
        if (Precision::IsInfinite(p1) && Precision::IsInfinite(p2)) {
            p1 = -50.0;
            p2 =  50.0;
        }
        else if (Precision::IsInfinite(p1)) {
            p1 = p2 - 100.0;
        }
        else if (Precision::IsInfinite(p2)) {
            p2 = p1 + 100.0;
        }
    };

    // recreate a face with a clear boundary in case it's infinite
    selectRange(u1, u2);
    selectRange(v1, v2);

    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    BRepBuilderAPI_MakeFace mkBuilder(surface, u1, u2, v1, v2, Precision::Confusion() );

    TopoDS_Shape shape = mkBuilder.Shape();
    shape.Location(loc);

    BRepMesh_IncrementalMesh(shape, 0.1);
    return BRep_Tool::Triangulation(TopoDS::Face(shape), loc);
}

}

namespace partgraph
{

std::shared_ptr<ur::VertexArray> TopoAdapter::BuildMeshFromShape(const TopoShape& shape)
{
    return BuildMesh(shape.GetShape());
}

std::shared_ptr<ur::VertexArray> TopoAdapter::BuildMeshFromShell(const TopoShape& shell)
{
    return BuildMesh(shell.GetShape());
}

std::shared_ptr<gs::Line3D> TopoAdapter::BuildGeoFromEdge(const TopoShape& shape)
{
    std::vector<sm::vec3> pts;

    auto& edge = shape.GetShape();
    TopoDS_Iterator it;
    int itr = 0;
    for (it.Initialize(edge); it.More(); it.Next(), ++itr)
    {
        auto& v = TopoDS::Vertex(it.Value());
        gp_Pnt p = BRep_Tool::Pnt(v);
        pts.push_back(sm::vec3(
            static_cast<float>(p.X()), 
            static_cast<float>(p.Y()),
            static_cast<float>(p.Z()))
        );
    }

    if (pts.size() < 2) {
        return nullptr;
    }

    auto geo = std::make_shared<gs::Line3D>();
    geo->SetStart(pts[0]);
    geo->SetEnd(pts[1]);

    return geo;
}

std::shared_ptr<gs::Polyline3D> TopoAdapter::BuildGeoFromWire(const TopoShape& wire)
{
    std::vector<sm::vec3> pts;

    auto& w = wire.GetShape();
    TopoDS_Iterator it;
    int itr = 0;
    for (it.Initialize(w); it.More(); it.Next(), ++itr)
    {
        const TopoDS_Edge& e = TopoDS::Edge(it.Value());

        int itr2 = 0;
        TopoDS_Iterator it2;
        for (it2.Initialize(e); it2.More(); it2.Next(), ++itr2)
        {
            auto& v = TopoDS::Vertex(it2.Value());
            gp_Pnt p = BRep_Tool::Pnt(v);
            pts.push_back(sm::vec3(
                static_cast<float>(p.X()),
                static_cast<float>(p.Y()),
                static_cast<float>(p.Z()))
            );
        }
    }

    auto geo = std::make_shared<gs::Polyline3D>();
    geo->SetVertices(pts);

    return geo;
}

std::shared_ptr<TopoShape> TopoAdapter::ToWire(const TopoShape& shape)
{
    return std::make_shared<TopoShape>(TopoDS::Wire(shape.GetShape()));
}

std::shared_ptr<ur::VertexArray> TopoAdapter::BuildMesh(const TopoDS_Shape& shape)
{
    auto algo = std::make_unique<BRepMesh_IncrementalMesh>();
    algo->SetShape(shape);
    algo->Perform();

    TopLoc_Location aLoc;
//    shape.Location(aLoc);

    std::vector<Vertex> vertices;

    TopTools_IndexedMapOfShape face_map; 
    TopExp::MapShapes(shape, TopAbs_FACE, face_map);
    for (int i=1; i <= face_map.Extent(); i++) 
    {
        const TopoDS_Face& act_face = TopoDS::Face(face_map(i));
        Handle (Poly_Triangulation) mesh = BRep_Tool::Triangulation(act_face, aLoc);
        if (mesh.IsNull()) {
            mesh = TriangulationOfFace(act_face);
        }
        if (mesh.IsNull()) {
            continue;
        }

        int nb_tri_in_face = mesh->NbTriangles();
        TopAbs_Orientation orient = act_face.Orientation();
        for (int j = 1; j <= nb_tri_in_face; ++j)
        {
            Standard_Integer N1, N2, N3;
            mesh->Triangle(j).Get(N1, N2, N3);

            // change orientation of the triangle if the face is reversed
            if (orient != TopAbs_FORWARD) {
                Standard_Integer tmp = N1;
                N1 = N2;
                N2 = tmp;
            }

            gp_Pnt V1(mesh->Node(N1)), V2(mesh->Node(N2)), V3(mesh->Node(N3));

            gp_Vec v1(V1.X(),V1.Y(),V1.Z()),
                   v2(V2.X(),V2.Y(),V2.Z()),
                   v3(V3.X(),V3.Y(),V3.Z());
            gp_Vec normal = (v2-v1)^(v3-v1);
            //gp_Vec NV1 = normal;
            //gp_Vec NV2 = normal;
            //gp_Vec NV3 = normal;

            sm::vec3 norm(sm::vec3(
                static_cast<float>(normal.X()),
                static_cast<float>(normal.Y()),
                static_cast<float>(normal.Z())
            ));
            vertices.push_back(Vertex({ sm::vec3(
                static_cast<float>(V1.X()),
                static_cast<float>(V1.Y()),
                static_cast<float>(V1.Z())
            ), norm }));
            vertices.push_back(Vertex({ sm::vec3(
                static_cast<float>(V2.X()),
                static_cast<float>(V2.Y()),
                static_cast<float>(V2.Z())
            ), norm }));
            vertices.push_back(Vertex({ sm::vec3(
                static_cast<float>(V3.X()),
                static_cast<float>(V3.Y()),
                static_cast<float>(V3.Z())
            ), norm }));
        }
    }

    if (vertices.empty()) {
        return nullptr;
    }

    auto dev = tt::Render::Instance()->Device();

    auto va = dev->CreateVertexArray();

	int vbuf_sz = static_cast<int>(sizeof(Vertex) * vertices.size());
	auto vbuf = dev->CreateVertexBuffer(ur::BufferUsageHint::StaticDraw, vbuf_sz);
	vbuf->ReadFromMemory(vertices.data(), vbuf_sz, 0);
	va->SetVertexBuffer(vbuf);

	std::vector<std::shared_ptr<ur::VertexInputAttribute>> vbuf_attrs;
    // pos
	vbuf_attrs.push_back(std::make_shared<ur::VertexInputAttribute>(
        0, ur::ComponentDataType::Float, 3, 0, 24
    ));
    // normal
	vbuf_attrs.push_back(std::make_shared<ur::VertexInputAttribute>(
        1, ur::ComponentDataType::Float, 3, 12, 24
    ));
	va->SetVertexBufferAttrs(vbuf_attrs);

    return va;
}

}