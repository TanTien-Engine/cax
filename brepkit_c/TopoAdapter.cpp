#include "TopoAdapter.h"
#include "TopoShape.h"
#include "MemProbe.h"

#include <unirender/Device.h>
#include <unirender/VertexBuffer.h>
#include <unirender/IndexBuffer.h>
#include <unirender/VertexArray.h>
#include <unirender/VertexInputAttribute.h>
#include <geoshape/Line3D.h>
#include <geoshape/Polyline3D.h>
#include <logger/logger.h>

// OCCT
#include <Standard_Handle.hxx>
#include <Poly_Triangulation.hxx>
#include <TopoDS_Face.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRepLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>

#include <set>
#include <cmath>

namespace
{

// Angular deflection (radians) for display tessellation. On curved surfaces
// this is the BINDING constraint (linear deflection is non-binding once scaled
// to the model), so it sets the circumferential facet count on cylinders/cones.
// 0.1 rad (~5.7°, ~63 facets/circle) over-tessellates; 0.3 rad (~17°, ~21
// facets/circle) ~ OCCT's viewer default and cuts curved-face triangles ~3x,
// shrinking both meshing time and the BRepMesh memory transient.
constexpr double kAngularDeflection = 0.3;

// Linear deflection for display tessellation, scaled to the model's size.
// A fixed absolute value (formerly 0.01) over-tessellates large mm-scale parts
// — tessellation is the dominant cost when (re)meshing — while a size-relative
// deflection adapts automatically. The 1e-3 coefficient matches OCCT's own
// viewer default (Prs3d DeviationCoefficient = bbox-diagonal * 0.001). The 0.01
// floor keeps the mesh no finer than the previous behaviour, so small parts are
// unaffected; only large parts get coarser (and faster / lighter on memory).
double DisplayDeflection(const TopoDS_Shape& shape)
{
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    if (box.IsVoid())
        return 0.01;
    double defl = std::sqrt(box.SquareExtent()) * 1.0e-3;
    return defl > 0.01 ? defl : 0.01;
}

// from FreeCAD part/app/tools

Handle(Poly_Triangulation) TriangulationOfFace(const TopoDS_Face& face)
{
    TopLoc_Location loc;
    Handle (Poly_Triangulation) mesh = BRep_Tool::Triangulation(face, loc);
    if (!mesh.IsNull())
        return mesh;

    // Try meshing the original face (preserves wire trimming)
    BRepMesh_IncrementalMesh(face, DisplayDeflection(face), Standard_False, kAngularDeflection);
    mesh = BRep_Tool::Triangulation(face, loc);
    if (!mesh.IsNull())
        return mesh;

    // Fallback for infinite or degenerate surfaces
    BRepAdaptor_Surface adapt(face);
    double u1 = adapt.FirstUParameter();
    double u2 = adapt.LastUParameter();
    double v1 = adapt.FirstVParameter();
    double v2 = adapt.LastVParameter();

    bool hasInfinite = Precision::IsInfinite(u1) || Precision::IsInfinite(u2) ||
                       Precision::IsInfinite(v1) || Precision::IsInfinite(v2);
    if (!hasInfinite)
        return mesh;

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

    selectRange(u1, u2);
    selectRange(v1, v2);

    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    BRepBuilderAPI_MakeFace mkBuilder(surface, u1, u2, v1, v2, Precision::Confusion() );

    TopoDS_Shape shape = mkBuilder.Shape();
    shape.Location(loc);

    BRepMesh_IncrementalMesh(shape, DisplayDeflection(shape), Standard_False, kAngularDeflection);
    return BRep_Tool::Triangulation(TopoDS::Face(shape), loc);
}

Handle(Poly_Polygon3D) PolygonOfEdge(const TopoDS_Edge& edge, TopLoc_Location& loc)
{
    // Degenerate edges (e.g. sphere poles, sketch helper edges) carry no 3D
    // curve and would crash BRepBuilderAPI_MakeEdge below.
    if (BRep_Tool::Degenerated(edge))
        return Handle(Poly_Polygon3D)();

    BRepAdaptor_Curve adapt(edge);
    double u = adapt.FirstParameter();
    double v = adapt.LastParameter();
    Handle(Poly_Polygon3D) aPoly = BRep_Tool::Polygon3D(edge, loc);
    if (!aPoly.IsNull() && !Precision::IsInfinite(u) && !Precision::IsInfinite(v))
        return aPoly;

    // recreate an edge with a clear range
    u = std::max(-50.0, u);
    v = std::min(50.0, v);

    double uv;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, uv, uv);
    if (curve.IsNull())
        return Handle(Poly_Polygon3D)();

    BRepBuilderAPI_MakeEdge mkBuilder(curve, u, v);
    TopoDS_Shape shape = mkBuilder.Shape();
    // why do we have to set the inverted location here?
    TopLoc_Location inv = loc.Inverted();
    shape.Location(inv);

    BRepMesh_IncrementalMesh(shape, 0.1);
    TopLoc_Location tmp;
    return BRep_Tool::Polygon3D(TopoDS::Edge(shape), tmp);
}

}

namespace brepkit
{

std::shared_ptr<ur::VertexArray> TopoAdapter::BuildMeshFromShape(const std::shared_ptr<ur::Device>& dev, const TopoShape& shape, float alpha)
{
    return BuildMesh(dev, shape.GetShape(), alpha);
}

std::shared_ptr<ur::VertexArray> TopoAdapter::BuildMeshFromShell(const std::shared_ptr<ur::Device>& dev, const TopoShape& shell, float alpha)
{
    return BuildMesh(dev, shell.GetShape(), alpha);
}

std::shared_ptr<ur::VertexArray> TopoAdapter::BuildEdgesFromShape(const std::shared_ptr<ur::Device>& dev, const TopoShape& shape, float alpha)
{
    const TopoDS_Shape& src = shape.GetShape();

    // BRepMesh attaches a Poly_Polygon3D to each edge as a side effect
    // of triangulating its faces. Without this call edges of a
    // freshly-loaded shape often have no polygonal representation
    // attached and PolygonOfEdge returns null.
    BRepLib::BuildCurves3d(src);
    // Last arg enables OCCT's per-face parallel meshing (uses its internal
    // OSD_ThreadPool); tessellation is the dominant cost when re-meshing.
    BRepMesh_IncrementalMesh algo(src, DisplayDeflection(src), Standard_False, kAngularDeflection, Standard_True);
    algo.Perform();

    std::vector<Vertex> vertices;

    // TopTools_IndexedMapOfShape uses TopoDS_Shape::IsSame for hashing,
    // so each edge appears once even when shared between two faces.
    TopTools_IndexedMapOfShape edge_map;
    TopExp::MapShapes(src, TopAbs_EDGE, edge_map);

    for (int i = 1; i <= edge_map.Extent(); i++)
    {
        const TopoDS_Edge& edge = TopoDS::Edge(edge_map(i));

        TopLoc_Location loc;
        Handle(Poly_Polygon3D) poly = PolygonOfEdge(edge, loc);
        if (poly.IsNull()) {
            continue;
        }

        bool identity = loc.IsIdentity();
        gp_Trsf trsf;
        if (!identity) {
            trsf = loc.Transformation();
        }

        const TColgp_Array1OfPnt& nodes = poly->Nodes();
        const int nb = poly->NbNodes();
        if (nb < 2) {
            continue;
        }

        // Emit as line segments: (n0,n1), (n1,n2), ... rather than a
        // line strip so a single VAO can hold all edges of the shape.
        sm::vec3 prev;
        bool have_prev = false;
        for (int j = 1; j <= nb; ++j)
        {
            gp_Pnt pnt = nodes(j);
            if (!identity) {
                pnt.Transform(trsf);
            }
            sm::vec3 p(
                static_cast<float>(pnt.X()),
                static_cast<float>(pnt.Y()),
                static_cast<float>(pnt.Z()));

            if (have_prev) {
                // Use a zero normal as a sentinel; the GBuffer shader
                // can branch on length(normal)==0 to skip lighting on
                // edge fragments. alpha is baked per vertex so the
                // transparent edge pass can blend lines at the same
                // opacity as the part's faces.
                vertices.push_back(Vertex({ prev, sm::vec3(0, 0, 0), alpha }));
                vertices.push_back(Vertex({ p,    sm::vec3(0, 0, 0), alpha }));
            }
            prev = p;
            have_prev = true;
        }
    }

    if (vertices.empty()) {
        return nullptr;
    }

    auto va = dev->CreateVertexArray();

    int vbuf_sz = static_cast<int>(sizeof(Vertex) * vertices.size());
    auto vbuf = dev->CreateVertexBuffer(ur::BufferUsageHint::StaticDraw, vbuf_sz);
    vbuf->ReadFromMemory(vertices.data(), vbuf_sz, 0);
    va->SetVertexBuffer(vbuf);

    std::vector<std::shared_ptr<ur::VertexInputAttribute>> vbuf_attrs;
    // Vertex is { vec3 pos; vec3 normal; float alpha; } = 28 bytes, same
    // layout as the surface mesh so the transparent edge shader can read
    // the per-vertex alpha at location 2.
    // pos
    vbuf_attrs.push_back(std::make_shared<ur::VertexInputAttribute>(
        0, ur::ComponentDataType::Float, 3, 0, 28));
    // normal (zeroed - sentinel meaning "edge, no shading")
    vbuf_attrs.push_back(std::make_shared<ur::VertexInputAttribute>(
        1, ur::ComponentDataType::Float, 3, 12, 28));
    // alpha (per-vertex opacity)
    vbuf_attrs.push_back(std::make_shared<ur::VertexInputAttribute>(
        2, ur::ComponentDataType::Float, 1, 24, 28));
    va->SetVertexBufferAttrs(vbuf_attrs);

    return va;
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

std::shared_ptr<ur::VertexArray> TopoAdapter::BuildMesh(const std::shared_ptr<ur::Device>& dev, const TopoDS_Shape& shape, float alpha)
{
    std::vector<Vertex> vertices;

    brepkit::MemProbe("BuildMesh: enter");

    auto type = shape.ShapeType();
    if (type == TopAbs_EDGE)
    {
        TriangulationEdges(shape, vertices);
    }
    else
    {
        // Rebuild missing 3D curves from pcurves (needed for deserialized shapes)
        BRepLib::BuildCurves3d(shape);

        BRepTools::Clean(shape);
        // Last arg enables OCCT's per-face parallel meshing (uses its internal
        // OSD_ThreadPool); tessellation is the dominant cost when re-meshing.
        brepkit::MemProbe("BuildMesh: before BRepMesh");
        BRepMesh_IncrementalMesh algo(shape, DisplayDeflection(shape), Standard_False, kAngularDeflection, Standard_True);
        algo.Perform();
        brepkit::MemProbe("BuildMesh: after BRepMesh");

        TriangulationFaces(shape, vertices, alpha);
    }

    if (vertices.empty()) {
        return nullptr;
    }

    auto va = dev->CreateVertexArray();

	int vbuf_sz = static_cast<int>(sizeof(Vertex) * vertices.size());
	auto vbuf = dev->CreateVertexBuffer(ur::BufferUsageHint::StaticDraw, vbuf_sz);
	vbuf->ReadFromMemory(vertices.data(), vbuf_sz, 0);
	va->SetVertexBuffer(vbuf);

	std::vector<std::shared_ptr<ur::VertexInputAttribute>> vbuf_attrs;
    // Vertex is now { vec3 pos; vec3 normal; float alpha; } = 28 bytes.
    // pos
	vbuf_attrs.push_back(std::make_shared<ur::VertexInputAttribute>(
        0, ur::ComponentDataType::Float, 3, 0, 28
    ));
    // normal
	vbuf_attrs.push_back(std::make_shared<ur::VertexInputAttribute>(
        1, ur::ComponentDataType::Float, 3, 12, 28
    ));
    // alpha (per-vertex opacity)
	vbuf_attrs.push_back(std::make_shared<ur::VertexInputAttribute>(
        2, ur::ComponentDataType::Float, 1, 24, 28
    ));
	va->SetVertexBufferAttrs(vbuf_attrs);

    brepkit::MemProbe("BuildMesh: done (VertexArray built)");
    return va;
}

void TopoAdapter::TriangulationFaces(const TopoDS_Shape& shape, std::vector<Vertex>& vertices, float alpha)
{
    TopLoc_Location aLoc;

    TopTools_IndexedMapOfShape face_map;
    TopExp::MapShapes(shape, TopAbs_FACE, face_map);
    for (int i = 1; i <= face_map.Extent(); i++)
    {
        const TopoDS_Face& act_face = TopoDS::Face(face_map(i));
        Handle(Poly_Triangulation) mesh = BRep_Tool::Triangulation(act_face, aLoc);

        // BRepMesh may under-tessellate curved surfaces; detect and re-triangulate
        if (!mesh.IsNull())
        {
            BRepAdaptor_Surface adapt(act_face);
            if (adapt.GetType() != GeomAbs_Plane && mesh->NbNodes() <= 4)
            {
                BRep_Builder BB;
                BB.UpdateFace(act_face, Handle(Poly_Triangulation)());
                mesh.Nullify();
            }
        }

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

            // Apply the face's TopLoc_Location to bring mesh nodes
            // from the surface's local frame into world space. Some
            // shapes (notably faces produced by BRepPrimAPI_MakePrism)
            // carry a non-identity location; ignoring it puts that
            // face's triangles at the wrong world position, which
            // either places them outside the camera frustum or makes
            // them z-fight with other faces.
            if (!aLoc.IsIdentity())
            {
                const gp_Trsf& trsf = aLoc.Transformation();
                V1.Transform(trsf);
                V2.Transform(trsf);
                V3.Transform(trsf);
            }

            gp_Vec v1(V1.X(), V1.Y(), V1.Z()),
                v2(V2.X(), V2.Y(), V2.Z()),
                v3(V3.X(), V3.Y(), V3.Z());
            gp_Vec normal = (v2 - v1) ^ (v3 - v1);

            sm::vec3 norm(sm::vec3(
                static_cast<float>(normal.X()),
                static_cast<float>(normal.Y()),
                static_cast<float>(normal.Z())
            ));
            vertices.push_back(Vertex({ sm::vec3(
                static_cast<float>(V1.X()),
                static_cast<float>(V1.Y()),
                static_cast<float>(V1.Z())
            ), norm, alpha }));
            vertices.push_back(Vertex({ sm::vec3(
                static_cast<float>(V2.X()),
                static_cast<float>(V2.Y()),
                static_cast<float>(V2.Z())
            ), norm, alpha }));
            vertices.push_back(Vertex({ sm::vec3(
                static_cast<float>(V3.X()),
                static_cast<float>(V3.Y()),
                static_cast<float>(V3.Z())
            ), norm, alpha }));
        }
    }
}

void TopoAdapter::TriangulationEdges(const TopoDS_Shape& shape, std::vector<Vertex>& vertices)
{
    TopTools_IndexedMapOfShape edge_map;
    TopExp::MapShapes(shape, TopAbs_EDGE, edge_map);

    for (int i = 1; i <= edge_map.Extent(); i++)
    {
        const TopoDS_Edge& aEdge = TopoDS::Edge(edge_map(i));
        Standard_Boolean identity = true;
        gp_Trsf myTransf;
        TopLoc_Location aLoc;

        Handle(Poly_Polygon3D) aPoly = PolygonOfEdge(aEdge, aLoc);
        if (aPoly.IsNull()) {
            continue;
        }

        if (!aLoc.IsIdentity()) 
        {
            identity = false;
            myTransf = aLoc.Transformation();
        }

        const TColgp_Array1OfPnt& aNodes = aPoly->Nodes();
        int nbNodesInEdge = aPoly->NbNodes();

        std::vector<sm::vec3> points;

        gp_Pnt pnt;
        for (Standard_Integer j = 1; j <= nbNodesInEdge; j++) 
        {
            pnt = aNodes(j);
            if (!identity) {
                pnt.Transform(myTransf);
            }

            points.push_back(sm::vec3(
                static_cast<float>(pnt.X()),
                static_cast<float>(pnt.Y()),
                static_cast<float>(pnt.Z())
            ));
        }

        if (points.size() < 2) {
            continue;
        }

        for (int j = 0; j < points.size() - 1; ++j)
        {
            auto& p0 = points[j];
            auto& p1 = points[j + 1];

            auto axis = p1 - p0;
            axis.Normalize();

            auto rotated = axis.Cross(sm::vec3(0, 0, 1));
            auto norm = axis.Cross(rotated);

            auto gen_tris = [&](const sm::vec3& extend, const sm::vec3& norm)
            {
                vertices.push_back(Vertex({ p0 - extend, norm }));
                vertices.push_back(Vertex({ p0 + extend, norm }));
                vertices.push_back(Vertex({ p1 - extend, norm }));

                vertices.push_back(Vertex({ p1 - extend, norm }));
                vertices.push_back(Vertex({ p1 + extend, norm }));
                vertices.push_back(Vertex({ p0 + extend, norm }));
            };

            gen_tris(rotated * 0.01, norm);
            gen_tris(norm * 0.01, rotated);
        }
    }
}

}
