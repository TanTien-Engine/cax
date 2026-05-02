#include "brepir_c/wrap_BrepIR.h"
#include "brepir_c/Data.h"
#include "brepir_c/Sender.h"
#include "brepir_c/Receiver.h"
#include "brepir_c/File.h"

#include <partgraph_c/TopoShape.h>
#include <partgraph_c/GlobalConfig.h>
#include <partgraph_c/TransHelper.h>
#include <breptopo_c/TopoNaming.h>
#include <breptopo_c/HistGraph.h>
#include <breptopo_c/NodeId.h>
#include <wrapper/TransHelper.h>
#include <graph/Node.h>

#include <TopTools_IndexedMapOfShape.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Solid.hxx>
#include <BRep_Builder.hxx>

#include <string>
#include <assert.h>

namespace
{

void w_BrepIR_save()
{
    auto shape = ((wrapper::Proxy<partgraph::TopoShape>*)ves_toforeign(1))->obj;
    std::string filepath = ves_tostring(2);

    const TopoDS_Shape& tshape = shape->GetShape();

	brepir::GeometryPool pool;
    brepir::Sender sender(partgraph::GlobalConfig::Instance()->GetTopoNaming());

    TopTools_IndexedMapOfShape all_shapes;
    TopExp::MapShapes(tshape, all_shapes);

    for (int i = 1; i <= all_shapes.Extent(); ++i)
    {
        const TopoDS_Shape& shape = all_shapes(i);

        uint32_t uid = sender.GetUID(shape);
        if (uid == 0xffffffff)
        {
            TopAbs_ShapeEnum type = shape.ShapeType();
            assert(type != TopAbs_VERTEX && type != TopAbs_EDGE && 
                type != TopAbs_FACE && type != TopAbs_SOLID);
            continue;
        }

        switch (shape.ShapeType())
        {
        case TopAbs_SOLID:
            sender.SerializeSolid(TopoDS::Solid(shape), uid, pool);
            break;
        case TopAbs_FACE:
            sender.SerializeFace(TopoDS::Face(shape), uid, pool);
            break;
        case TopAbs_EDGE:
            sender.SerializeEdge(TopoDS::Edge(shape), uid, pool);
            break;
        case TopAbs_VERTEX:
            sender.SerializeVertex(TopoDS::Vertex(shape), uid, pool);
            break;
        default:
            break;
        }
    }

    brepir::File::Save(filepath, pool);
}

void w_BrepIR_load()
{
    std::string filepath = ves_tostring(1);
    brepir::GeometryPool pool;
    brepir::File::Load(filepath, pool);

    brepir::Receiver receiver(pool);
    BRep_Builder B;
    TopoDS_Compound root_compound;
    B.MakeCompound(root_compound);

    for (const auto& h : pool.headers)
    {
        if (h.type == brepir::Type::Solid)
        {
            TopoDS_Shape solid = receiver.GetShape(h.persistent_id);
            if (!solid.IsNull()) {
                B.Add(root_compound, solid);
            }
        }
    }

    auto shape = std::make_shared<partgraph::TopoShape>(root_compound);
    partgraph::return_topo_shape(shape);
}

}

namespace brepir
{

VesselForeignMethodFn BrepIRBindMethod(const char* signature)
{
    if (strcmp(signature, "static BrepIR.save(_,_)") == 0) return w_BrepIR_save;
    if (strcmp(signature, "static BrepIR.load(_)") == 0) return w_BrepIR_load;

    return nullptr;
}

void BrepIRBindClass(const char* class_name, VesselForeignClassMethods* methods)
{
}

}