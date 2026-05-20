#include "brepdb_c/StepFile.h"
#include "brepdb_c/TypedPool.h"
#include "brepdb_c/WorldSender.h"
#include "brepdb_c/WorldReceiver.h"

#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <Interface_Static.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

namespace brepdb
{

bool StepFile::Export(const std::string& filename, const BRepWorld& world)
{
    // Reconstruct TopoDS_Shape from BRepWorld
    WorldReceiver receiver(world);
    TopoDS_Shape compound = receiver.GetAll();
    if (compound.IsNull())
        return false;

    // Check that there is at least one real sub-shape
    TopExp_Explorer any_check(compound, TopAbs_VERTEX);
    if (!any_check.More())
        return false;

    STEPControl_Writer writer;
    Interface_Static::SetCVal("write.step.schema", "AP214");

    bool has_shape = false;
    for (TopExp_Explorer exp(compound, TopAbs_SOLID); exp.More(); exp.Next()) {
        IFSelect_ReturnStatus status = writer.Transfer(exp.Current(), STEPControl_AsIs);
        if (status == IFSelect_RetDone)
            has_shape = true;
    }

    if (!has_shape) {
        IFSelect_ReturnStatus status = writer.Transfer(compound, STEPControl_AsIs);
        if (status != IFSelect_RetDone)
            return false;
    }

    IFSelect_ReturnStatus write_status = writer.Write(filename.c_str());
    return write_status == IFSelect_RetDone;
}

bool StepFile::Import(const std::string& filename, BRepWorld& world,
                      const std::shared_ptr<brepgraph::TopoNaming>& tn)
{
    STEPControl_Reader reader;
    IFSelect_ReturnStatus status = reader.ReadFile(filename.c_str());
    if (status != IFSelect_RetDone)
        return false;

    reader.TransferRoots();
    TopoDS_Shape shape = reader.OneShape();
    if (shape.IsNull())
        return false;

    world.Clear();

    WorldSender sender(tn);
    sender.Serialize(shape, world);

    return world.EntityCount() > 0;
}

} // namespace brepdb
