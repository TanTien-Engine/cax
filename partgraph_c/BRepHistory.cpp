#include "BRepHistory.h"
#include "TopoDataset.h"

// OCCT
#include <TopExp.hxx>
#include <BRepBuilderAPI_MakeShape.hxx>
#include <BRepTools_History.hxx>
#include <BRepOffset_MakeSimpleOffset.hxx>

namespace
{

class BuilderHist : public partgraph::BRepHistory::History
{
public:
    BuilderHist(BRepBuilderAPI_MakeShape& builder) : m_builder(builder) {}

    virtual const TopTools_ListOfShape& Generated(const TopoDS_Shape& S) {
        return m_builder.Generated(S);
    }
    virtual const TopTools_ListOfShape& Modified(const TopoDS_Shape& S) {
        return m_builder.Modified(S);
    }
    virtual Standard_Boolean IsDeleted(const TopoDS_Shape& S) {
        return m_builder.IsDeleted(S);
    }

private:
    BRepBuilderAPI_MakeShape& m_builder;

}; // BuilderHist

class ToolsHist : public partgraph::BRepHistory::History
{
public:
    ToolsHist(opencascade::handle<BRepTools_History> hist) : m_hist(hist) {}

    virtual const TopTools_ListOfShape& Generated(const TopoDS_Shape& S) {
        return m_hist->Generated(S);
    }
    virtual const TopTools_ListOfShape& Modified(const TopoDS_Shape& S) {
        return m_hist->Modified(S);
    }
    virtual Standard_Boolean IsDeleted(const TopoDS_Shape& S) {
        return m_hist->IsRemoved(S);
    }

private:
    opencascade::handle<BRepTools_History> m_hist;

}; // ToolsHist

class OffsetHist : public partgraph::BRepHistory::History
{
public:
    OffsetHist(const BRepOffset_MakeSimpleOffset& builder) : m_builder(builder) {}

    virtual const TopTools_ListOfShape& Generated(const TopoDS_Shape& S)
    {
        m_new_list.Clear();
        m_new_list.Append(m_builder.Generated(S));
        return m_new_list;
    }
    virtual const TopTools_ListOfShape& Modified(const TopoDS_Shape& S)
    {
        m_mod_list.Clear();
        m_mod_list.Append(m_builder.Modified(S));
        return m_mod_list;
    }
    virtual Standard_Boolean IsDeleted(const TopoDS_Shape& S) {
        return false;
    }

private:
    const BRepOffset_MakeSimpleOffset& m_builder;

    TopTools_ListOfShape m_new_list, m_mod_list;

}; // OffsetHist

}

namespace partgraph
{

BRepHistory::BRepHistory(BRepBuilderAPI_MakeShape& builder, TopAbs_ShapeEnum type, 
	                     const TopoShape& new_shape, const TopoShape& old_shape)
{
    auto hist = std::make_shared<BuilderHist>(builder);
    BuildHistory(hist, type, new_shape, old_shape);
}

BRepHistory::BRepHistory(opencascade::handle<BRepTools_History> hist, TopAbs_ShapeEnum type,
                         const TopoShape& new_shape, const TopoShape& old_shape)
{
    auto h = std::make_shared<ToolsHist>(hist);
    BuildHistory(h, type, new_shape, old_shape);
}

BRepHistory::BRepHistory(const BRepOffset_MakeSimpleOffset& builder, const TopoShape& old_shape)
{
    auto hist = std::make_shared<OffsetHist>(builder);
    BuildHistory(hist, old_shape.GetShape().ShapeType(), builder.GetResultShape(), old_shape);
}

void BRepHistory::BuildHistory(const std::shared_ptr<History>& hist, TopAbs_ShapeEnum type,
                               const TopoShape& new_shape, const TopoShape& old_shape)
{
    TopExp::MapShapes(new_shape.GetShape(), type, m_new_map); // map containing all old objects of type "type"
    TopExp::MapShapes(old_shape.GetShape(), type, m_old_map); // map containing all new objects of type "type"

    // Look at all objects in the old shape and try to find the modified object in the new shape
    for (int i = 1; i <= m_old_map.Extent(); i++)
    {
        bool found = false;
        TopTools_ListIteratorOfListOfShape it;
        // Find all new objects that are a modification of the old object (e.g. a face was resized)
        for (it.Initialize(hist->Modified(m_old_map(i))); it.More(); it.Next())
        {
            found = true;
            for (int j = 1; j <= m_new_map.Extent(); j++) { // one old object might create several new ones!
                if (m_new_map(j).IsPartner(it.Value())) {
                    m_shape_map[i-1].push_back(j-1); // adjust indices to start at zero
                    break;
                }
            }
        }

        // Find all new objects that were generated from an old object (e.g. a face generated from an edge)
        for (it.Initialize(hist->Generated(m_old_map(i))); it.More(); it.Next())
        {
            found = true;
            for (int j = 1; j <= m_new_map.Extent(); j++) {
                if (m_new_map(j).IsPartner(it.Value())) {
                    m_shape_map[i-1].push_back(j-1);
                    break;
                }
            }
        }

        if (!found) 
        {
            // Find all old objects that don't exist any more (e.g. a face was completely cut away)
            if (hist->IsDeleted(m_old_map(i)))
            {
                m_shape_map[i-1] = std::vector<int>();
            }
            else 
            {
                // Mop up the rest (will this ever be reached?)
                for (int j = 1; j <= m_new_map.Extent(); j++) {
                    if (m_new_map(j).IsPartner(m_old_map(i))) {
                        m_shape_map[i-1].push_back(j-1);
                        break;
                    }
                }
            }
        }
    }
}

}