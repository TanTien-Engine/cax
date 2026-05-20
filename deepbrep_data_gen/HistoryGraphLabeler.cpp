#include "HistoryGraphLabeler.h"

#include "brepgraph_c/history/HistGraph.h"
#include "brepkit_c/TopoShape.h"
#include "deepbrep_c/FeatureLabels.h"

// OCCT
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

namespace deepbrep_data_gen
{

HistoryGraphLabeler::HistoryGraphLabeler(
    std::shared_ptr<brepgraph::HistGraph> face_hg,
    OpResolver                           resolver,
    std::shared_ptr<OpNameVocabulary>    vocab)
    : m_face_hg(std::move(face_hg))
    , m_resolver(std::move(resolver))
    , m_vocab(std::move(vocab))
{
}

std::vector<FaceLabel>
HistoryGraphLabeler::Label(const brepkit::TopoShape& shape) const
{
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape.GetShape(), TopAbs_FACE, faces);

    std::vector<FaceLabel> out;
    out.reserve(faces.Extent());

    for (int i = 1; i <= faces.Extent(); ++i) {
        FaceLabel lbl;
        const TopoDS_Face& face = TopoDS::Face(faces.FindKey(i));

        const uint32_t uid = m_face_hg ? m_face_hg->GetUID(face) : 0xFFFFFFFFu;
        if (uid == 0xFFFFFFFFu) {
            // No lineage -- face wasn't recorded by the history pipeline.
            // Default to Stock so it doesn't poison the dataset.
            lbl.class_id    = static_cast<int>(deepbrep::FaceClass::Stock);
            lbl.instance_id = 0;
            out.push_back(std::move(lbl));
            continue;
        }

        const uint32_t op_id = brepgraph::HistGraph::OpOf(uid);
        const std::string name = m_resolver ? m_resolver(op_id) : std::string();

        // TODO(provenance): when the most-recent op is a low-level boolean
        // (e.g. Cut used internally by a fillet macro), walk Predecessors()
        // back to the deepest op whose name is in the vocab. Naive
        // most-recent pick mislabels chained features.

        lbl.class_id    = static_cast<int>(m_vocab->Lookup(name));
        lbl.instance_id = op_id;
        out.push_back(std::move(lbl));
    }
    return out;
}

}
