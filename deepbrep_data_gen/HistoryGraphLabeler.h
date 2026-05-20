#pragma once

#include "OpNameVocabulary.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace brepkit { class TopoShape; }
namespace breptopo  { class HistGraph; }

namespace deepbrep_data_gen
{

struct FaceLabel
{
    int      class_id    = 0;     // index into deepbrep::FaceClass
    uint32_t instance_id = 0;     // groups co-feature faces (= originating op_id)
};

// Resolves a HistGraph op_id to the cax operator name that produced it. The
// expected source is breptopo::CompGraph (which already owns the
// op_id -> step/IRNode mapping); we take it as a callback to avoid a hard
// dependency on CompGraph from this header.
//
// Return empty string when op_id has no known operator (e.g. internal bind).
using OpResolver = std::function<std::string(uint32_t op_id)>;

// Walks the face HistGraph for `shape` and emits one FaceLabel per face, in
// the **same face order** as deepbrep::BRepGraphBuilder::Build emits node
// rows. The two must stay in lock-step or the labels won't line up with the
// GraphData node features.
//
// Algorithm (skeleton -- TODO points marked):
//   for face_idx in TopExp::MapShapes(shape, FACE):
//     uid    = face_hg.GetUID(face)
//     op_id  = HistGraph::OpOf(uid)
//     name   = resolver(op_id)
//     class  = vocab.Lookup(name)
//     emit  { class, instance_id = op_id }
//
// Edge cases the skeleton does NOT yet handle:
//   - A face produced by a later op (e.g. fillet) that "owns" what was
//     formerly a hole wall -- the op_id we get is the fillet's, not the
//     hole's. May need to walk Predecessors() and pick the *deepest
//     semantically-meaningful* op (the hole) instead of the most recent one.
//   - Multi-feature interaction: a single face touched by both a pocket
//     and a fillet -- which wins? Likely needs heuristics or multi-label.
//   - HistGraph::GetUID(face) failing (face not bound) -- happens after
//     deserialize-without-warmup. Skeleton returns class=Stock in that case.
class HistoryGraphLabeler
{
public:
    HistoryGraphLabeler(std::shared_ptr<breptopo::HistGraph> face_hg,
                        OpResolver                           resolver,
                        std::shared_ptr<OpNameVocabulary>    vocab);

    std::vector<FaceLabel> Label(const brepkit::TopoShape& shape) const;

private:
    std::shared_ptr<breptopo::HistGraph> m_face_hg;
    OpResolver                           m_resolver;
    std::shared_ptr<OpNameVocabulary>    m_vocab;
};

}
