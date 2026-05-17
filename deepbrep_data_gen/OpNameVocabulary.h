#pragma once

#include "deepbrep_c/FeatureLabels.h"

#include <string>
#include <unordered_map>

namespace deepbrep_data_gen
{

// Maps a physical-level cax operator name (as recorded in
// breptopo::CompGraph -- e.g. "PrimMaker.Box", "TopoAlgo.Cut") to a
// deepbrep::FaceClass. This is the recognizer's output vocabulary.
//
// Today the cax blueprint exposes only physical operators -- "Cut" is the
// same op whether the user meant a hole, a pocket, or a slot. So this
// mapping is necessarily coarse:
//   TopoAlgo.Cut    -> Hole          (most-common interpretation; biased)
//   TopoAlgo.Fillet -> Fillet
//   TopoAlgo.Chamfer-> Chamfer
//   PrimMaker.*     -> Stock
//
// When the blueprint gains "group" nodes (semantic-level macros that the
// user explicitly tags as Hole / Pocket / Slot), those group names become
// the new vocab keys and the fine-grained distinction becomes lossless.
// Until then, datasets generated through this map carry a known label bias
// on Cut -- the recognizer trained on it will under-distinguish hole vs
// pocket.
class OpNameVocabulary
{
public:
    // Builds the default physical-level mapping.
    OpNameVocabulary();

    // Override or add an entry. Empty string is treated as "no mapping" and
    // falls through to Stock.
    void Set(const std::string& op_name, deepbrep::FaceClass cls);

    // FaceClass::Stock when op_name is empty or has no entry.
    deepbrep::FaceClass Lookup(const std::string& op_name) const;

    // Convenience for data-driven config: load "op_name=ClassName" pairs
    // from a flat text file (one per line, '#' starts a comment).
    bool LoadFromFile(const std::string& path);

private:
    std::unordered_map<std::string, deepbrep::FaceClass> m_table;
};

}
