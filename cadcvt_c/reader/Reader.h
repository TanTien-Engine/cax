#pragma once

#include "cadapp_c/ir/FeatureIR.h"

#include <string>

// ============================================================
// cadcvt/reader/Reader.h
//
// Abstract base for readers: CAD document -> cadapp::DocumentIR.
//
// Each concrete reader is responsible only for emitting features
// and sketches; OCCT reconstruction is done uniformly by
// emitter/Replayer.
//
// A reader builds a FeatureIR by constructing the matching payload
// struct, then plugging it into FeatureIR::data:
//
//   FeatPayloadExtrude pl;
//   pl.sketch_id      = sk_id;
//   pl.direction[0]   = nx;
//   pl.direction[1]   = ny;
//   pl.direction[2]   = nz;
//   pl.distance       = depth;
//   pl.end_type       = ExtrudeEndType::Blind;
//
//   FeatureIR f;
//   f.id   = next_id();
//   f.type = FeatType::BossExtrude;
//   f.name = sw_feature_name;
//   f.data = std::move(pl);
//   doc.features.push_back(std::move(f));
//
// The MakeFeature<P>(id, type, name, payload) helper in FeatureIR.h
// is the shortcut for the same thing.
//
// Usage (a Reader never pulls in OCCT; each reader's .cpp only
// depends on its own CAD SDK):
//
//   SwReader   reader;
//   DocumentIR doc;
//   if (reader.ReadFile("part.sldprt", doc)) {
//       ...
//   }
// ============================================================

namespace cadcvt
{

class Reader
{
public:
    virtual ~Reader() = default;

    // Read a path (CAD-specific). On failure returns false and
    // writes a reason into err_msg when not null.
    virtual bool ReadFile(const std::string& path,
                          cadapp::DocumentIR& out,
                          std::string*       err_msg = nullptr) = 0;

    // Reader name, written into cadapp::DocumentIR::source for diagnostics.
    virtual const char* Name() const = 0;
};

} // namespace cadcvt
