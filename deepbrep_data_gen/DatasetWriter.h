#pragma once

#include "HistoryGraphLabeler.h"
#include "deepbrep_c/GraphData.h"

#include <cstdio>
#include <string>
#include <vector>

namespace deepbrep_data_gen
{

// Streaming writer for a multi-sample training set. One file holds many
// (GraphData, labels) pairs back-to-back; opens once, append per-sample.
//
// On-disk layout:
//   magic 'DSET'  (4 bytes)
//   uint32 sample_count_placeholder (patched in Close())
//   uint32 node_feat_dim
//   uint32 edge_feat_dim
//   uint32 num_classes
//   repeated SAMPLE:
//     a complete GraphData payload as written by write_graph_data() -- but
//     dumped to the same file handle in place of a fresh file. The graph's
//     labels / instance_ids fields carry the per-face ground truth (set by
//     the caller via merge_labels_into_graph()).
//
// Note: writing requires a seekable stream so Close() can patch the count.
class DatasetWriter
{
public:
    DatasetWriter(const std::string& path,
                  uint32_t node_feat_dim,
                  uint32_t edge_feat_dim,
                  uint32_t num_classes);
    ~DatasetWriter();

    // Move the per-face FaceLabel.class_id / instance_id into g, then append.
    // Regression params are dropped for now -- the GNN doesn't consume them
    // yet. Add a side-channel when the regression head lands.
    bool Append(deepbrep::GraphData& g, const std::vector<FaceLabel>& labels);

    // Patches the sample count and closes the file. Safe to call twice.
    void Close();

    uint32_t SampleCount() const { return m_count; }

private:
    std::FILE*  m_file = nullptr;
    uint32_t    m_count = 0;
    long        m_count_offset = 0;  // file offset of the count u32
};

// Copies FaceLabel fields into the GraphData's per-node label vectors. Length
// must match graph.num_nodes; otherwise this leaves g.labels empty and
// returns false.
bool merge_labels_into_graph(deepbrep::GraphData& g,
                             const std::vector<FaceLabel>& labels);

}
