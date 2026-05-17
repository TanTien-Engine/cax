#pragma once

#include "GraphData.h"

#include <string>
#include <vector>

namespace deepbrep
{

// Reads a multi-sample dataset file produced by DatasetWriter (magic 'DSET').
// Returns the header metadata and all graphs with labels populated.
bool read_dataset(const std::string& path,
                  DatasetHeader& header,
                  std::vector<GraphData>& out);

}
