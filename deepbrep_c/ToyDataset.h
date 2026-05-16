#pragma once

#include "GraphData.h"

#include <random>

namespace deepbrep
{

// Synthetic graph used by demos and unit tests: a box with a single through-
// hole. 6 box faces (label = stock) + 1 cylindrical hole wall + 1 hole-bottom
// face (both labeled hole). Edge features mark the hole-loop edges as
// concave/circle, box edges as convex/line.
GraphData make_box_with_hole_graph(std::mt19937& rng);

}
