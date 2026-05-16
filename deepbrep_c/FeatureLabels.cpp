#include "FeatureLabels.h"

namespace deepbrep
{

const char* face_class_name(int cls)
{
    switch (cls) {
    case 0: return "stock";
    case 1: return "hole";
    case 2: return "slot";
    case 3: return "fillet";
    case 4: return "chamfer";
    case 5: return "pocket";
    default: return "unknown";
    }
}

}
