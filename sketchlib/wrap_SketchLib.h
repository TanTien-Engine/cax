#pragma once

#include <vessel.h>

namespace sketchlib
{

VesselForeignMethodFn SketchLibBindMethod(const char* signature);
void SketchLibBindClass(const char* class_name, VesselForeignClassMethods* methods);

}