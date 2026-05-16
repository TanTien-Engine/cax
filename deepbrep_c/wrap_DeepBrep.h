#pragma once

#include <vessel.h>

namespace deepbrep
{

VesselForeignMethodFn DeepBrepBindMethod(const char* signature);
void DeepBrepBindClass(const char* class_name, VesselForeignClassMethods* methods);

}
