#pragma once

#include <vessel.h>

namespace nurbslib
{

VesselForeignMethodFn NurbsLibBindMethod(const char* signature);
void NurbsLibBindClass(const char* class_name, VesselForeignClassMethods* methods);

}