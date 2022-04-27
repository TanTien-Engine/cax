#pragma once

#include <vessel.h>

namespace gcsgen
{

VesselForeignMethodFn GcsGenBindMethod(const char* signature);
void GcsGenBindClass(const char* class_name, VesselForeignClassMethods* methods);

}