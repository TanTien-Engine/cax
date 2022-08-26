#pragma once

#include <vessel.h>

namespace partgraph
{

VesselForeignMethodFn PartGraphBindMethod(const char* signature);
void PartGraphBindClass(const char* class_name, VesselForeignClassMethods* methods);

}