#pragma once

#include <vessel.h>

namespace brepgraph
{

VesselForeignMethodFn BrepGraphBindMethod(const char* signature);
void BrepGraphBindClass(const char* class_name, VesselForeignClassMethods* methods);

}