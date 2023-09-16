#pragma once

#include <vessel.h>

namespace breptopo
{

VesselForeignMethodFn BrepTopoBindMethod(const char* signature);
void BrepTopoBindClass(const char* class_name, VesselForeignClassMethods* methods);

}