#pragma once

#include <vessel.h>

namespace brepdb
{

VesselForeignMethodFn BrepDBBindMethod(const char* signature);
void BrepDBBindClass(const char* class_name, VesselForeignClassMethods* methods);

}