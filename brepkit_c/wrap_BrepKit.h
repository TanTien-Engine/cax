#pragma once

#include <vessel.h>

namespace brepkit
{

VesselForeignMethodFn BrepKitBindMethod(const char* signature);
void BrepKitBindClass(const char* class_name, VesselForeignClassMethods* methods);

}