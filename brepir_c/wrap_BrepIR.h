#pragma once

#include <vessel.h>

namespace brepir
{

VesselForeignMethodFn BrepIRBindMethod(const char* signature);
void BrepIRBindClass(const char* class_name, VesselForeignClassMethods* methods);

}