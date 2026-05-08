#pragma once

#include <vessel.h>

namespace brepdb
{

VesselForeignMethodFn VersionTreeBindMethod(const char* signature);
void                  VersionTreeBindClass(const char* class_name, VesselForeignClassMethods* methods);

} // namespace brepdb
