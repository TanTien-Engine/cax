#pragma once

#include <vessel.h>

// ============================================================
// cadcvt/wrap_CadCvt.h
//
// Vessel binding entry points. Wire these into the host editor's
// main.cpp next to PartGraphBindMethod / PartGraphBindClass:
//
//   // module source lookup
//   } else if (strcmp(module, "cadcvt") == 0) {
//       source = cadcvtModuleSource;
//   }
//
//   // class binder chain
//   cadcvt::CadCvtBindClass(className, &methods);
//   if (methods.allocate != NULL) return methods;
//
//   // method binder chain
//   method = cadcvt::CadCvtBindMethod(fullName);
//   if (method != NULL) return method;
// ============================================================

namespace cadcvt
{

VesselForeignMethodFn CadCvtBindMethod(const char* signature);
void                  CadCvtBindClass (const char* class_name,
                                       VesselForeignClassMethods* methods);

} // namespace cadcvt
