# Shim Eigen3 package: points Ceres's find_package(Eigen3) at the shared
# vendored Eigen submodule headers (thirdparty/eigen). The submodule tracks
# Eigen master (5.x dev) which cax uses header-only; this shim lets Ceres
# consume the same headers without requiring a system/installed Eigen, and
# (via the ConfigVersion below) reports the requested version as compatible.
get_filename_component(_eigen_root
    "${CMAKE_CURRENT_LIST_DIR}/../../thirdparty/eigen" ABSOLUTE)

if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${_eigen_root}")
endif()

set(EIGEN3_INCLUDE_DIR  "${_eigen_root}")
set(EIGEN3_INCLUDE_DIRS "${_eigen_root}")
set(Eigen3_FOUND TRUE)
