# Report any requested Eigen3 version as compatible. The vendored Eigen is
# master (5.x), whose SameMajorVersion config would otherwise reject Ceres's
# find_package(Eigen3 3.3) on the major-version mismatch.
set(PACKAGE_VERSION "3.4.0")
set(PACKAGE_VERSION_COMPATIBLE TRUE)
set(PACKAGE_VERSION_EXACT FALSE)
