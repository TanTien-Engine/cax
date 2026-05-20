# ============================================================
# test/cadcvt_c/CMakeLists.txt
#
# FreeCAD reader golden harness. Builds one executable that walks
# the fixtures directory and diffs each .FCStd / .xml against its
# committed *.ir.golden / *.geo.golden snapshots.
#
# The fixtures directory is passed to the binary at compile time so
# `ctest` can run it from anywhere. To refresh goldens after an
# intentional change:
#
#   ./cadcvt_golden --update
#
# and review the diff before committing.
# ============================================================

cmake_minimum_required(VERSION 3.16)

project(cadcvt_golden)

add_executable(${PROJECT_NAME}
    golden/golden_main.cpp
    golden/IrFingerprint.cpp
    golden/GeoFingerprint.cpp
)

# Repo root: this file is at <root>/test/cadcvt_c, so go up two.
set(CADCVT_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CADCVT_REPO_ROOT}
    ${CADCVT_REPO_ROOT}/thirdparty
    ${CADCVT_REPO_ROOT}/thirdparty/sm
    ${CADCVT_REPO_ROOT}/thirdparty/graph/include
    ${CADCVT_REPO_ROOT}/thirdparty/guard/include
    ${CADCVT_REPO_ROOT}/thirdparty/geoshape/include
    ${CADCVT_REPO_ROOT}/thirdparty/objcomp/include
    ${CADCVT_REPO_ROOT}/thirdparty/vessel/src/include
    ${CADCVT_REPO_ROOT}/thirdparty/wrapper/include
    ${OpenCASCADE_INCLUDE_DIR}
)

# Same OCCT module set as test/brepdb_occt_c -- FreeCadReader +
# Replayer transitively pull in TKBRep / TKTopAlgo / TKFillet / etc.
set(OCCT_GOLDEN_LIBS
    TKernel TKMath TKG2d TKG3d TKGeomBase TKGeomAlgo
    TKBRep TKTopAlgo TKShHealing TKPrim TKMesh TKOffset
    TKDESTEP TKDE TKXSBase TKXCAF TKLCAF TKCDF
    TKBO TKBool TKFillet TKHLR
)

# The parent `cax` static lib already wires FreeCadReader, Replayer,
# TopoShape, brepgraph, and all their dependencies. OGDF is pulled
# in because graph (a cax dep) references it.
target_link_libraries(${PROJECT_NAME} PRIVATE
    cax
    OGDF
    ${OCCT_GOLDEN_LIBS}
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

# cax bundles thirdparty/pugixml/pugixml.cpp (FreeCadReader needs it)
# and vcpkg's OGDF separately vendors its own pugixml. When both are
# in the link line and any pugi::* symbol is referenced (which we do,
# transitively via FreeCadReader), MSVC sees the same pugi symbols
# in caxd.lib(pugixml.obj) and OGDF-debug.lib(pugixml.obj) and emits
# LNK2005. The two copies are ABI-compatible (same upstream lib), so
# /FORCE:MULTIPLE keeps one and silently drops the dup. Suppress the
# accompanying LNK4006 noise too.
if(MSVC)
    target_link_options(${PROJECT_NAME} PRIVATE /FORCE:MULTIPLE /IGNORE:4006)
endif()

# Bake the fixtures path in so `ctest` and a bare invocation both
# find the data without a cwd assumption.
target_compile_definitions(${PROJECT_NAME} PRIVATE
    CADCVT_GOLDEN_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/golden/fixtures"
)

# Group this project under the "cax" folder in the VS solution
# (USE_FOLDERS is enabled at the root). Keeps the solution tidy as
# more cadcvt test executables get added.
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "cax")

include(CTest)
add_test(
    NAME cadcvt_golden
    COMMAND ${PROJECT_NAME}
)
