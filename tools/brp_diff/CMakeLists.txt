# ============================================================
# tools/brp_diff/CMakeLists.txt
#
# Auto-diff per-feature .brp dumps from cax against the matching
# PartShapeN.brp entries inside a FreeCAD .FCStd zip.
#
# Build (assuming `cax` static lib is already configured):
#
#   cmake --build <build_dir> --target brp_diff
#
# Run:
#
#   ./brp_diff path/to/Part.fcstd [dump_dir]
# ============================================================

cmake_minimum_required(VERSION 3.16)

project(brp_diff)

add_executable(${PROJECT_NAME}
    main.cpp
)

set(BRP_DIFF_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

target_include_directories(${PROJECT_NAME} PRIVATE
    ${BRP_DIFF_REPO_ROOT}
    ${BRP_DIFF_REPO_ROOT}/thirdparty
    ${BRP_DIFF_REPO_ROOT}/thirdparty/miniz
    ${BRP_DIFF_REPO_ROOT}/thirdparty/pugixml/src
    ${BRP_DIFF_REPO_ROOT}/thirdparty/sm
    ${BRP_DIFF_REPO_ROOT}/thirdparty/graph/include
    ${BRP_DIFF_REPO_ROOT}/thirdparty/guard/include
    ${BRP_DIFF_REPO_ROOT}/thirdparty/geoshape/include
    ${BRP_DIFF_REPO_ROOT}/thirdparty/objcomp/include
    ${BRP_DIFF_REPO_ROOT}/thirdparty/vessel/src/include
    ${BRP_DIFF_REPO_ROOT}/thirdparty/wrapper/include
    # miniz.h #include's miniz_export.h, which is normally produced
    # by miniz's own CMake. The root CMakeLists writes a stub at
    # configure time into CADCVT_MINIZ_EXPORT_DIR -- reuse it here.
    "${CADCVT_MINIZ_EXPORT_DIR}"
    ${OpenCASCADE_INCLUDE_DIR}
)

# OCCT module set: BRepTools::Read + BRepGProp + BRepBndLib.
set(BRP_DIFF_OCCT_LIBS
    TKernel TKMath TKG2d TKG3d TKGeomBase
    TKBRep TKTopAlgo TKShHealing
)

# Reuse cax's TopoGeomUtils via the cax static lib, and the bundled
# miniz / pugixml that cax already pulls in.
target_link_libraries(${PROJECT_NAME} PRIVATE
    cax
    OGDF
    ${BRP_DIFF_OCCT_LIBS}
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

# Same MSVC duplicate-pugixml workaround the golden harness needs:
# cax bundles thirdparty/pugixml and OGDF (vcpkg) bundles its own.
if(MSVC)
    target_link_options(${PROJECT_NAME} PRIVATE /FORCE:MULTIPLE /IGNORE:4006)
endif()
