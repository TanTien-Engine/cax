# ============================================================
# tools/zw_verify/CMakeLists.txt
#
# zw_verify: the per-file zero-diff gate for ZW3D conversion.
# Replays a zw_export .cax.json (ZwReader + Replayer, both inside the
# cax lib) and diffs the body against ZW3D's own .cax.step truth via
# OCCT. Same link recipe as test/cadcvt_c (cax + OGDF + OCCT modules);
# see that file for the /FORCE:MULTIPLE pugixml note.
# ============================================================

cmake_minimum_required(VERSION 3.16)
project(zw_verify)

add_executable(${PROJECT_NAME} main.cpp)

set(ZWV_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

target_include_directories(${PROJECT_NAME} PRIVATE
    ${ZWV_REPO_ROOT}
    ${ZWV_REPO_ROOT}/thirdparty
    ${ZWV_REPO_ROOT}/thirdparty/sm
    ${ZWV_REPO_ROOT}/thirdparty/graph/include
    ${ZWV_REPO_ROOT}/thirdparty/guard/include
    ${ZWV_REPO_ROOT}/thirdparty/geoshape/include
    ${ZWV_REPO_ROOT}/thirdparty/objcomp/include
    ${ZWV_REPO_ROOT}/thirdparty/vessel/src/include
    ${ZWV_REPO_ROOT}/thirdparty/wrapper/include
    ${ZWV_REPO_ROOT}/thirdparty/nlohmann/single_include
    ${OpenCASCADE_INCLUDE_DIR}
)

set(ZWV_OCCT_LIBS
    TKernel TKMath TKG2d TKG3d TKGeomBase TKGeomAlgo
    TKBRep TKTopAlgo TKShHealing TKPrim TKMesh TKOffset
    TKDESTEP TKDE TKXSBase TKXCAF TKLCAF TKCDF
    TKBO TKBool TKFillet TKHLR
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    cax
    OGDF
    ${ZWV_OCCT_LIBS}
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

if(MSVC)
    target_link_options(${PROJECT_NAME} PRIVATE /FORCE:MULTIPLE /IGNORE:4006)
endif()
