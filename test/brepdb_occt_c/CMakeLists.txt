cmake_minimum_required(VERSION 3.16)

project(brepdb_occt_test)

include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.2
)
FetchContent_MakeAvailable(Catch2)

# OCCT-dependent brepdb tests.
# Links against the parent cax library to avoid duplicating the deep
# dependency chain (WorldSender -> HistGraph -> GraphLayout -> OGDF, etc.).
add_executable(${PROJECT_NAME}
    catch_main.cpp
    test_step_file.cpp
    test_world_roundtrip.cpp
    test_prim_maker.cpp
    test_shape_selector.cpp
    test_shape_tools.cpp
    test_topo_algo.cpp
    test_topo_graph_builder.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty
    ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/sm
    ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/graph/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/guard/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/geoshape/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/objcomp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/vessel/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/wrapper/include
    ${OpenCASCADE_INCLUDE_DIR}
)

set(OCCT_TEST_LIBS
    TKernel TKMath TKG2d TKG3d TKGeomBase TKGeomAlgo
    TKBRep TKTopAlgo TKShHealing TKPrim TKMesh TKOffset
    TKDESTEP TKDE TKXSBase TKXCAF TKLCAF TKCDF
    TKBO TKBool TKFillet TKHLR
)
target_link_libraries(${PROJECT_NAME} PRIVATE cax Catch2::Catch2 OGDF ${OCCT_TEST_LIBS})

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

include(CTest)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

if(WIN32 AND OpenCASCADE_BINARY_DIR)
    set(_DL_PATHS "${OpenCASCADE_BINARY_DIR}")
else()
    set(_DL_PATHS "")
endif()
catch_discover_tests(${PROJECT_NAME} DL_PATHS ${_DL_PATHS})
