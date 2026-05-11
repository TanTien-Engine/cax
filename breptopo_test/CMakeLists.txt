cmake_minimum_required(VERSION 3.16)

project(breptopo_test)

include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.2
)
FetchContent_MakeAvailable(Catch2)

# CompGraphCore.cpp is the OCCT-independent core (IRGraph, Evaluator, etc.).
add_executable(${PROJECT_NAME}
    catch_main.cpp
    test_shape_cache.cpp
    test_evaluator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../breptopo_c/CompGraphCore.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../breptopo_c
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(${PROJECT_NAME} PRIVATE Catch2::Catch2)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

include(CTest)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

catch_discover_tests(${PROJECT_NAME})
