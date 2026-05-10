cmake_minimum_required(VERSION 3.16)

project(brepdb_test)

include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.2
)
FetchContent_MakeAvailable(Catch2)

# VersionTree.cpp is compiled directly to avoid pulling OCCT/Boost into tests.
add_executable(${PROJECT_NAME}
    catch_main.cpp
    test_version_tree.cpp
    test_component_view.cpp
    test_component_diff.cpp
    test_typed_pool.cpp
    test_world_file.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../brepdb_c/VersionTree.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../brepdb_c/WorldFile.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../brepdb_c
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

# Link Catch2::Catch2 (no main), we supply our own in catch_main.cpp.
# This avoids the MSVC LNK2019 / MSVCRTD conflict with Catch2WithMain.
target_link_libraries(${PROJECT_NAME} PRIVATE Catch2::Catch2)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

include(CTest)
include(Catch)

catch_discover_tests(${PROJECT_NAME})
