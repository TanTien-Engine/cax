cmake_minimum_required(VERSION 3.16)

project(deepbrep_test)

include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.2
)
FetchContent_MakeAvailable(Catch2)

# Compile the pure-C++ core sources directly so the test binary doesn't pull
# in OCCT/Boost/vessel from the main cax target.
add_executable(${PROJECT_NAME}
    catch_main.cpp
    test_tensor.cpp
    test_layers.cpp
    test_gnn_model.cpp
    test_graph_data.cpp
    test_toy_train.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../deepbrep_c/Tensor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../deepbrep_c/GraphData.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../deepbrep_c/Layers.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../deepbrep_c/GNNModel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../deepbrep_c/Trainer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../deepbrep_c/FeatureLabels.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../deepbrep_c/ToyDataset.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../deepbrep_c
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

target_link_libraries(${PROJECT_NAME} PRIVATE Catch2::Catch2)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

include(CTest)
# When this subdir is configured after another that already pulled Catch2 via
# FetchContent, the second MakeAvailable is a no-op and the extras/ dir
# stays out of CMAKE_MODULE_PATH for this scope -- so include(Catch) below
# can't find Catch.cmake. Push the path in explicitly.
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

catch_discover_tests(${PROJECT_NAME})
