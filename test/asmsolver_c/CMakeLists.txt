# asmsolver scissor_lift solver test. Links only asmsolver (-> Ceres),
# no cax / OCCT, so it builds and runs without the OGDF link chain.
add_executable(asmsolver_test scissor_test.cpp)
target_link_libraries(asmsolver_test PRIVATE asmsolver Ceres::ceres)
target_compile_features(asmsolver_test PRIVATE cxx_std_17)
add_test(NAME asmsolver_scissor COMMAND asmsolver_test)
