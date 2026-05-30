# asmsolver scissor_lift solver test (hardcoded data). Links only
# asmsolver -> Ceres, no cax / OCCT, so it builds and runs without the
# OGDF link chain.
add_executable(asmsolver_test scissor_test.cpp)
target_link_libraries(asmsolver_test PRIVATE asmsolver Ceres::ceres)
target_compile_features(asmsolver_test PRIVATE cxx_std_17)
add_test(NAME asmsolver_scissor COMMAND asmsolver_test)

# End-to-end: real FreeCadReader -> IR adapter -> solve. Needs the cax
# lib (reader) + OCCT + OGDF, same link surface as the cadcvt golden
# harness, so it is gated on the same FreeCAD-deps flag.
if(CADCVT_FREECAD_DEPS_OK)
    set(ASM_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
    set(OCCT_ASM_LIBS
        TKernel TKMath TKG2d TKG3d TKGeomBase TKGeomAlgo
        TKBRep TKTopAlgo TKShHealing TKPrim TKMesh TKOffset
        TKDESTEP TKDE TKXSBase TKXCAF TKLCAF TKCDF
        TKBO TKBool TKFillet TKHLR
    )
    # IrAdapterGeom (OCCT-dependent cylinder-radius resolution) + AsmSession
    # are compiled into the cax lib when CAX_ASMSOLVER_OK, so this test gets
    # them by linking cax -- do NOT compile IrAdapterGeom here too or clang
    # flags a duplicate symbol (only MSVC tolerated it via /FORCE:MULTIPLE).
    add_executable(asmsolver_import_test
        import_solve_test.cpp)
    target_include_directories(asmsolver_import_test PRIVATE
        ${ASM_REPO_ROOT} ${OpenCASCADE_INCLUDE_DIR})
    target_link_libraries(asmsolver_import_test PRIVATE
        asmsolver cax OGDF Ceres::ceres ${OCCT_ASM_LIBS})
    target_compile_features(asmsolver_import_test PRIVATE cxx_std_17)
    target_compile_definitions(asmsolver_import_test PRIVATE
        ASM_SCISSOR_FCSTD="${ASM_REPO_ROOT}/test/cadcvt_c/golden/fixtures/examples/freecad_assembly_test_corpus/native_AssemblyWB/scissor_lift.FCStd")
    if(MSVC)
        target_link_options(asmsolver_import_test PRIVATE /FORCE:MULTIPLE /IGNORE:4006)
    endif()
    add_test(NAME asmsolver_import COMMAND asmsolver_import_test)

    # Interactive pick + drag via cadcvt::AsmSession (Increment 6). Same link
    # surface (cax provides AsmSession + IrAdapterGeom; OCCT for bbox checks).
    add_executable(asmsession_drag_test asmsession_drag_test.cpp)
    target_include_directories(asmsession_drag_test PRIVATE
        ${ASM_REPO_ROOT} ${OpenCASCADE_INCLUDE_DIR})
    target_link_libraries(asmsession_drag_test PRIVATE
        asmsolver cax OGDF Ceres::ceres ${OCCT_ASM_LIBS})
    target_compile_features(asmsession_drag_test PRIVATE cxx_std_17)
    target_compile_definitions(asmsession_drag_test PRIVATE
        ASM_SCISSOR_FCSTD="${ASM_REPO_ROOT}/test/cadcvt_c/golden/fixtures/examples/freecad_assembly_test_corpus/native_AssemblyWB/scissor_lift.FCStd")
    if(MSVC)
        target_link_options(asmsession_drag_test PRIVATE /FORCE:MULTIPLE /IGNORE:4006)
    endif()
    add_test(NAME asmsession_drag COMMAND asmsession_drag_test)
endif()
