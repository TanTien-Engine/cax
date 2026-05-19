cmake_minimum_required(VERSION 3.16)

project(cax)

add_definitions(
    -DNO_RTTR
)

################################################################################
# Source groups
################################################################################
set(breptopo
    "breptopo_c/BrepTopo.cpp"
    "breptopo_c/BrepTopo.h"
)
source_group("breptopo" FILES ${breptopo})

set(breptopo__computation
    "breptopo_c/CompGraph.cpp"
    "breptopo_c/CompGraphCore.cpp"
    "breptopo_c/CompGraph.h"
    "breptopo_c/CompGraphBuilder.cpp"
    "breptopo_c/CompGraphBuilder.h"
    "breptopo_c/ShapeCache.h"
    "breptopo_c/comp_ops.cpp"
    "breptopo_c/comp_ops.h"
)
source_group("breptopo\\computation" FILES ${breptopo__computation})

set(breptopo__history
    "breptopo_c/HistGraph.cpp"
    "breptopo_c/HistGraph.h"
    "breptopo_c/TopoNaming.cpp"
    "breptopo_c/TopoNaming.h"
)
source_group("breptopo\\history" FILES ${breptopo__history})

# Viz-only TUs that pull graph + ogdf. Kept in cax static lib so UI callers
# transparently get the symbols, but isolated in their own object so headless
# consumers (data gen, training, recognition) don't drag graph/ogdf via
# unrelated lineage methods.
set(breptopo__history__viz
    "breptopo_c/HistGraphViz.cpp"
)
source_group("breptopo\\history\\viz" FILES ${breptopo__history__viz})

set(breptopo__history__comp
    "breptopo_c/NodeFlags.h"
    "breptopo_c/NodeId.h"
)
source_group("breptopo\\history\\comp" FILES ${breptopo__history__comp})

set(breptopo__script
    "breptopo_c/breptopo.ves.inc"
    "breptopo_c/wrap_BrepTopo.cpp"
    "breptopo_c/wrap_BrepTopo.h"
)
source_group("breptopo\\script" FILES ${breptopo__script})

set(breptopo__tools__comp
    "breptopo_c/NodeShape.h"
)
source_group("breptopo\\tools\\comp" FILES ${breptopo__tools__comp})

set(breptopo__topology
    "breptopo_c/TopoGraph.cpp"
    "breptopo_c/TopoGraph.h"
    "breptopo_c/TopoGraphBuilder.cpp"
    "breptopo_c/TopoGraphBuilder.h"
)
source_group("breptopo\\topology" FILES ${breptopo__topology})

set(breptopo__topology__comp
    "breptopo_c/GraphShape.h"
)
source_group("breptopo\\topology\\comp" FILES ${breptopo__topology__comp})

set(nurbslib
    "nurbslib/nurbslib.ves.inc"
    "nurbslib/wrap_NurbsLib.cpp"
    "nurbslib/wrap_NurbsLib.h"
)
source_group("nurbslib" FILES ${nurbslib})

set(partgraph__brep
    "partgraph_c/BRepBuilder.cpp"
    "partgraph_c/BRepBuilder.h"
    "partgraph_c/BRepHistory.cpp"
    "partgraph_c/BRepHistory.h"
    "partgraph_c/BRepSelector.cpp"
    "partgraph_c/BRepSelector.h"
    "partgraph_c/BRepTools.cpp"
    "partgraph_c/BRepTools.h"
    "partgraph_c/GeomDataset.h"
    "partgraph_c/PrimMaker.cpp"
    "partgraph_c/PrimMaker.h"
    "partgraph_c/TopoShape.cpp"
    "partgraph_c/TopoShape.h"
)
source_group("partgraph\\brep" FILES ${partgraph__brep})

set(partgraph__script
    "partgraph_c/partgraph.ves.inc"
    "partgraph_c/TransHelper.cpp"
    "partgraph_c/TransHelper.h"
    "partgraph_c/wrap_PartGraph.cpp"
    "partgraph_c/wrap_PartGraph.h"
)
source_group("partgraph\\script" FILES ${partgraph__script})

set(partgraph__tools
    "partgraph_c/occt_adapter.h"
    "partgraph_c/TopoAdapter.cpp"
    "partgraph_c/TopoAdapter.h"
    "partgraph_c/TopoAlgo.cpp"
    "partgraph_c/TopoAlgo.h"
    "partgraph_c/TopoAlgo_Ext.cpp"
    "partgraph_c/TopoAlgo_Ext.h"
    "partgraph_c/GlobalConfig.cpp"
    "partgraph_c/GlobalConfig.h"
)
source_group("partgraph\\tools" FILES ${partgraph__tools})

set(sketchlib__data
    "sketchlib/Constraint.h"
    "sketchlib/Geometry.h"
    "sketchlib/Scene.cpp"
    "sketchlib/Scene.h"
    "sketchlib/Util.h"
)
source_group("sketchlib\\data" FILES ${sketchlib__data})

set(sketchlib__external__PlaneGCS
    "thirdparty/PlaneGCS/Constraints.cpp"
    "thirdparty/PlaneGCS/Constraints.h"
    "thirdparty/PlaneGCS/GCS.cpp"
    "thirdparty/PlaneGCS/GCS.h"
    "thirdparty/PlaneGCS/Geo.cpp"
    "thirdparty/PlaneGCS/Geo.h"
    "thirdparty/PlaneGCS/qp_eq.cpp"
    "thirdparty/PlaneGCS/qp_eq.h"
    "thirdparty/PlaneGCS/SubSystem.cpp"
    "thirdparty/PlaneGCS/SubSystem.h"
)
source_group("sketchlib\\external\\PlaneGCS" FILES ${sketchlib__external__PlaneGCS})

set(sketchlib__script
    "sketchlib/sketchlib.ves.inc"
    "sketchlib/wrap_SketchLib.cpp"
    "sketchlib/wrap_SketchLib.h"
    "thirdparty/PlaneGCS/Util.h"
)
source_group("sketchlib\\script" FILES ${sketchlib__script})

set(brepdb
    "brepdb_c/BrepDB.cpp"
    "brepdb_c/BrepDB.h"
)
source_group("brepdb" FILES ${brepdb})

set(brepdb__geom
    "brepdb_c/WorldSender.cpp"
    "brepdb_c/WorldSender.h"
    "brepdb_c/WorldReceiver.cpp"
    "brepdb_c/WorldReceiver.h"
    "brepdb_c/WorldFile.cpp"
    "brepdb_c/WorldFile.h"
    "brepdb_c/StepFile.cpp"
    "brepdb_c/StepFile.h"
    "brepdb_c/TypedPool.h"
    "brepdb_c/ComponentView.h"
    "brepdb_c/ComponentDiff.h"
    "brepdb_c/ComponentDiff.cpp"
)
source_group("brepdb\\geom" FILES ${brepdb__geom})

set(brepdb__topo
    "brepdb_c/TopoGraph.cpp"
    "brepdb_c/TopoGraph.h"
)
source_group("brepdb\\topo" FILES ${brepdb__topo})

set(brepdb__index
    "brepdb_c/ShapeIndex.cpp"
    "brepdb_c/ShapeIndex.h"
)
source_group("brepdb\\index" FILES ${brepdb__index})

set(brepdb__version
    "brepdb_c/VersionTree.cpp"
    "brepdb_c/VersionTree.h"
    "brepdb_c/NodeVersionInfo.h"
    "brepdb_c/VersionGraph.cpp"
    "brepdb_c/VersionGraph.h"
)
source_group("brepdb\\version" FILES ${brepdb__version})

set(brepdb__script
    "brepdb_c/brepdb.ves.inc"
    "brepdb_c/wrap_Brepdb.cpp"
    "brepdb_c/wrap_Brepdb.h"
)
source_group("brepdb\\script" FILES ${brepdb__script})

set(deepbrep__core
    "deepbrep_c/Tensor.cpp"
    "deepbrep_c/Tensor.h"
    "deepbrep_c/GraphData.cpp"
    "deepbrep_c/GraphData.h"
    "deepbrep_c/Layers.cpp"
    "deepbrep_c/Layers.h"
    "deepbrep_c/GNNModel.cpp"
    "deepbrep_c/GNNModel.h"
    "deepbrep_c/Trainer.cpp"
    "deepbrep_c/Trainer.h"
    "deepbrep_c/FeatureLabels.cpp"
    "deepbrep_c/FeatureLabels.h"
    "deepbrep_c/ToyDataset.cpp"
    "deepbrep_c/ToyDataset.h"
    "deepbrep_c/DatasetReader.cpp"
    "deepbrep_c/DatasetReader.h"
)
source_group("deepbrep\\core" FILES ${deepbrep__core})

set(deepbrep__brep
    "deepbrep_c/BRepGraphBuilder.cpp"
    "deepbrep_c/BRepGraphBuilder.h"
)
source_group("deepbrep\\brep" FILES ${deepbrep__brep})

set(deepbrep__script
    "deepbrep_c/deepbrep.ves.inc"
    "deepbrep_c/wrap_DeepBrep.cpp"
    "deepbrep_c/wrap_DeepBrep.h"
)
source_group("deepbrep\\script" FILES ${deepbrep__script})

set(deepbrep__datagen
    "deepbrep_data_gen/OpNameVocabulary.cpp"
    "deepbrep_data_gen/OpNameVocabulary.h"
    "deepbrep_data_gen/HistoryGraphLabeler.cpp"
    "deepbrep_data_gen/HistoryGraphLabeler.h"
    "deepbrep_data_gen/DatasetWriter.cpp"
    "deepbrep_data_gen/DatasetWriter.h"
)
source_group("deepbrep\\datagen" FILES ${deepbrep__datagen})


# ---- cadapp: CAD application layer (IR + replay + name resolve) ----
#
# Sits between breptopo (kernel-agnostic computation graph) and
# cadcvt (external-format readers). Owns the FeatureIR / SketchIR
# data model, the Replayer that turns a DocumentIR into a CompGraph
# + history, the TopoRefResolver, the binary stores, and the
# sketch-related ops registered on breptopo's OpRegistry.

set(cadapp__ir
    "cadapp_c/ir/Enums.h"
    "cadapp_c/ir/SketchIR.h"
    "cadapp_c/ir/TopoRefIR.h"
    "cadapp_c/ir/FeatureIR.h"
)
source_group("cadapp\\ir" FILES ${cadapp__ir})

set(cadapp__store
    "cadapp_c/store/SketchStore.cpp"
    "cadapp_c/store/SketchStore.h"
    "cadapp_c/store/FeatureStore.cpp"
    "cadapp_c/store/FeatureStore.h"
    "cadapp_c/store/SketchBridge.cpp"
    "cadapp_c/store/SketchBridge.h"
)
source_group("cadapp\\store" FILES ${cadapp__store})

set(cadapp__resolve
    "cadapp_c/resolve/TopoRefResolver.cpp"
    "cadapp_c/resolve/TopoRefResolver.h"
)
source_group("cadapp\\resolve" FILES ${cadapp__resolve})

set(cadapp__emitter
    "cadapp_c/emitter/Replayer.cpp"
    "cadapp_c/emitter/Replayer.h"
)
source_group("cadapp\\emitter" FILES ${cadapp__emitter})

set(cadapp__ops
    "cadapp_c/ops/sketch_ops.cpp"
    "cadapp_c/ops/sketch_ops.h"
)
source_group("cadapp\\ops" FILES ${cadapp__ops})


# ---- cadcvt: CAD inter-format converter ----
#
# External-format readers (FreeCAD .FCStd, ...) that produce
# cadapp::DocumentIR, plus the FreeCadLoader Ves binding.

set(cadcvt__reader
    "cadcvt_c/reader/Reader.h"
)
source_group("cadcvt\\reader" FILES ${cadcvt__reader})

set(cadcvt__script
    "cadcvt_c/cadcvt.ves.inc"
    "cadcvt_c/wrap_CadCvt.cpp"
    "cadcvt_c/wrap_CadCvt.h"
)
source_group("cadcvt\\script" FILES ${cadcvt__script})

# FreeCadReader needs miniz (zip) + pugixml (xml). Both are git
# submodules; the gate below catches "user forgot to run
# git submodule update --init" and skips this reader with a clean
# status message instead of a compile error.
set(CADCVT_FREECAD_DEPS_OK FALSE)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/miniz/miniz.h" AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/miniz/miniz.c" AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pugixml/src/pugixml.hpp" AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pugixml/src/pugixml.cpp")
    set(CADCVT_FREECAD_DEPS_OK TRUE)
endif()

if(CADCVT_FREECAD_DEPS_OK)
    # miniz.h #include's miniz_export.h, which is normally produced
    # by miniz's own CMake via generate_export_header(miniz). We
    # consume miniz.c as raw source inside cax, so the file is
    # never generated. Write a one-line stub at configure time
    # (MINIZ_EXPORT is empty for static library builds).
    set(CADCVT_MINIZ_EXPORT_DIR "${CMAKE_CURRENT_BINARY_DIR}/cadcvt_generated")
    file(WRITE "${CADCVT_MINIZ_EXPORT_DIR}/miniz_export.h"
"#ifndef MINIZ_EXPORT_H\n"
"#define MINIZ_EXPORT_H\n"
"#define MINIZ_EXPORT\n"
"#define MINIZ_NO_EXPORT\n"
"#endif\n")

    set(cadcvt__reader_freecad
        "cadcvt_c/reader/FreeCadReader.cpp"
        "cadcvt_c/reader/FreeCadReader.h"
        "thirdparty/miniz/miniz.c"
        "thirdparty/miniz/miniz.h"
        "thirdparty/pugixml/src/pugixml.cpp"
        "thirdparty/pugixml/src/pugixml.hpp"
    )
    source_group("cadcvt\\reader\\freecad" FILES ${cadcvt__reader_freecad})
    message(STATUS "cadcvt: FreeCadReader enabled")
else()
    set(cadcvt__reader_freecad "")
    set(CADCVT_MINIZ_EXPORT_DIR "")
    message(STATUS "cadcvt: FreeCadReader disabled (run: git submodule update --init thirdparty/miniz thirdparty/pugixml)")
endif()


set(ALL_FILES
    ${breptopo}
    ${breptopo__computation}
    ${breptopo__history}
    ${breptopo__history__viz}
    ${breptopo__history__comp}
    ${breptopo__script}
    ${breptopo__tools__comp}
    ${breptopo__topology}
    ${breptopo__topology__comp}
    ${nurbslib}
    ${partgraph__brep}
    ${partgraph__script}
    ${partgraph__tools}
    ${sketchlib__data}
    ${sketchlib__external__PlaneGCS}
    ${sketchlib__script}
    ${brepdb}
    ${brepdb__geom}
    ${brepdb__topo}
    ${brepdb__index}
    ${brepdb__version}
    ${brepdb__script}
    ${deepbrep__core}
    ${deepbrep__brep}
    ${deepbrep__script}
    ${deepbrep__datagen}
    ${cadapp__ir}
    ${cadapp__store}
    ${cadapp__resolve}
    ${cadapp__emitter}
    ${cadapp__ops}
    ${cadcvt__reader}
    ${cadcvt__script}
    ${cadcvt__reader_freecad}
)

################################################################################
# Target
################################################################################

if(NOT BOOST_ROOT)
    set(BOOST_ROOT "" CACHE PATH "Root directory of Boost installation")
endif()
if(BOOST_ROOT)
    set(Boost_INCLUDE_DIR "${BOOST_ROOT}/include")
    set(Boost_LIBRARY_DIRS "${BOOST_ROOT}/lib")
endif()

find_package(Boost REQUIRED COMPONENTS graph)

if(Boost_FOUND)
    message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")
endif()

if(NOT OCCT_DIR)
    set(OCCT_DIR "" CACHE PATH "Root directory of OpenCASCADE installation")
endif()
if(OCCT_DIR)
    list(APPEND CMAKE_PREFIX_PATH "${OCCT_DIR}")
endif()
find_package(OpenCASCADE REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_library(${PROJECT_NAME} STATIC ${ALL_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE . ${OpenCASCADE_INCLUDE_DIR})
if(TARGET sm)
    target_link_libraries(${PROJECT_NAME} PRIVATE sm)
else()
    target_include_directories(${PROJECT_NAME} PRIVATE thirdparty thirdparty/sm)
endif()
if(TARGET objcomp)
    target_link_libraries(${PROJECT_NAME} PRIVATE objcomp)
else()
    target_include_directories(${PROJECT_NAME} PRIVATE thirdparty/objcomp/include)
endif()
if(TARGET vessel)
    target_link_libraries(${PROJECT_NAME} PRIVATE vessel)
else()
    target_include_directories(${PROJECT_NAME} PRIVATE thirdparty/vessel/src/include)
endif()
if(TARGET wrapper)
    target_link_libraries(${PROJECT_NAME} PRIVATE wrapper)
else()
    target_include_directories(${PROJECT_NAME} PRIVATE thirdparty/wrapper/include)
endif()
if(TARGET graph)
    target_link_libraries(${PROJECT_NAME} PRIVATE graph)
else()
    target_include_directories(${PROJECT_NAME} PRIVATE thirdparty/graph/include)
endif()
if(TARGET guard)
    target_link_libraries(${PROJECT_NAME} PRIVATE guard)
else()
    target_include_directories(${PROJECT_NAME} PRIVATE thirdparty/guard/include)
endif()
if(TARGET geoshape)
    target_link_libraries(${PROJECT_NAME} PRIVATE geoshape)
else()
    target_include_directories(${PROJECT_NAME} PRIVATE thirdparty/geoshape/include)
endif()
if(TARGET unirender)
    target_link_libraries(${PROJECT_NAME} PRIVATE unirender)
else()
    target_include_directories(${PROJECT_NAME} PRIVATE thirdparty/unirender/include)
endif()
if(TARGET spatialdb)
    target_link_libraries(${PROJECT_NAME} PRIVATE spatialdb)
else()
    target_include_directories(${PROJECT_NAME} PRIVATE thirdparty/spatialdb/include)
endif()
if(TARGET nurbs)
    target_link_libraries(${PROJECT_NAME} PRIVATE nurbs)
else()
    target_include_directories(${PROJECT_NAME} PRIVATE thirdparty/nurbs/include)
endif()
if(TARGET logger)
    target_link_libraries(${PROJECT_NAME} PRIVATE logger)
endif()
target_include_directories(${PROJECT_NAME} PRIVATE thirdparty)
if(CADCVT_FREECAD_DEPS_OK)
    target_include_directories(${PROJECT_NAME} PRIVATE
        thirdparty/miniz
        thirdparty/pugixml/src
        "${CADCVT_MINIZ_EXPORT_DIR}")
endif()
if(TARGET Eigen3::Eigen)
    target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
else()
    target_include_directories(${PROJECT_NAME} PRIVATE thirdparty/eigen)
endif()

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)

if(NOT TARGET spatialdb)
    add_subdirectory(thirdparty/spatialdb)
endif()

################################################################################
# Unit tests
################################################################################
option(CAX_BUILD_TESTS "Build unit tests" ON)

if(CAX_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test/brepdb_c)
    add_subdirectory(test/brepdb_occt_c)
    add_subdirectory(test/breptopo_c)
    add_subdirectory(test/deepbrep_c)
endif()

################################################################################
# Optional demo executables
################################################################################
option(CAX_BUILD_DEEPBREP_DEMO "Build deepbrep toy training demo" ON)

if(CAX_BUILD_DEEPBREP_DEMO)
    add_executable(deepbrep_demo
        deepbrep_c/demo_main.cpp
        deepbrep_c/Tensor.cpp
        deepbrep_c/GraphData.cpp
        deepbrep_c/Layers.cpp
        deepbrep_c/GNNModel.cpp
        deepbrep_c/Trainer.cpp
        deepbrep_c/FeatureLabels.cpp
        deepbrep_c/ToyDataset.cpp
    )
    target_include_directories(deepbrep_demo PRIVATE .)
    target_compile_features(deepbrep_demo PRIVATE cxx_std_17)
endif()

option(CAX_BUILD_DEEPBREP_TRAIN "Build deepbrep training tool" ON)

if(CAX_BUILD_DEEPBREP_TRAIN)
    add_executable(deepbrep_train
        deepbrep_c/train_main.cpp
        deepbrep_c/DatasetReader.cpp
        deepbrep_c/Tensor.cpp
        deepbrep_c/GraphData.cpp
        deepbrep_c/Layers.cpp
        deepbrep_c/GNNModel.cpp
        deepbrep_c/Trainer.cpp
        deepbrep_c/FeatureLabels.cpp
    )
    target_include_directories(deepbrep_train PRIVATE .)
    target_compile_features(deepbrep_train PRIVATE cxx_std_17)
endif()

option(CAX_BUILD_DEEPBREP_DATAGEN "Build deepbrep dataset-generation tool" ON)

if(CAX_BUILD_DEEPBREP_DATAGEN)
    # Links against the main cax static lib so it can reuse OCCT/Boost wiring
    # already set up there (PrimMaker, TopoNaming, BRepGraphBuilder, ...).
    add_executable(deepbrep_data_gen
        deepbrep_data_gen/gen_dataset_main.cpp
    )
    target_include_directories(deepbrep_data_gen PRIVATE
        .
        ${OpenCASCADE_INCLUDE_DIR}
    )
    target_link_libraries(deepbrep_data_gen PRIVATE
        ${PROJECT_NAME}
        ${OpenCASCADE_LIBRARIES}
    )
    # cax propagates `graph` as a PRIVATE dep, and graph in turn has
    # unresolved ogdf symbols at its TUs. Even though this tool's own code
    # doesn't use graph or ogdf, PrimMaker.obj references WorldSender, which
    # references graph, which references ogdf -- so the static-archive walk
    # pulls them all into the link line. Until cax is restructured to peel
    # viz/io off the core, just link both here.
    if(TARGET graph)
        target_link_libraries(deepbrep_data_gen PRIVATE graph)
    endif()
    # OGDF is a vcpkg-installed lib (not a CMake target in this scope) --
    # link by raw name and let the linker find it via configured search
    # paths. Matches brepdb_occt_test/CMakeLists.txt:47.
    target_link_libraries(deepbrep_data_gen PRIVATE OGDF)
    target_compile_features(deepbrep_data_gen PRIVATE cxx_std_17)
endif()
