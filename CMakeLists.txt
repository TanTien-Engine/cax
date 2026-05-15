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

set(ALL_FILES
    ${breptopo}
    ${breptopo__computation}
    ${breptopo__history}
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
    add_subdirectory(brepdb_test)
    add_subdirectory(brepdb_occt_test)
    add_subdirectory(breptopo_test)
endif()
