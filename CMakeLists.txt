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
    "breptopo_c/comp_nodes.cpp"
    "breptopo_c/comp_nodes.h"
    "breptopo_c/comp_variants.h"
    "breptopo_c/CompGraph.cpp"
    "breptopo_c/CompGraph.h"
    "breptopo_c/CompNode.h"
    "breptopo_c/CompVariant.h"
)
source_group("breptopo\\computation" FILES ${breptopo__computation})

set(breptopo__computation__comp
    "breptopo_c/NodeComp.h"
)
source_group("breptopo\\computation\\comp" FILES ${breptopo__computation__comp})

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

set(ALL_FILES
    ${breptopo}
    ${breptopo__computation}
    ${breptopo__computation__comp}
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
)

################################################################################
# Target
################################################################################

set(BOOST_ROOT "Y:/tools/vcpkg/installed/arm64-windows")
set(Boost_INCLUDE_DIR "${BOOST_ROOT}/include")
set(Boost_LIBRARY_DIRS "${BOOST_ROOT}/lib")

find_package(Boost REQUIRED COMPONENTS graph)

if(Boost_FOUND)
    message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")
endif()

set(OCCT_DIR "Y:/OCCT/install")
list(APPEND CMAKE_PREFIX_PATH "${OCCT_DIR}")
find_package(OpenCASCADE REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_library(${PROJECT_NAME} STATIC ${ALL_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE . thirdparty thirdparty/sm thirdparty/objcomp/include thirdparty/vessel/src/include thirdparty/wrapper/include thirdparty/graph/include thirdparty/guard/include thirdparty/geoshape/include thirdparty/nurbs/include thirdparty/unirender/include thirdparty/eigen ${OpenCASCADE_INCLUDE_DIR})
