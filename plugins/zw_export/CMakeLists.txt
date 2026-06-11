# ============================================================
# plugins/zw_export/CMakeLists.txt
#
# Standalone build for the ZW3D export plugin (CaxExport.dll).
#
# This is DELIBERATELY its own project, NOT add_subdirectory'd by the
# top-level cax CMakeLists. The plugin links the ZW3D SDK (ZW3D.lib),
# is Windows-only, runs inside ZW3D, and must match ZW3D's MSVC runtime
# (/MD) -- none of which belongs in cax's dependency graph (cax builds
# and ships on machines with no ZW3D installed). The two halves share
# exactly one thing: the wire-format header interop/CaxIntermediateSchema.h.
#
# Configure it separately, on a Windows node that has the ZW3D SDK:
#
#   cmake -S plugins/zw_export -B build/zw_export -A x64 ^
#         -DCAX_ZW_SDK_DIR="C:/Program Files/ZWSOFT/ZW3D 2026/api" ^
#         -DCAX_NLOHMANN_DIR="<cax>/thirdparty/nlohmann/single_include"
#   cmake --build build/zw_export --config Release
#
# Output: CaxExport.dll. Load it in ZW3D (Application -> Applications and
# Plugin Manager -> Load Dll, or drop it in the apilibs folder), then run
# the ~CaxExportRun command on the active part to write <part>.cax.json.
# ============================================================

cmake_minimum_required(VERSION 3.16)
project(zw_export LANGUAGES CXX)

if(NOT WIN32)
    message(FATAL_ERROR "zw_export is a ZW3D plugin and only builds on Windows.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# cax tree root (two levels up): holds interop/ and thirdparty/.
get_filename_component(CAX_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# ---- ZW3D SDK ----
# CAX_ZW_SDK_DIR is the ZW3D INSTALL ROOT. Verified layout (ZW3D 2026):
#   <root>/api/inc/VxApi.h , zwapi_*.h   (headers)
#   <root>/ZW3D.lib                      (import lib, at the root)
#   <root>/apilibs/                      (auto-load plugin folder)
# The default is a stock install path; override on the command line, e.g.
# for the extracted SDK used during bring-up:
#   -DCAX_ZW_SDK_DIR="C:/Users/<you>/Downloads/ZW3D-.../Productionx64"
set(CAX_ZW_SDK_DIR "C:/Program Files/ZWSOFT/ZW3D 2026"
    CACHE PATH "ZW3D install root (has api/inc and ZW3D.lib)")

find_path(ZW_INCLUDE_DIR
    NAMES VxApi.h
    HINTS "${CAX_ZW_SDK_DIR}/api/inc" "${CAX_ZW_SDK_DIR}/api" "${CAX_ZW_SDK_DIR}"
    DOC   "Directory containing VxApi.h (ZW3D <root>/api/inc)")

find_library(ZW_IMPORT_LIB
    NAMES ZW3D zw3d
    HINTS "${CAX_ZW_SDK_DIR}" "${CAX_ZW_SDK_DIR}/lib"
    DOC   "ZW3D import library (ZW3D.lib, at the install root)")

if(NOT ZW_INCLUDE_DIR OR NOT ZW_IMPORT_LIB)
    message(WARNING
        "zw_export: ZW3D SDK not found under CAX_ZW_SDK_DIR='${CAX_ZW_SDK_DIR}'. "
        "Set -DCAX_ZW_SDK_DIR=<ZW3D install root> (the folder with api/inc and "
        "ZW3D.lib). Skipping the plugin target.")
    return()
endif()

# Plugin auto-load folder, for the optional install() rule below.
set(ZW_APILIBS_DIR "${CAX_ZW_SDK_DIR}/apilibs"
    CACHE PATH "ZW3D apilibs folder (drop CaxExport.dll here to auto-load)")

# ---- nlohmann/json (header-only) ----
# Reuse the same submodule cax's ZwReader uses. Override CAX_NLOHMANN_DIR
# if the header lives elsewhere on this node.
set(CAX_NLOHMANN_DIR "${CAX_ROOT}/thirdparty/nlohmann/single_include"
    CACHE PATH "Directory whose nlohmann/json.hpp can be #included")

if(NOT EXISTS "${CAX_NLOHMANN_DIR}/nlohmann/json.hpp")
    message(WARNING
        "zw_export: nlohmann/json.hpp not found under '${CAX_NLOHMANN_DIR}'. "
        "Run 'git submodule update --init thirdparty/nlohmann' in the cax tree, "
        "or set -DCAX_NLOHMANN_DIR. Skipping the plugin target.")
    return()
endif()

# ---- the plugin DLL ----
# MODULE: a plugin loaded at runtime, never linked against. The output
# base name drives ZW3D's entry-point lookup: a "CaxExport.dll" makes
# ZW3D call the exported "CaxExportInit" / "CaxExportExit" (base name +
# Init/Exit). So force the name and drop the platform "lib" prefix.
add_library(CaxExport MODULE
    ZwCaxPlugin.cpp
    ZwCaxExport.cpp
)

set_target_properties(CaxExport PROPERTIES
    OUTPUT_NAME "CaxExport"
    PREFIX ""
)

target_include_directories(CaxExport PRIVATE
    "${CAX_ROOT}"          # interop/CaxIntermediateSchema.h
    "${CAX_NLOHMANN_DIR}"  # nlohmann/json.hpp
    "${ZW_INCLUDE_DIR}"    # VxApi.h / zwapi_*.h
)

target_link_libraries(CaxExport PRIVATE "${ZW_IMPORT_LIB}")

# std::fopen in the exporter trips MSVC's _CRT_INSECURE_DEPRECATE.
if(MSVC)
    target_compile_definitions(CaxExport PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# Match ZW3D's MSVC runtime (/MD). Mismatched runtimes make the DLL fail
# to load inside ZW3D.
if(MSVC)
    set_property(TARGET CaxExport PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
endif()

# ---- CaxBatch.dll: same plugin, alternate DLL/entry name ----
# ZW3D derives the entry points from the DLL base name, and Windows
# locks a loaded DLL's file. When a long-lived interactive ZW3D holds
# CaxExport.dll open, this identically-featured twin (CaxBatchInit /
# CaxBatchExit) can still be installed fresh and loaded by a second
# ZW3D instance -- which is how the unattended CAX_BATCH_QUEUE batch
# runs without disturbing the user's session. Same sources, only the
# exported entry names differ.
add_library(CaxBatch MODULE
    ZwCaxPlugin.cpp
    ZwCaxExport.cpp
)

set_target_properties(CaxBatch PROPERTIES
    OUTPUT_NAME "CaxBatch"
    PREFIX ""
)

target_compile_definitions(CaxBatch PRIVATE
    CAX_PLUGIN_INIT=CaxBatchInit
    CAX_PLUGIN_EXIT=CaxBatchExit
)

target_include_directories(CaxBatch PRIVATE
    "${CAX_ROOT}"
    "${CAX_NLOHMANN_DIR}"
    "${ZW_INCLUDE_DIR}"
)

target_link_libraries(CaxBatch PRIVATE "${ZW_IMPORT_LIB}")

if(MSVC)
    target_compile_definitions(CaxBatch PRIVATE _CRT_SECURE_NO_WARNINGS)
    set_property(TARGET CaxBatch PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
endif()

# Convenience: `cmake --install` drops CaxExport.dll into ZW3D's apilibs
# folder so it auto-loads on the next ZW3D start. Skipped if that folder
# is not writable / not present. CaxBatch is deliberately NOT installed:
# auto-loading both twins would double-register every command and fire
# the batch autostart twice; copy it in by hand for a one-off batch and
# remove it after.
if(EXISTS "${ZW_APILIBS_DIR}")
    install(TARGETS CaxExport RUNTIME DESTINATION "${ZW_APILIBS_DIR}"
                              LIBRARY DESTINATION "${ZW_APILIBS_DIR}")
endif()

message(STATUS "zw_export: CaxExport plugin enabled (ZW3D SDK: ${ZW_INCLUDE_DIR})")
