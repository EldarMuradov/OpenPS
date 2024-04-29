cmake_minimum_required(VERSION 3.10)

set(ProjectName OpenPS)

project(${ProjectName} VERSION 1.0)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(WIN32)
  set(CMAKE_USE_RELATIVE_PATHS true)
  set(CMAKE_SUPPRESS_REGENERATION true)
  set(CMAKE_DEBUG_POSTFIX "_d")
endif()

set(Sources 
include/openps/openps.h
include/openps/openps_decl.h
include/openps/memory/ememory.h
include/openps/ecs/px_colliders.h
include/openps/ecs/px_rigidbody.h
include/openps/core/px_aggregates.h
include/openps/core/px_gjk_support.h
include/openps/core/px_logger.h
include/openps/core/px_physics.h
include/openps/core/px_structs.h
include/openps/core/px_tasks.h
include/openps/core/px_wrappers.h
src/memory/ememory.cpp
src/core/px_wrappers.cpp
src/core/px_aggregates.cpp
src/core/px_physics.cpp
src/core/px_gjk_support.cpp
src/ecs/px_colliders.cpp
src/ecs/px_rigidbody.cpp)

add_library(OpenPS STATIC ${Sources})

include_directories(include/openps src)

add_executable(Example example.cpp)

if (MSVC)
    add_compile_options(/W)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PHYSX_INCLUDE_DIRS ext/PhysX/physx ext/CUDA/include)

# ext/bin/Release/PhysXGpu_64.dll  is too large to keep it in repo

set(PHYSX_BIN_RELEASE_PATHES ext/bin/Release/PhysX_64.dll ext/bin/Release/PhysXGpu_64.dll ext/bin/Release/PhysXFoundation_64.dll ext/bin/Release/PhysXCooking_64.dll ext/bin/Release/PhysXCommon_64.dll ext/bin/Release/PhysXDevice64.dll)
set(PHYSX_BIN_DEBUG_PATHES ext/bin/Debug/PhysX_64.dll ext/bin/Debug/PhysXGpu_64.dll ext/bin/Debug/PhysXFoundation_64.dll ext/bin/Debug/PhysXCooking_64.dll ext/bin/Debug/PhysXCommon_64.dll ext/bin/Debug/PhysXDevice64.dll)

set(OPENPS_LIB_DEBUG_PATH debug Debug/OpenPS_d)
set(OPENPS_LIB_RELEASE_PATH optimized Release/OpenPS)

set(PHYSX_LIB_RELEASE_PATHES 
optimized ../ext/libs/Release/PhysX_64 
optimized ../ext/libs/Release/PhysXExtensions_static_64 
optimized ../ext/libs/Release/PhysXPvdSDK_static_64 
optimized ../ext/libs/Release/PhysXCooking_64 
optimized ../ext/libs/Release/PhysXCommon_64 
optimized ../ext/libs/Release/PhysXFoundation_64 
../ext/CUDA/lib/x64/cuda)

set(PHYSX_LIB_DEBUG_PATHES 
debug ../ext/libs/Debug/PhysX_64 
debug ../ext/libs/Debug/PhysXExtensions_static_64 
debug ../ext/libs/Debug/PhysXPvdSDK_static_64 
debug ../ext/libs/Debug/PhysXCooking_64 
debug ../ext/libs/Debug/PhysXCommon_64 
debug ../ext/libs/Debug/PhysXFoundation_64 
../ext/CUDA/lib/x64/cuda)

target_include_directories(OpenPS PUBLIC ${PHYSX_INCLUDE_DIRS})

target_include_directories(Example PUBLIC include/openps ${PHYSX_INCLUDE_DIRS})

target_link_libraries(OpenPS ${PHYSX_LIB_DEBUG_PATHES})
target_link_libraries(OpenPS ${PHYSX_LIB_RELEASE_PATHES})

target_link_libraries(Example ${PHYSX_LIB_DEBUG_PATHES})
target_link_libraries(Example ${PHYSX_LIB_RELEASE_PATHES})

target_link_libraries(Example ${OPENPS_LIB_RELEASE_PATH})
target_link_libraries(Example ${OPENPS_LIB_DEBUG_PATH})

file(COPY ${PHYSX_BIN_RELEASE_PATHES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release)
file(COPY ${PHYSX_BIN_DEBUG_PATHES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug)