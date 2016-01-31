cmake_minimum_required(VERSION 2.8)

project(chrono-terrain-leveling)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules")

if(USE_IRRLICHT)
    add_definitions( -DSIM_USE_IRRLICHT )
    if(USE_CUDA)
        find_package(Chrono REQUIRED
                     COMPONENTS Irrlicht Parallel PostProcess
                     CONFIG)
        add_definitions( -DSIM_USE_CUDA )
    else(USE_CUDA)
        find_package(Chrono REQUIRED
                     COMPONENTS Irrlicht PostProcess
                     CONFIG)
    endif(USE_CUDA)
else(USE_IRRLICHT)
if(USE_CUDA)
    find_package(Chrono REQUIRED
                 COMPONENTS Parallel PostProcess
                 CONFIG)
    add_definitions( -DSIM_USE_CUDA )
else(USE_CUDA)
    find_package(Chrono REQUIRED
                 COMPONENTS PostProcess
                 CONFIG)
endif(USE_CUDA)
endif(USE_IRRLICHT)

add_definitions( -DASSIMP_BUILD_NO_C4D_IMPORTER )
add_definitions( -DASSIMP_BUILD_NO_EXPORT )
add_definitions( -DASSIMP_BUILD_NO_OWN_ZLIB )

find_package(ZLIB REQUIRED)
#find_package(Assimp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/RapidXML)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/stb)
include_directories(${CHRONO_INCLUDE_DIRS})
#include_directories(${ASSIMP_INCLUDE_DIR})
include_directories(${ZLIB_INCLUDE_DIR})
include_directories(${BOOST_INCLUDE_DIR})

file(GLOB_RECURSE HDR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)
file(GLOB_RECURSE SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
    src/*.cpp
    src/assimp/*.cpp
    src/openddlparser/*.cpp
    src/assimp/contrib/*/*.cpp
    src/assimp/contrib/*/*.c
    src/assimp/contrib/poly2tri/poly2tri/*/*.cc
)

add_executable(ctl ${SRC} ${HDR})

target_link_libraries(ctl ${Boost_LIBRARIES} ${CHRONO_LIBRARIES} -lz)
