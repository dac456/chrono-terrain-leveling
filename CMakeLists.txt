cmake_minimum_required(VERSION 2.8)

project(chrono-terrain-leveling)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules")

if(USE_CUDA)
    find_package(Chrono REQUIRED
                 COMPONENTS Irrlicht Parallel
                 CONFIG)
    add_definitions( -DSIM_USE_CUDA )
else(USE_CUDA)
    find_package(Chrono REQUIRED
                 COMPONENTS Irrlicht
                 CONFIG)
endif(USE_CUDA)

find_package(Assimp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/RapidXML)
include_directories(${CHRONO_INCLUDE_DIRS})
include_directories(${ASSIMP_INCLUDE_DIR})
include_directories(${BOOST_INCLUDE_DIR})

file(GLOB_RECURSE HDR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)
file(GLOB_RECURSE SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(ctl ${SRC} ${HDR})

target_link_libraries(ctl ${Boost_LIBRARIES} ${CHRONO_LIBRARIES} ${ASSIMP_LIBRARY})
