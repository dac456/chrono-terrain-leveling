cmake_minimum_required(VERSION 2.8)

project(chrono-terrain-leveling)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g -ggdb -std=c++11")

find_package(Chrono
             COMPONENTS Irrlicht Parallel
             CONFIG)
             
include_directories(${CHRONO_INCLUDE_DIRS})

file(GLOB_RECURSE HDR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)
file(GLOB_RECURSE SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(ctl ${SRC})

target_link_libraries(ctl ${CHRONO_LIBRARIES})