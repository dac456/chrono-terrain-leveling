cmake_minimum_required(VERSION 2.8)

project(chrono-terrain-leveling)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -fopenmp -O3 -std=c++11")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules")

MACRO(SOURCE_GROUP_BY_FOLDER target)
  SET(SOURCE_GROUP_DELIMITER "/")
  SET(last_dir "")
  SET(files "")
  FOREACH(file ${${target}_SRC} ${${target}_HEADERS})
    file(RELATIVE_PATH relative_file "${PROJECT_SOURCE_DIR}/${target}" ${file})
    GET_FILENAME_COMPONENT(dir "${relative_file}" PATH)
    IF (NOT "${dir}" STREQUAL "${last_dir}")
      IF (files)
        SOURCE_GROUP("${last_dir}" FILES ${files})
      ENDIF (files)
      SET(files "")
    ENDIF (NOT "${dir}" STREQUAL "${last_dir}")
    SET(files ${files} ${file})
    SET(last_dir "${dir}")
  ENDFOREACH(file)
  IF (files)
    SOURCE_GROUP("${last_dir}" FILES ${files})
  ENDIF (files)
ENDMACRO(SOURCE_GROUP_BY_FOLDER)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if(USE_IRRLICHT)
    add_definitions( -DSIM_USE_IRRLICHT )
    if(USE_PARALLEL)
        find_package(Chrono REQUIRED
                     COMPONENTS Irrlicht Parallel PostProcess
                     CONFIG)
        add_definitions( -DSIM_USE_PARALLEL )
    else(USE_PARALLEL)
        find_package(Chrono REQUIRED
                     COMPONENTS Irrlicht PostProcess
                     CONFIG)
    endif(USE_PARALLEL)
else(USE_IRRLICHT)
    if(USE_PARALLEL)
        find_package(Chrono REQUIRED
                     COMPONENTS Parallel PostProcess
                     CONFIG)
        add_definitions( -DSIM_USE_PARALLEL )
    else(USE_PARALLEL)
        find_package(Chrono REQUIRED
                     COMPONENTS PostProcess
                     CONFIG)
    endif(USE_PARALLEL)
endif(USE_IRRLICHT)

add_definitions( -DASSIMP_BUILD_NO_C4D_IMPORTER )
add_definitions( -DASSIMP_BUILD_NO_EXPORT )
add_definitions( -DASSIMP_BUILD_NO_OWN_ZLIB )
add_definitions( -DOPENDDLPARSER_BUILD )

if(WIN32)
    add_definitions( -D_CRT_SECURE_NO_WARNINGS )
endif(WIN32)


find_package(ZLIB REQUIRED)
#find_package(Assimp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem program_options)
message("Using Boost libraries: ${Boost_LIBRARIES}")

link_directories(${Boost_LIBRARY_DIRS})
if(USE_PARALLEL)
    if(USE_TBB)
        find_package(TBB)
        link_directories(${TBB_LIBRARY_DIR})
        include_directories(${TBB_INCLUDE_DIR})
        add_definitions( -DTHRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_TBB )

        set(USE_TBB_LIBRARIES "${TBB_LIBRARIES}")
    else(USE_TBB)
        add_definitions( -DTHRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_OMP )
        set(USE_TBB_LIBRARIES "")
    endif(USE_TBB)
endif(USE_PARALLEL)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/RapidXML)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/stb)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/contrib)
include_directories(${CHRONO_INCLUDE_DIRS})
include_directories(${ZLIB_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})

file(GLOB_RECURSE ctl_HEADERS ABSOLUTE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp include/*.h)
file(GLOB_RECURSE ctl_SRC ABSOLUTE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
    src/*.cpp
    src/assimp/*.cpp
    src/openddlparser/*.cpp
    src/assimp/contrib/*/*.cpp
    src/assimp/contrib/*/*.c
    src/assimp/contrib/poly2tri/poly2tri/*/*.cc
)

SOURCE_GROUP_BY_FOLDER(ctl)

add_executable(ctl ${ctl_SRC} ${ctl_HEADERS})

target_link_libraries(ctl ${Boost_LIBRARIES} ${CHRONO_LIBRARIES} ${ZLIB_LIBRARIES} ${USE_TBB_LIBRARIES})
