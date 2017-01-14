cmake_minimum_required(VERSION 2.8.3)
project(minho_team_pihw CXX)

set(PDIR ${minho_team_pihw_SOURCE_DIR})
message(STATUS "Current directory: " ${PDIR})
find_package(wiringPi REQUIRED)
find_package(Threads REQUIRED)
include_directories(${PDIR}/include ${WIRINPI_INCLUDE_DIRECTORIES})
set(SDIR ${PDIR}/src)

set(MLIB_DIR ${PDIR}/libs)
link_directories(${MLIB_DIR})

## Compile mcp3k8 as a shared library
add_library(mcp3k8 SHARED ${SDIR}/mcp3k8.cpp)
set_target_properties( mcp3k8
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${MLIB_DIR}
)
target_compile_features(Omni3MD PRIVATE cxx_range_for)
##

## Compile Omni3MD as a shared library
add_library(Omni3MD SHARED ${SDIR}/Omni3MD.cpp)
set_target_properties( Omni3MD
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${MLIB_DIR}
)
target_compile_features(Omni3MD PRIVATE cxx_range_for)
##





## Compile program
set(SRC ${SDIR}/main.c)
message(STATUS "Source directory: " ${SDIR})
add_executable(main ${SRC})
target_link_libraries(main wiringPi ${CMAKE_THREAD_LIBS_INIT} mcp3k8 Omni3MD)
##
