cmake_minimum_required(VERSION 2.8.3)
project(minho_team_pihw CXX)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   message_generation   
   genmsg   
)

catkin_package(
   #  INCLUDE_DIRS include
   #  LIBRARIES minho_team
   CATKIN_DEPENDS message_runtime std_msgs
)

set(PDIR ${minho_team_pihw_SOURCE_DIR})
message(STATUS "Current directory: " ${PDIR})
find_package(wiringPi REQUIRED)
find_package(Threads REQUIRED)
include_directories(${PDIR}/include ${WIRINGPI_INCLUDE_DIRS})
set(SDIR ${PDIR}/src)

set(MLIB_DIR ${PDIR}/libs)
link_directories(${MLIB_DIR})

## Compile mcp3k8 as a shared library
add_library(mcp3k8 SHARED ${SDIR}/MCP3k8/mcp3k8.cpp)
set_target_properties( mcp3k8
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${MLIB_DIR}
)
target_compile_features(mcp3k8 PRIVATE cxx_range_for)
##

## Compile Omni3MD as a shared library
add_library(Omni3MD SHARED ${SDIR}/Omni3MD/Omni3MD.cpp)
set_target_properties( Omni3MD
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${MLIB_DIR}
)
target_compile_features(Omni3MD PRIVATE cxx_range_for)
##

## Compile Ada10Dof as a shared library
add_library(Ada10Dof SHARED ${SDIR}/Ada10Dof/Ada10Dof.cpp)
set_target_properties( Ada10Dof
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${MLIB_DIR}
)
target_compile_features(Ada10Dof PRIVATE cxx_range_for)
##

## Compile program
set(SRC ${SDIR}/main.cpp)
message(STATUS "Source directory: " ${SDIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(~/catkin_ws/devel/include)
add_executable(main ${SRC})
target_link_libraries(main ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} mcp3k8 Omni3MD ${WIRINGPI_LIBRARIES})
add_dependencies(main Omni3MD mcp3k8 minho_team_ros_gencpp)
##
