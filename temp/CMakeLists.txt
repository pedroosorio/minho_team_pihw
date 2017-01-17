cmake_minimum_required(VERSION 2.8.3)
project(timers CXX)

set(PDIR ${timers_SOURCE_DIR})
message(STATUS "Current directory: " ${PDIR})
find_package(Threads REQUIRED)

## Compile program
set(SRC main.cpp)
message(STATUS "Source directory: " ${SDIR})
add_executable(main ${SRC})
target_link_libraries(main ${CMAKE_THREAD_LIBS_INIT})
##
