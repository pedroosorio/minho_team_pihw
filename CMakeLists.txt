cmake_minimum_required(VERSION 2.8.3)
project(test C CXX)


set(PDIR ${test_SOURCE_DIR})
message(STATUS "Current directory: " ${PDIR})
find_package(wiringPi REQUIRED)
find_package(Threads REQUIRED)
include_directories(${PDIR}/include ${WIRINPI_INCLUDE_DIRECTORIES})
set(SDIR ${PDIR}/src)
set(SRC ${SDIR}/main.c ${SDIR}/mcp3008.c)
set(CMAKE_C_FLAGS "-std=c11")
message(STATUS "Source directory: " ${SDIR})
add_executable(main ${SRC})
target_link_libraries(main wiringPi ${CMAKE_THREAD_LIBS_INIT})