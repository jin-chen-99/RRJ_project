cmake_minimum_required(VERSION 3.10)
project(untitled)

set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES main.cpp )
set(LIB lib/catch.hpp  test.cpp)
include_directories(lib)
add_executable(DateConverter_run ${SOURCE_FILES})
add_executable(unit_test ${LIB} )
