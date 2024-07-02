cmake_minimum_required(VERSION 3.19)
project(fuzztest)

set(CMAKE_CXX_STANDARD 17)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set (COMPILER_GCC 1)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "AppleClang")
  set (COMPILER_CLANG 1) # Safe to treat AppleClang as a regular Clang, in general.
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set (COMPILER_CLANG 1)
else ()
  #message (FATAL_ERROR "Compiler ${CMAKE_CXX_COMPILER_ID} is not supported")
endif ()

include(cmake/AddFuzzTest.cmake)
include(cmake/FuzzTestFlagSetup.cmake)

fuzztest_setup_fuzzing_flags()

include(cmake/BuildDependencies.cmake)
include(cmake/FuzzTestHelpers.cmake)

include_directories(${re2_SOURCE_DIR})

add_subdirectory(fuzztest)
