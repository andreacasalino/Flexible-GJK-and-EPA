cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

option(LIB_OPT "Compile shared libraries (ON) or static (OFF)" OFF)
project(Flexible-GJK-and-EPA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FetchDependency.cmake)

option(GJK_EPA_DIAGNOSTIC "Diagnostic log enabled (ON) or disabled (OFF)" OFF)
option(BUILD_GJK_EPA_SAMPLES "Build the samples showing how to use the package" ON)
option(BUILD_GJK_EPA_TESTS "" OFF)

add_subdirectory(src)

if(BUILD_GJK_EPA_TESTS OR BUILD_GJK_EPA_SAMPLES)
    add_subdirectory(utils)
endif()

if(BUILD_GJK_EPA_SAMPLES)
    add_subdirectory(samples)
endif()

if(BUILD_GJK_EPA_TESTS)
    add_subdirectory(tests)
endif()
