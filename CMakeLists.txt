cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

option(LIB_OPT "Compile shared libraries (ON) or static (OFF)" OFF)
project(Flexible-GJK-and-EPA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# include cmake custom functions
set(WITH_SOURCE_TREE ON)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/GroupSources.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/AutoCollect.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/MakeLibrary.cmake)

option(GJK_EPA_DIAGNOSTIC "Diagnostic log enabled (ON) or disabled (OFF)" OFF)
add_subdirectory(src)

option(BUILD_GJK_EPA_SAMPLES "Build the samples showing how to use the package" ON)
if(BUILD_GJK_EPA_SAMPLES)
    project(Flexible-GJK-and-EPA-Samples)
    add_subdirectory(samples)
endif()

option(BUILD_GJK_EPA_TESTS "" OFF)
if(BUILD_GJK_EPA_TESTS)
    project(Flexible-GJK-and-EPA-Tests)
    add_subdirectory(tests)
endif()
