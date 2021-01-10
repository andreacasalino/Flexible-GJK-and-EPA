option(BUILD_EXAMPLE_00 "Build Example_00" ON)
if(BUILD_EXAMPLE_00)
    add_executable(Sample00 Sample00.cpp)
    target_link_libraries(Sample00 PUBLIC flx::GJK-EPA)
endif()

option(BUILD_EXAMPLE_01 "Build Example_01" ON)
if(BUILD_EXAMPLE_01)
    add_executable(Sample01 Sample01.cpp)
    target_link_libraries(Sample01 PUBLIC flx::GJK-EPA)	

#    add_custom_command(TARGET Example_01 PRE_BUILD
#	    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Ex_01/XML_example_01.xml ${CMAKE_CURRENT_BINARY_DIR}/
# XML_example_01.xml)
endif()

option(BUILD_EXAMPLE_02 "Build Example_02" ON)
if(BUILD_EXAMPLE_02)
    add_executable(Sample02 Sample02.cpp)
    target_link_libraries(Sample02 PUBLIC flx::GJK-EPA)	
endif()

option(BUILD_EXAMPLE_03 "Build Example_03" ON)
if(BUILD_EXAMPLE_03)
    add_executable(Sample03 Sample03.cpp)
    target_link_libraries(Sample03 PUBLIC flx::GJK-EPA)	
endif()
