option(BUILD_EXAMPLE_01 "Build Example_01" ON)
if(BUILD_EXAMPLE_01)
    add_executable(Sample01 Sample01.cpp Utils.h Utils.cpp)
    target_link_libraries(Sample01 PUBLIC GJK-EPA)	

    add_custom_command(TARGET Sample01 PRE_BUILD
	    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Plotter.py ${CMAKE_CURRENT_BINARY_DIR}/Plotter.py)
endif()

option(BUILD_EXAMPLE_02 "Build Example_02" ON)
if(BUILD_EXAMPLE_02)
    add_executable(Sample02 Sample02.cpp Utils.h Utils.cpp)
    target_link_libraries(Sample02 PUBLIC GJK-EPA)	
	
    add_custom_command(TARGET Sample02 PRE_BUILD
	    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Plotter.py ${CMAKE_CURRENT_BINARY_DIR}/Plotter.py)
endif()

option(BUILD_EXAMPLE_03 "Build Example_03" ON)
if(BUILD_EXAMPLE_03)
    add_executable(Sample03 Sample03.cpp Utils.h Utils.cpp)
    target_link_libraries(Sample03 PUBLIC GJK-EPA)	
	
    add_custom_command(TARGET Sample03 PRE_BUILD
	    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Plotter.py ${CMAKE_CURRENT_BINARY_DIR}/Plotter.py)
endif()
