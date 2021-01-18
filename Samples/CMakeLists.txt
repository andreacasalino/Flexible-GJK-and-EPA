option(BUILD_EXAMPLE_00 "Build Example_00" ON)
if(BUILD_EXAMPLE_00)
    add_executable(Sample00 Sample00.cpp Utils.h Utils.cpp)
    target_link_libraries(Sample00 PUBLIC flx::GJK-EPA)
endif()

option(BUILD_EXAMPLE_01 "Build Example_01" OFF)
if(BUILD_EXAMPLE_01)
    add_executable(Sample01 Sample01.cpp Utils.h Utils.cpp)
    target_link_libraries(Sample01 PUBLIC flx::GJK-EPA)	

    # add_custom_command(target Sample01 pre_build
	    # command ${cmake_command} -e copy ${cmake_current_source_dir}/ResultVisualizer.py ${cmake_current_binary_dir}/ResultVisualizer.py)
endif()

option(BUILD_EXAMPLE_02 "Build Example_02" OFF)
if(BUILD_EXAMPLE_02)
    add_executable(Sample02 Sample02.cpp Utils.h Utils.cpp)
    target_link_libraries(Sample02 PUBLIC flx::GJK-EPA)	
	
    # add_custom_command(target Sample02 pre_build
	    # command ${cmake_command} -e copy ${cmake_current_source_dir}/ResultVisualizer.py ${cmake_current_binary_dir}/ResultVisualizer.py)
endif()

option(BUILD_EXAMPLE_03 "Build Example_03" OFF)
if(BUILD_EXAMPLE_03)
    add_executable(Sample03 Sample03.cpp Utils.h Utils.cpp)
    target_link_libraries(Sample03 PUBLIC flx::GJK-EPA)	
	
    # add_custom_command(target Sample03 pre_build
	    # command ${cmake_command} -e copy ${cmake_current_source_dir}/ResultVisualizer.py ${cmake_current_binary_dir}/ResultVisualizer.py)
endif()
