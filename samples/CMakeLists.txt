message(STATUS "fetching nlohmann json for logging samples results")
include(FetchContent)
FetchContent_Declare(
	nlohmann_json
	GIT_REPOSITORY https://github.com/nlohmann/json.git
	GIT_TAG        7194245a314d142a5560f6906a87f7c67ebbcf2e
)
FetchContent_MakeAvailable(nlohmann_json)

macro(COPY_SCRIPT TARGET SCRIPT)
    add_custom_command(TARGET ${TARGET} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${SCRIPT}.py ${CMAKE_CURRENT_BINARY_DIR}/${SCRIPT}.py)    
endmacro()

macro(MAKE_SAMPLE NAME)
    add_executable(${NAME} ${NAME}.cpp 
        Utils.h Utils.cpp
        Logger.h Logger.cpp
    )
    target_link_libraries(${NAME} PUBLIC 
        GJK-EPA
        nlohmann_json::nlohmann_json
    )

    COPY_SCRIPT(${NAME} Visualize)	
endmacro()

MAKE_SAMPLE(Readme)
MAKE_SAMPLE(Sample01)
MAKE_SAMPLE(Sample02)
MAKE_SAMPLE(Sample03)
MAKE_SAMPLE(Sample04)
