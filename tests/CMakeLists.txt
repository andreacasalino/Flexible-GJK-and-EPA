include(FetchContent)
FetchContent_Declare(
	catch2
	GIT_REPOSITORY https://github.com/catchorg/Catch2.git
	GIT_TAG        4ff8b27bb6bed4b8b309e56cd269b4f1fbc24e89
)
FetchContent_MakeAvailable(catch2)
	
set(TEST_NAME Tests)

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

add_executable(${TEST_NAME} ${SOURCES})

target_link_libraries(${TEST_NAME} PUBLIC
    Catch2::Catch2
    Catch2::Catch2WithMain
    GJK-EPA
)

# target_compile_definitions(${TEST_NAME} PUBLIC
    # -D TEST_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}/"
# )

install(TARGETS ${TEST_NAME})
