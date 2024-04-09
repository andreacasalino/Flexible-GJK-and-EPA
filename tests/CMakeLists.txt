FetchDep(https://github.com/catchorg/Catch2.git v3.4.0 "Fetching Catch2" catch2)
	
set(TEST_NAME Tests)

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

add_executable(${TEST_NAME} 
    ${SOURCES}
)

target_link_libraries(${TEST_NAME} PUBLIC
    Catch2::Catch2
    Catch2::Catch2WithMain
    GJK-EPA
    GJK-EPA-Utils
)

# target_compile_definitions(${TEST_NAME} PUBLIC
    # -D TEST_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}/"
# )

install(TARGETS ${TEST_NAME})
