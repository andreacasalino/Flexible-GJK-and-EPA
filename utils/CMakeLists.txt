FetchDep(https://github.com/nlohmann/json.git 7194245a314d142a5560f6906a87f7c67ebbcf2e "Fetching nlohmann json" nlohmann_json)

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

set(LIBNAME GJK-EPA-Utils)

add_library(${LIBNAME} 
    ${SOURCES}
)

target_link_libraries(${LIBNAME} PUBLIC
    GJK-EPA
    nlohmann_json::nlohmann_json
)

target_include_directories(${LIBNAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
target_compile_definitions(${LIBNAME} PUBLIC
-D LOG_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}/logs"
-D PYTHON_CMD="${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Visualize.py"
)
