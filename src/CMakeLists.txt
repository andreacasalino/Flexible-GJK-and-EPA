message(STATUS "fetching Hull")
include(FetchContent)
FetchContent_Declare(
  hull
  GIT_REPOSITORY https://github.com/andreacasalino/Hull.git
  GIT_TAG        506a77bd926c0f448f070f2cfa58a0812edefeb3
)
FetchContent_MakeAvailable(hull)

set(PROJECT_SHORTNAME GJK-EPA)

MakeLibrary(${PROJECT_SHORTNAME} header)

target_link_libraries(${PROJECT_SHORTNAME} PUBLIC
Hull
)

if(LOGGER_OPT)
target_compile_definitions(${PROJECT_NAME} PUBLIC FLX_LOGGER_ENABLED)
endif()

