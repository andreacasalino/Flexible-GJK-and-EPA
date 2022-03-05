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

if(GJK_EPA_DIAGNOSTIC)
  message(STATUS "fetching nlohmann json for diagnostics")
  include(FetchContent)
  FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        7194245a314d142a5560f6906a87f7c67ebbcf2e
  )
  FetchContent_MakeAvailable(nlohmann_json)

  target_compile_definitions(${PROJECT_SHORTNAME} PRIVATE 
    GJK_EPA_DIAGNOSTIC
  )

  target_link_libraries(${PROJECT_SHORTNAME} PRIVATE 
    nlohmann_json::nlohmann_json
  )
endif()
