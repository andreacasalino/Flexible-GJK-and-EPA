FetchDep(https://github.com/andreacasalino/Hull.git bed032cbfb6c9f36ff581ca132803a250809d37e "Fetching Hull" hull)

set(LIBRARY_NAME GJK-EPA)

file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.h ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

if(LIB_OPT)
	if (WIN32)
		set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
	endif ()	
	
	add_library(${LIBRARY_NAME} SHARED ${SOURCES})
else()
	add_library(${LIBRARY_NAME} STATIC ${SOURCES})
endif()

target_include_directories(${LIBRARY_NAME} PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/header
)

target_link_libraries(${LIBRARY_NAME} PUBLIC
  Hull
)

if(GJK_EPA_DIAGNOSTIC)
target_compile_definitions(${LIBRARY_NAME} PUBLIC 
  GJK_EPA_DIAGNOSTIC
)
endif()

install(TARGETS ${LIBRARY_NAME})
install (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${INCLUDE_DIR}/ DESTINATION include/${LIBRARY_NAME} FILES_MATCHING PATTERN "*.h*")
