function(FetchDep URL TAG MSG CMAKE_NAME)
    message(STATUS ${MSG})
    include(FetchContent)
    FetchContent_Declare(
        ${CMAKE_NAME}
        GIT_REPOSITORY ${URL}
        GIT_TAG        ${TAG}
    )
    FetchContent_MakeAvailable(${CMAKE_NAME})
endfunction()
