macro(MAKE_SAMPLE NAME)
    add_executable(${NAME} ${NAME}.cpp)
    target_link_libraries(${NAME} PUBLIC 
        GJK-EPA
        GJK-EPA-Utils
    )
endmacro()

MAKE_SAMPLE(Readme)
MAKE_SAMPLE(Sample01)
MAKE_SAMPLE(Sample02)
MAKE_SAMPLE(Sample03)
MAKE_SAMPLE(Sample04)
