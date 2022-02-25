macro(COPY_SCRIPT TARGET SCRIPT)
    add_custom_command(TARGET ${TARGET} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${SCRIPT}.py ${CMAKE_CURRENT_BINARY_DIR}/${SCRIPT}.py)    
endmacro()

macro(MAKE_SAMPLE NAME SCRIPT)
    add_executable(${NAME} ${NAME}.cpp Utils.h Utils.cpp)
    target_link_libraries(${NAME} PUBLIC GJK-EPA)

    COPY_SCRIPT(${NAME} Plotter)
    COPY_SCRIPT(${NAME} ${SCRIPT})	
endmacro()

MAKE_SAMPLE(Sample01 Visualize01)
MAKE_SAMPLE(Sample02 Visualize02)
MAKE_SAMPLE(Sample03 Visualize03)
