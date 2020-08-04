add_subdirectory(minisdl)
add_subdirectory(test)

set(HEADLESS_INCLUDES
    ${ARCH_INCLUDES}
    )

set(HEADLESS_SOURCES
    Headless.cpp
    mapper.cpp
    timer.c
    DOSBox.mm
    libco/libco.c
    )

add_library(headless STATIC ${HEADLESS_SOURCES})
target_link_libraries(headless PUBLIC midi)
target_include_directories(headless PRIVATE ${HEADLESS_INCLUDES} libco)
target_compile_options(headless PUBLIC "-fobjc-arc")