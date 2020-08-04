add_subdirectory(cpu)
add_subdirectory(debug)
add_subdirectory(dos)
add_subdirectory(fpu)
add_subdirectory(gui)
add_subdirectory(hardware)
add_subdirectory(headless)
add_subdirectory(ints)
add_subdirectory(libs)
add_subdirectory(midi)
add_subdirectory(misc)
add_subdirectory(shell)

set(DOSBOX_INCLUDES
    ${ARCH_INCLUDES}
    )

set(DOSBOX_SOURCES
    dosbox.cpp
    )

set(DOSBOX_LIBS
    cpu debug dos fpu gui hardware mame serial
    ints decoders nuked ppscale midi misc shell
    )

if (USE_HEADLESS)
    list(APPEND DOSBOX_LIBS headless minisdl)
    add_library(oedosbox SHARED ${DOSBOX_SOURCES})
    target_link_libraries(oedosbox ${DOSBOX_LIBS} "-framework Foundation")
    target_include_directories(oedosbox PRIVATE ${DOSBOX_INCLUDES})
else()
    list(APPEND DOSBOX_LIBS ${SDL2_LIBRARIES})
    add_executable(dosbox ${DOSBOX_SOURCES})
    target_include_directories(dosbox PRIVATE ${DOSBOX_INCLUDES})
    target_link_libraries(dosbox ${DOSBOX_LIBS} "-framework Foundation")
endif()

