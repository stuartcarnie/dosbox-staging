set(DEBUG_INCLUDES
    ${ARCH_INCLUDES}
    )

set(DEBUG_SOURCES
    debug.cpp
    debug_gui.cpp
    debug_disasm.cpp
    debug_inc.h
    debug_win32.cpp
    )

add_library(debug STATIC ${DEBUG_SOURCES})
target_include_directories(debug PRIVATE ${DEBUG_INCLUDES})