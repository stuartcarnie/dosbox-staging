set(MAME_INCLUDES
    ${ARCH_INCLUDES}
    )

set(MAME_SOURCES
    emu.h
    fmopl.cpp
    fmopl.h
    saa1099.cpp
    saa1099.h
    sn76496.cpp
    sn76496.h
    ymdeltat.cpp
    ymdeltat.h
    ymf262.cpp
    ymf262.h
    )

add_library(mame STATIC ${MAME_SOURCES})
target_include_directories(mame PRIVATE ${MAME_INCLUDES})