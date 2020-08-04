set(NUKED_INCLUDES
    ${ARCH_INCLUDES}
    )

set(NUKED_SOURCES
    nukedopl.cpp
    )

add_library(nuked STATIC ${NUKED_SOURCES})
target_include_directories(nuked PRIVATE ${NUKED_INCLUDES})