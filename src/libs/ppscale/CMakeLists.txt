set(PPSCALE_INCLUDES
    ${ARCH_INCLUDES}
    )

set(PPSCALE_SOURCES
    ppscale.c
    )

add_library(ppscale STATIC ${PPSCALE_SOURCES})
target_include_directories(ppscale PRIVATE ${PPSCALE_INCLUDES})