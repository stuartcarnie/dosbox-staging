set(ZMBV_INCLUDES
    ${ARCH_INCLUDES}
    )

set(ZMBV_SOURCES

    )

add_library(zmbv STATIC ${ZMBV_SOURCES})
target_include_directories(zmbv PRIVATE ${ZMBV_INCLUDES})