set(MISC_INCLUDES
    ${ARCH_INCLUDES}
    )

set(MISC_SOURCES
    cross.cpp messages.cpp programs.cpp setup.cpp support.cpp
    )

add_library(misc STATIC ${MISC_SOURCES})
target_include_directories(misc PRIVATE ${MISC_INCLUDES})