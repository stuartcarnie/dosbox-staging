set(SHELL_INCLUDES
    ${ARCH_INCLUDES}
    )

set(SHELL_SOURCES
    shell.cpp shell_batch.cpp shell_cmds.cpp shell_misc.cpp
    )

add_library(shell STATIC ${SHELL_SOURCES})
target_include_directories(shell PRIVATE ${SHELL_INCLUDES})