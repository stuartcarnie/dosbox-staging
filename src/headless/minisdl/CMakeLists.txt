set(MINISDL_INCLUDES
    ${ARCH_INCLUDES}
    )

set(MINISDL_SOURCES
    src/SDL.c
    src/SDL_error.c
    src/file/SDL_rwops.c
    src/file/cocoa/SDL_rwopsbundlesupport.m
    src/stdlib/SDL_malloc.c
    src/stdlib/SDL_qsort.c
    src/stdlib/SDL_string.c
    src/thread/SDL_thread.c
    # src/timer/unix/SDL_systimer.c
    )

add_library(minisdl STATIC ${MINISDL_SOURCES})
target_include_directories(minisdl PRIVATE ${MINISDL_INCLUDES})
target_compile_definitions(minisdl PRIVATE -DSDL_THREADS_DISABLED)