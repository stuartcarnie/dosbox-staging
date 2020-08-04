set(CPU_INCLUDES
    ${ARCH_INCLUDES}
    )

set(CPU_SOURCES
    callback.cpp
    core_dynrec.cpp
    core_dyn_x86.cpp
    core_full.cpp
    core_normal.cpp
    core_prefetch.cpp
    core_simple.cpp
    cpu.cpp
    dyn_cache.h
    flags.cpp
    instructions.h
    lazyflags.h
    modrm.cpp
    modrm.h
    paging.cpp
    )

add_library(cpu STATIC ${CPU_SOURCES})
target_include_directories(cpu PRIVATE ${CPU_INCLUDES})