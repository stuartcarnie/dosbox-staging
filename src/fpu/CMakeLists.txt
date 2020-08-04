set(FPU_INCLUDES
    ${ARCH_INCLUDES}
    )

set(FPU_SOURCES
    fpu.cpp
    fpu_instructions.h
    fpu_instructions_x86.h
    )

add_library(fpu STATIC ${FPU_SOURCES})
target_include_directories(fpu PRIVATE ${FPU_INCLUDES})