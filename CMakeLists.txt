cmake_minimum_required(VERSION 3.17.0)

project(dosbox LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 11)

option(USE_HEADLESS "Compile OpenEmu headless library" ON)
option(USE_OPUS "Use Opus sound" OFF)

list(APPEND ARCH_INCLUDES
    ${CMAKE_SOURCE_DIR}/include
    )

if (USE_HEADLESS)
    add_definitions(-DUSE_HEADLESS)
    list(APPEND ARCH_INCLUDES
        ${CMAKE_SOURCE_DIR}/src/headless
        ${CMAKE_SOURCE_DIR}/src/headless/minisdl/include
        )
else()
    find_package(SDL2 REQUIRED)
    list(APPEND ARCH_INCLUDES ${SDL2_INCLUDE_DIRS})
endif()

add_subdirectory(src)