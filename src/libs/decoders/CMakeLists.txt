set(DECODERS_INCLUDES
    ${ARCH_INCLUDES}
    )

set(DECODERS_SOURCES
    archive.h
    dr_flac.h
    dr_mp3.h
    dr_wav.h
    flac.c
    mp3.cpp
    mp3_seek_table.cpp
    mp3_seek_table.h
    SDL_sound.c
    SDL_sound.h
    SDL_sound_internal.h
    stb.h
    stb_vorbis.h
    vorbis.c
    wav.c
    xxhash.h
    )

if (USE_OPUS)
    message(FATAL_ERROR "Need to fix Opus support")
endif()

add_library(decoders STATIC ${DECODERS_SOURCES})
target_include_directories(decoders PRIVATE ${DECODERS_INCLUDES})