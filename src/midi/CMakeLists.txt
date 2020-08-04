set(MIDI_INCLUDES
    ${ARCH_INCLUDES}
    )

set(MIDI_SOURCES
    midi.cpp midi_alsa.h midi_coreaudio.h midi_coremidi.h
    midi_oss.h midi_win32.h
    )

add_library(midi STATIC ${MIDI_SOURCES})
target_include_directories(midi PUBLIC . PRIVATE ${MIDI_INCLUDES} )