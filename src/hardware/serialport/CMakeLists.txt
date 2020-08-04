set(SERIAL_INCLUDES
    ${ARCH_INCLUDES}
    )

set(SERIAL_SOURCES
    directserial.cpp
    directserial.h
    libserial.cpp
    libserial.h
    misc_util.cpp
    misc_util.h
    nullmodem.cpp
    nullmodem.h
    serialdummy.cpp
    serialdummy.h
    serialport.cpp
    softmodem.cpp
    softmodem.h
    )

add_library(serial STATIC ${SERIAL_SOURCES})
target_include_directories(serial PRIVATE ${SERIAL_INCLUDES})