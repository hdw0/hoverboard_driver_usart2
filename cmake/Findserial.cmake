find_path(SERIAL_INCLUDE_DIR NAMES serial/serial.h
  PATHS
    /usr/include
    /usr/local/include
)

find_library(SERIAL_LIBRARY NAMES serial
  PATHS
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
)

if(SERIAL_INCLUDE_DIR AND SERIAL_LIBRARY)
  set(SERIAL_FOUND TRUE)
  set(SERIAL_LIBRARIES ${SERIAL_LIBRARY})
  set(SERIAL_INCLUDE_DIRS ${SERIAL_INCLUDE_DIR})
else()
  set(SERIAL_FOUND FALSE)
endif()

mark_as_advanced(SERIAL_INCLUDE_DIR SERIAL_LIBRARY)
