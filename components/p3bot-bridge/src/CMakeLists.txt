# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
)


LIST(APPEND LIBS ${LIBS})

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-fmax-errors=1 -fno-char8_t)

# Disabling Pragma region warnings.
add_definitions(-O3 -Wall -fmax-errors=1 -fno-char8_t -Wno-unknown-pragmas)

# Linking Webots libraries
set(WEBOTS_HOME "/usr/local/webots")
link_directories(${WEBOTS_HOME}/lib/controller)
SET (LIBS ${LIBS} -L/usr/local/webots/lib/controller Controller CppController)
include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)

