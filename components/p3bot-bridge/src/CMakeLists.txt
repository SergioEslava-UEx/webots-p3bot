# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  fixedsizedeque.h
)


LIST(APPEND LIBS ${LIBS})

find_package(OpenMP REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Linking OpenCV
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/opencv4.cmake )

# Disabling Pragma region warnings.
add_definitions(
    -O3               # Máxima optimización
    -Wall             # Todos los warnings
    -fmax-errors=1    # Mostrar solo 1 error
    -fno-char8_t      # Deshabilitar char8_t para compatibilidad
    -Wno-unknown-pragmas  # Ignorar pragmas desconocidos
)

# Linking Webots libraries
set(WEBOTS_HOME "/usr/local/webots")
link_directories(${WEBOTS_HOME}/lib/controller)
LIST(APPEND LIBS -L/usr/local/webots/lib/controller Controller CppController OpenMP::OpenMP_CXX)
include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)

