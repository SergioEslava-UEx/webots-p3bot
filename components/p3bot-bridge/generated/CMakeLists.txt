cmake_minimum_required(VERSION 3.10)

#Select the compiler standard
if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 11)
    message(FATAL_ERROR "Version of g++ is lower than 11, g++ 11 or higher is required.")
elseif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 12)
    message(STATUS "Using g++ 11, setting C++20 standard.")
    set(CXX_STANDARD 20)
elseif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 13)
    message(STATUS "Using g++ 12, setting C++23 standard.")
    set(CXX_STANDARD 23)
else()
    message(STATUS "Using g++ 13 or higher, setting the C++23 standard.")
    set(CXX_STANDARD 23)
endif()


IF(NOT EXISTS $ENV{ROBOCOMP}/cmake)
  MESSAGE(FATAL_ERROR "Couldn't find RoboComp in $ENV{ROBOCOMP}! Exiting...")
ENDIF(NOT EXISTS $ENV{ROBOCOMP}/cmake)

INCLUDE($ENV{ROBOCOMP}/cmake/robocomp.cmake)
INCLUDE($ENV{ROBOCOMP}/cmake/modules/qt.cmake)
INCLUDE (../src/CMakeLists.txt)

ADD_DEFINITIONS ("-DICE_CPP11_MAPPING")
FIND_PACKAGE( Ice REQUIRED COMPONENTS Ice++11 IceStorm++11)

# Sources set
SET (SOURCES
  ${SOURCES}
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/sigwatch/sigwatch.cpp
  $ENV{ROBOCOMP}/classes/grafcetStep/GRAFCETStep.cpp
  $ENV{ROBOCOMP}/classes/ConfigLoader/ConfigLoader.cpp
  main.cpp
  genericworker.cpp
  camera360rgbI.cpp
  kinovaarmI.cpp
  lidar3dI.cpp
  omnirobotI.cpp
)


#ROBOCOMP
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_IDSL_TO_ICE(Lidar3D OmniRobot GenericBase KinovaArm FullPoseEstimation Camera360RGB FullPoseEstimationPub)
ROBOCOMP_ICE_TO_SRC(Lidar3D OmniRobot GenericBase KinovaArm FullPoseEstimation Camera360RGB FullPoseEstimationPub)
SET (EXECUTABLE_OUTPUT_PATH ${RC_COMPONENT_DEVEL_PATH}/bin)

# Specify construction and link process
ADD_EXECUTABLE( P3BotBridge ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( P3BotBridge ${LIBS}  ${STATIC_LIBS} ${SPECIFIC_LIBS} ${QT_LIBRARIES} ${Ice_LIBRARIES} )
SET_TARGET_PROPERTIES(P3BotBridge
        PROPERTIES
        CXX_STANDARD ${CXX_STANDARD}
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
        )


INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/P3BotBridge DESTINATION ${RC_COMPONENT_INSTALL_PATH}/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
