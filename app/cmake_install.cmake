# Install script for directory: /home/milad/software/robotology-superbuild/robotology/walking-controllers/app

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/milad/software/robotology-superbuild/build/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yarp/robots/iCubGazeboV2_5" TYPE FILE FILES
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGazeboV2_5/controllerParams.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGazeboV2_5/dcmReactiveControllerParams.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGazeboV2_5/dcmWalkingCoordinator.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGazeboV2_5/forceTorqueSensors.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGazeboV2_5/forwardKinematics.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGazeboV2_5/inverseKinematics.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGazeboV2_5/pidParams.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGazeboV2_5/plannerParams.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGazeboV2_5/qpInverseKinematics.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGazeboV2_5/robotControl.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGazeboV2_5/stepAdaptation.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGazeboV2_5/walkingLogger.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGazeboV2_5/zmpControllerParams.ini"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yarp/robots/iCubGenova04" TYPE FILE FILES
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGenova04/controllerParams.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGenova04/dcmReactiveControllerParams.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGenova04/dcmWalkingCoordinator.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGenova04/forceTorqueSensors.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGenova04/forwardKinematics.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGenova04/inverseKinematics.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGenova04/pidParams.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGenova04/plannerParams.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGenova04/qpInverseKinematics.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGenova04/robotControl.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGenova04/stepAdaptation.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGenova04/walkingLogger.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/iCubGenova04/zmpControllerParams.ini"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yarp/robots/icubGazeboSim" TYPE FILE FILES
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/icubGazeboSim/controllerParams.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/icubGazeboSim/dcmReactiveControllerParams.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/icubGazeboSim/dcmWalkingCoordinator.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/icubGazeboSim/forceTorqueSensors.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/icubGazeboSim/forwardKinematics.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/icubGazeboSim/inverseKinematics.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/icubGazeboSim/pidParams.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/icubGazeboSim/plannerParams.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/icubGazeboSim/qpInverseKinematics.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/icubGazeboSim/robotControl.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/icubGazeboSim/walkingLogger.ini"
    "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/robots/icubGazeboSim/zmpControllerParams.ini"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yarp/applications" TYPE FILE FILES "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/scripts/walking.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yarp/contexts" TYPE DIRECTORY FILES "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/dcmWalkingLogger")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yarp/contexts" TYPE DIRECTORY FILES "/home/milad/software/robotology-superbuild/robotology/walking-controllers/app/dcmWalkingJoypad")
endif()

