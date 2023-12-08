# Install script for directory: /home/hoang/Documents/shiv/src/sti_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/hoang/Documents/shiv/src/sti_msgs/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/hoang/Documents/shiv/src/sti_msgs/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.bash;/usr/local/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/hoang/Documents/shiv/src/sti_msgs/build/catkin_generated/installspace/setup.bash"
    "/home/hoang/Documents/shiv/src/sti_msgs/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.sh;/usr/local/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/hoang/Documents/shiv/src/sti_msgs/build/catkin_generated/installspace/setup.sh"
    "/home/hoang/Documents/shiv/src/sti_msgs/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.zsh;/usr/local/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/hoang/Documents/shiv/src/sti_msgs/build/catkin_generated/installspace/setup.zsh"
    "/home/hoang/Documents/shiv/src/sti_msgs/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/home/hoang/Documents/shiv/src/sti_msgs/build/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sti_msgs/msg" TYPE FILE FILES
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Velocities.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/EMC.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_0x.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_read.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Modbus_4x_write.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Map_config.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Request_mapInfo.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/List_MapName.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parameter_mergerMap.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_getPoseAruco.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/webConsole_receive.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Goal_control.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Check_shelves.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goal_control.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_goalControl.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_status.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Parking_control.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_control.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Setpose_status.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Safety_zone.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_status.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Mission_control.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_request.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Client_respond.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_request.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Move_respond.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Ps2_msgs.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Read_prepheral.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Write_prepheral.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Status_base.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_parameter.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Motor_rpm.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRespond.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_infoRequest.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRespond.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/FL_cmdRequest.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_cmdRequest.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRespond.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/NN_infoRequest.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeAuto.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeByHand.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeLaunch.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailFL.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeDetailNN.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papeMessage.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_allButton.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_infoGeneral.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_buttonPassword.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/HMI_papePassword.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/PID.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_missionControl.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_status.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Main_switchMode.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Zone_lidar_2head.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/ManageLaunch.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Port_status.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_status.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Reconnect_stt.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Magnetic_line.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/RFID.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_info.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/HC_request.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_info.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/POWER_request.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_control.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_status.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_control.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Lift_status.msg"
    "/home/hoang/Documents/shiv/src/sti_msgs/msg/Imu_version1.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sti_msgs/cmake" TYPE FILE FILES "/home/hoang/Documents/shiv/src/sti_msgs/build/catkin_generated/installspace/sti_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/hoang/Documents/shiv/src/sti_msgs/build/devel/include/sti_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/hoang/Documents/shiv/src/sti_msgs/build/devel/share/roseus/ros/sti_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/hoang/Documents/shiv/src/sti_msgs/build/devel/share/common-lisp/ros/sti_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/hoang/Documents/shiv/src/sti_msgs/build/devel/share/gennodejs/ros/sti_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/hoang/Documents/shiv/src/sti_msgs/build/devel/lib/python2.7/dist-packages/sti_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/hoang/Documents/shiv/src/sti_msgs/build/devel/lib/python2.7/dist-packages/sti_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/hoang/Documents/shiv/src/sti_msgs/build/catkin_generated/installspace/sti_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sti_msgs/cmake" TYPE FILE FILES "/home/hoang/Documents/shiv/src/sti_msgs/build/catkin_generated/installspace/sti_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sti_msgs/cmake" TYPE FILE FILES
    "/home/hoang/Documents/shiv/src/sti_msgs/build/catkin_generated/installspace/sti_msgsConfig.cmake"
    "/home/hoang/Documents/shiv/src/sti_msgs/build/catkin_generated/installspace/sti_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sti_msgs" TYPE FILE FILES "/home/hoang/Documents/shiv/src/sti_msgs/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hoang/Documents/shiv/src/sti_msgs/build/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/hoang/Documents/shiv/src/sti_msgs/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
