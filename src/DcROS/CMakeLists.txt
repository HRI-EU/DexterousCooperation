################################################################################
# ROS
################################################################################
IF (UNIX)

  OPTION(USE_DC_ROS "Enable DexterousCooperation ROS" OFF)

  IF(USE_DC_ROS)

    LIST(APPEND CMAKE_PREFIX_PATH /opt/ros/$ENV{ROS_DISTRO})
    FIND_PACKAGE(roscpp QUIET)
    IF(NOT roscpp_FOUND)
      MESSAGE(STATUS "Couldn't find roscpp, so not building ROS-specific functionality. Did you source the ROS setup.sh file?")
    ELSE()
      MESSAGE(STATUS "Found roscpp installed at ${roscpp_DIR}, so building ROS-specific functionality.")
    ENDIF()

    FIND_PACKAGE(std_msgs REQUIRED)
    FIND_PACKAGE(tf2_ros REQUIRED)
    FIND_PACKAGE(geometry_msgs REQUIRED)
    FIND_PACKAGE(trajectory_msgs REQUIRED)
    FIND_PACKAGE(sensor_msgs REQUIRED)

    ############################################################################
    # Source Files
    ############################################################################
    SET(SRC
      ROSSpinnerComponent.cpp
      )

    ADD_LIBRARY(DcROS ${SRC})

    TARGET_LINK_LIBRARIES(DcROS
      ${roscpp_LIBRARIES}
      ${std_msgs_LIBRARIES}
      ${tf2_ros_LIBRARIES}
      ${geometry_msgs_LIBRARIES}
      ${sensor_msgs_LIBRARIES}
      RcsCore DcCore
      )

    TARGET_INCLUDE_DIRECTORIES(DcROS PUBLIC 
      ${CMAKE_CURRENT_SOURCE_DIR} 
      ${CMAKE_CURRENT_SOURCE_DIR}/RcsROSMsg
      #${CMAKE_CURRENT_SOURCE_DIR}/../ECS 
      #${CMAKE_CURRENT_SOURCE_DIR}/../ESLib/include
      )
    TARGET_INCLUDE_DIRECTORIES(DcROS SYSTEM PUBLIC ${roscpp_INCLUDE_DIRS})
    TARGET_COMPILE_DEFINITIONS(DcROS PUBLIC -DUSE_ROS)

    # Mark for export
    RCS_ADD_EXPORT_LIBRARY(DcROS)

  ENDIF(USE_DC_ROS)

ENDIF(UNIX)
