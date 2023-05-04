###############################################################################
# Test program for component-based wheel turning
###############################################################################
ADD_EXECUTABLE(TestDexterousCooperation TestDexterousCooperation.cpp ../src/DcCommon/EcsHardwareComponents.cpp)
IF(USE_BULLET)
  TARGET_COMPILE_DEFINITIONS(TestDexterousCooperation PRIVATE ${BULLET_DEFINITIONS})
ENDIF(USE_BULLET)
TARGET_INCLUDE_DIRECTORIES(TestDexterousCooperation PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/DcCommon)
TARGET_LINK_LIBRARIES(TestDexterousCooperation DcIntention DcEventGui ${ROBO_LIBS})
TARGET_COMPILE_DEFINITIONS(TestDexterousCooperation PRIVATE -DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")
TARGET_COMPILE_DEFINITIONS(TestDexterousCooperation PRIVATE ${ROBO_DEFINES})
TARGET_VALIDATE_EVENTS(TestDexterousCooperation)

###############################################################################
# Test program for component-based box turning
###############################################################################
ADD_EXECUTABLE(TestBoxPlanner TestBoxPlanner.cpp ../src/DcCommon/EcsHardwareComponents.cpp)
IF(USE_BULLET)
  TARGET_COMPILE_DEFINITIONS(TestBoxPlanner PRIVATE ${BULLET_DEFINITIONS})
ENDIF(USE_BULLET)
TARGET_INCLUDE_DIRECTORIES(TestBoxPlanner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/DcCommon)
TARGET_LINK_LIBRARIES(TestBoxPlanner DcPlanning DcEventGui ${ROBO_LIBS})
TARGET_COMPILE_DEFINITIONS(TestBoxPlanner PRIVATE -DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")
TARGET_COMPILE_DEFINITIONS(TestBoxPlanner PRIVATE ${ROBO_DEFINES})
#TARGET_VALIDATE_EVENTS(TestBoxPlanner)

###############################################################################
# Test program for component-based wheel turning
###############################################################################
ADD_EXECUTABLE(TestWheelPlanner TestWheelPlanner.cpp ../src/DcCommon/EcsHardwareComponents.cpp)
IF(USE_BULLET)
  TARGET_COMPILE_DEFINITIONS(TestWheelPlanner PRIVATE ${BULLET_DEFINITIONS})
ENDIF(USE_BULLET)
TARGET_INCLUDE_DIRECTORIES(TestWheelPlanner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/DcCommon)
TARGET_LINK_LIBRARIES(TestWheelPlanner DcPlanning DcEventGui ${ROBO_LIBS})
TARGET_COMPILE_DEFINITIONS(TestWheelPlanner PRIVATE -DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")
TARGET_COMPILE_DEFINITIONS(TestWheelPlanner PRIVATE ${ROBO_DEFINES})
#TARGET_VALIDATE_EVENTS(TestBoxPlanner)

###############################################################################
# Test program for component-based polygon outline planner
###############################################################################
ADD_EXECUTABLE(TestPolygonPlanner TestPolygonPlanner.cpp)# ../src/DcCommon/EcsHardwareComponents.cpp)
TARGET_INCLUDE_DIRECTORIES(TestPolygonPlanner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/DcCommon)
TARGET_LINK_LIBRARIES(TestPolygonPlanner DcVision DcEventGui ${ROBO_LIBS})
  IF (UNIX)
    TARGET_LINK_LIBRARIES(TestPolygonPlanner -Wl,--no-as-needed DcExamples -Wl,--as-needed)
  ELSE()
    TARGET_LINK_LIBRARIES(TestPolygonPlanner DcExamples)
  ENDIF()
TARGET_COMPILE_DEFINITIONS(TestPolygonPlanner PRIVATE -DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")
TARGET_COMPILE_DEFINITIONS(TestPolygonPlanner PRIVATE ${ROBO_DEFINES})

###############################################################################
# Test program for pose sequence optimization
###############################################################################
ADD_EXECUTABLE(TestPoseGraph TestPoseGraph.cpp)
TARGET_INCLUDE_DIRECTORIES(TestPoseGraph PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/DcCommon)
TARGET_LINK_LIBRARIES(TestPoseGraph DcPlanning DcEventGui RcsExamples DcExamples)
TARGET_COMPILE_DEFINITIONS(TestPoseGraph PRIVATE -DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")

###############################################################################
# Test program for component-based polygon outline planner without libs
###############################################################################
LIST(APPEND POLY_SRC
  ../src/DcPlanning/TrajectoryComponent.cpp
  ../src/DcPlanning/TrajectoryPredictor.cpp
  ../src/DcPlanning/BoxStrategy5D.cpp
  ../src/DcVision/PolygonObjectConstraint.cpp
  ../src/DcVision/PolygonContactPoint2D.cpp
  ../src/DcVision/DyadicPolygonObjectConstraint.cpp
  ../src/DcVision/PolygonObjectPlanner.cpp
  ../src/DcVision/PolygonObjectModel.cpp
  ../src/DcVision/RebaComponent.cpp
  ../src/DcVision/PolyGraspDetector.cpp
  ../src/DcVision/PolyROSComponent.cpp)

IF (USE_DC_ROS)
  LIST(APPEND POLY_SRC ../src/DcVision/VirtualKinect2.cpp)
ENDIF(USE_DC_ROS)

ADD_EXECUTABLE(TestPolygonPlanner_event TestPolygonPlanner.cpp ${POLY_SRC} ../src/DcCommon/EcsHardwareComponents.cpp)
TARGET_INCLUDE_DIRECTORIES(TestPolygonPlanner_event PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/DcCommon
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/DcPlanning
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/DcVision)
TARGET_LINK_LIBRARIES(TestPolygonPlanner_event RcsCore RcsGraphics RcsPhysics Tropic Gras DcEventGui DcPlanning ${ROBO_LIBS})
TARGET_COMPILE_DEFINITIONS(TestPolygonPlanner_event PRIVATE -DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")
TARGET_COMPILE_DEFINITIONS(TestPolygonPlanner_event PRIVATE ${ROBO_DEFINES})
IF (USE_DC_ROS)
  FIND_PACKAGE(image_transport REQUIRED HINTS "$ENV{ROS_PACKAGE_PATH}/image_transport")
  FIND_PACKAGE(cv_bridge REQUIRED HINTS "$ENV{ROS_PACKAGE_PATH}/cv_bridge")
  TARGET_INCLUDE_DIRECTORIES(TestPolygonPlanner_event PUBLIC ${cv_bridge_INCLUDE_DIRS})
  TARGET_LINK_LIBRARIES(TestPolygonPlanner_event DcROS ${image_transport_LIBRARIES} ${cv_bridge_LIBRARIES})
ENDIF()
TARGET_VALIDATE_EVENTS(TestPolygonPlanner_event)

###############################################################################
# Install the binaries
###############################################################################
INSTALL(TARGETS TestDexterousCooperation TestBoxPlanner TestWheelPlanner TestPolygonPlanner TestPoseGraph RUNTIME DESTINATION bin LIBRARY DESTINATION lib)
