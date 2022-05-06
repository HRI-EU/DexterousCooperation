##############################################################################
# Polygon planning target
##############################################################################
LIST(APPEND POLY_SRC
  PolygonObjectConstraint.cpp
  PolygonContactPoint2D.cpp
  DyadicPolygonObjectConstraint.cpp
  PolygonObjectPlanner.cpp
  PolygonObjectModel.cpp
  RebaComponent.cpp
  PolyGraspDetector.cpp
  PolyROSComponent.cpp)
  
IF (USE_DC_ROS)
  LIST(APPEND POLY_SRC VirtualKinect2.cpp)
ENDIF(USE_DC_ROS)

ADD_LIBRARY(DcVision ${POLY_SRC})
TARGET_INCLUDE_DIRECTORIES(DcVision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
TARGET_LINK_LIBRARIES(DcVision DcPlanning)

IF (USE_DC_ROS)
  LIST(APPEND CMAKE_PREFIX_PATH "$ENV{ROS_PACKAGE_PATH}/image_transport")
  LIST(APPEND CMAKE_PREFIX_PATH "$ENV{ROS_PACKAGE_PATH}/cv_bridge")
  FIND_PACKAGE(image_transport REQUIRED HINTS "$ENV{ROS_PACKAGE_PATH}/image_transport")
  FIND_PACKAGE(cv_bridge REQUIRED HINTS "$ENV{ROS_PACKAGE_PATH}/cv_bridge")
  
  TARGET_INCLUDE_DIRECTORIES(DcVision PUBLIC ${cv_bridge_INCLUDE_DIRS})
  TARGET_LINK_LIBRARIES(DcVision DcROS ${image_transport_LIBRARIES} ${cv_bridge_LIBRARIES})

  ADD_EXECUTABLE(ExampleVirtualKinect ExampleVirtualKinect.cpp)
  TARGET_LINK_LIBRARIES(ExampleVirtualKinect DcVision DcEventGui RoboComponents DcROS)

ENDIF()

COPY_DLL(DcVision)
