###############################################################################
# DcPlanning library: Depends additionally on RcsTrajectory and RcsSearch
###############################################################################
SET(DCPLANNING_SRC
  DyadicMotionPlannerComponent.cpp
  BoxStrategy5D.cpp
  DyadicTrajectorySet.cpp
  TrajectoryComponent.cpp
  TrajectoryPredictor.cpp
  WheelConstraint.cpp
  WheelPlannerComponent.cpp
  WheelStrategy7D.cpp
  JacoConstraint.cpp
  DynamicPoseGraph.cpp
  PoseGraph.cpp
)

ADD_LIBRARY(DcPlanning ${DCPLANNING_SRC})
TARGET_INCLUDE_DIRECTORIES(DcPlanning PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
TARGET_LINK_LIBRARIES(DcPlanning DcCore Tropic Gras)
COPY_DLL(DcPlanning)

