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

#TARGET_INCLUDE_DIRECTORIES(DcPlanning PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

TARGET_INCLUDE_DIRECTORIES(DcPlanning PUBLIC
    # Header source location
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    
    # Header install location
    $<INSTALL_INTERFACE:include>
    
    # Regular dependencies
    )
TARGET_LINK_LIBRARIES(DcPlanning DcCore Tropic Gras)
COPY_DLL(DcPlanning)

###############################################################################
# Install the libraries
###############################################################################
INSTALL(
  DIRECTORY . 
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)
DEXTEROUSCOOPERATION_ADD_EXPORT_LIBRARY(DcPlanning)
INSTALL(TARGETS DcPlanning EXPORT DexterousCooperationExport DESTINATION lib/$ENV{MAKEFILE_PLATFORM})
