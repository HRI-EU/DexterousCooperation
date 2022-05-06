###############################################################################
# Test program for component-based wheel turning
###############################################################################
ADD_EXECUTABLE(ExampleDcComponents ExampleDcComponents.cpp)
IF(USE_BULLET)
  TARGET_COMPILE_DEFINITIONS(ExampleDcComponents PRIVATE ${BULLET_DEFINITIONS})
ENDIF(USE_BULLET)
TARGET_INCLUDE_DIRECTORIES(ExampleDcComponents PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/DcCommon)
TARGET_LINK_LIBRARIES(ExampleDcComponents DcCore DcPlanning DcEventGui ${ROBO_LIBS})
IF(USE_DC_ROS)
  TARGET_LINK_LIBRARIES(ExampleDcComponents DcROS)
ENDIF(USE_DC_ROS)

TARGET_COMPILE_DEFINITIONS(ExampleDcComponents PRIVATE -DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")
#TARGET_VALIDATE_EVENTS(TestDexterousCooperation)
