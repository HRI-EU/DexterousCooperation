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

###############################################################################
# Example classes for ExampleRunner
###############################################################################
ADD_LIBRARY(DcExamples ExamplePoseGraph.cpp)
TARGET_INCLUDE_DIRECTORIES(DcExamples PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
TARGET_LINK_LIBRARIES(DcExamples RcsGraphics RcsPhysics RcsGui Tropic DcPlanning RcsExamples)

# Copy the .dll file to the same folder as the executable
IF (WIN32)
ADD_CUSTOM_COMMAND(
    TARGET DcExamples POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/DcExamples.dll
    ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/DcExamples.dll)
ENDIF()

# We add this option to force the linker to export the symbols. This
# solves the issue that it will not be exposed, since all initialization 
# happens inside statically. Under Unix, we can use -Wl,--no-as-needed
IF (WIN32)
  TARGET_LINK_OPTIONS(DcExamples PUBLIC /INCLUDE:DcExampleInfo)
ENDIF()

###############################################################################
# Install the binaries
###############################################################################
INSTALL(TARGETS ExampleDcComponents DcExamples RUNTIME DESTINATION bin LIBRARY DESTINATION lib)
