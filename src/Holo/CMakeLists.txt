###############################################################################
# DcHolo library: Depends on Rcs and ESlib
###############################################################################
SET(HOLO_SRC
  HoloComponent.cpp
  HoloGraph.cpp
  HoloNetworkConnection.cpp
  HoloParser.cpp
  HoloStudyDataLogger.cpp
  #HoloTestComponent.cpp
)

ADD_LIBRARY(DcHolo ${HOLO_SRC})
TARGET_LINK_LIBRARIES(DcHolo DcCore)
IF (WIN32)
  TARGET_LINK_LIBRARIES(DcHolo wsock32 ws2_32 Iphlpapi)
ENDIF(WIN32)

#TARGET_INCLUDE_DIRECTORIES(DcHolo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../../external/websocketpp/include ${CMAKE_CURRENT_SOURCE_DIR}/../../external/rapidjson/include)

TARGET_INCLUDE_DIRECTORIES(DcHolo PUBLIC
    # Header source location
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../external/websocketpp/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../external/rapidjson/include>
    
    # Header install location
    $<INSTALL_INTERFACE:include>
    
    # Regular dependencies
    )

IF (WIN32)
  ADD_CUSTOM_COMMAND(
      TARGET DcHolo POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy
      ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/DcHolo.dll
      ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/DcHolo.dll)
ENDIF(WIN32)

###############################################################################
# Install the libraries
###############################################################################
INSTALL(
  DIRECTORY . 
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)
DEXTEROUSCOOPERATION_ADD_EXPORT_LIBRARY(DcHolo)
INSTALL(TARGETS DcHolo EXPORT DexterousCooperationExport DESTINATION lib/$ENV{MAKEFILE_PLATFORM})

