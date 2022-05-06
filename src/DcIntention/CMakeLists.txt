##############################################################################
# We first look if Qt5 is vailable. If it isn't, we fall back to Qt4
##############################################################################
FIND_PACKAGE(Qt5 COMPONENTS Core Gui Widgets QUIET)
IF (NOT Qt5_FOUND)
  MESSAGE(STATUS "Couldn't find Qt5 - trying Qt4")
  FIND_PACKAGE(Qt4 REQUIRED)
  INCLUDE(${QT_USE_FILE})
ENDIF(NOT Qt5_FOUND)

###############################################################################
# DcIntention library: for interactive box and wheel turning
# Depends on Qt due to monitor widgets
###############################################################################
SET(DCINTENTION_SRC
  MonitorRequest.cpp
  ObjectModel.cpp
  BoxObjectModel.cpp
  WheelObjectModel.cpp
  ActionProgressGraph.cpp
  ActionProgressState.cpp
  ActionProgressTransition.cpp
  ActionProgressMonitor.cpp
  ManipulationComponent.cpp
  RemoteVisualizationComponent.cpp
  MonitorComponent.cpp
  MonitorWidgetBase.cpp 
  MonitorRequestWidget.cpp
  StateEstimatorComponent.cpp
)

ADD_LIBRARY(DcIntention ${DCINTENTION_SRC})
TARGET_INCLUDE_DIRECTORIES(DcIntention PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../../external/websocketpp/include ${CMAKE_CURRENT_SOURCE_DIR}/../../external/rapidjson/include) 
TARGET_LINK_LIBRARIES(DcIntention DcPlanning)
SET_TARGET_PROPERTIES(DcIntention PROPERTIES AUTOMOC TRUE)
IF (WIN32)
  TARGET_INCLUDE_DIRECTORIES(DcIntention PUBLIC ${HGR}/External/asio-1.12.1/include ${HGR}/External/boost/boost_1_59_0)
  TARGET_LINK_LIBRARIES(DcIntention wsock32 ws2_32 Iphlpapi)
ENDIF(WIN32)
COPY_DLL(DcIntention)

