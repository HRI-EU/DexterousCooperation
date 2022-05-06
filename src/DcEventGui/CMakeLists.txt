##############################################################################
# We first look if Qt5 is vailable. If it isn't, we fall back to Qt4
##############################################################################
FIND_PACKAGE(Qt5 COMPONENTS Core Gui Widgets QUIET)
IF (NOT Qt5_FOUND)
  MESSAGE(STATUS "Couldn't find Qt5 - trying Qt4")
  FIND_PACKAGE(Qt4 REQUIRED)
  INCLUDE(${QT_USE_FILE})
ENDIF(NOT Qt5_FOUND)

##############################################################################
# Library for event system Qt Gui
##############################################################################
ADD_LIBRARY(DcEventGui EventGui.cpp)
TARGET_INCLUDE_DIRECTORIES(DcEventGui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
TARGET_LINK_LIBRARIES(DcEventGui DcCore)
SET_TARGET_PROPERTIES(DcEventGui PROPERTIES AUTOMOC TRUE)
COPY_DLL(DcEventGui)
