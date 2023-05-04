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

#TARGET_INCLUDE_DIRECTORIES(DcEventGui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

TARGET_INCLUDE_DIRECTORIES(DcEventGui PUBLIC
    # Header source location
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    
    # Header install location
    $<INSTALL_INTERFACE:include>
    
    # Regular dependencies
    )

TARGET_LINK_LIBRARIES(DcEventGui RcsGui)
SET_TARGET_PROPERTIES(DcEventGui PROPERTIES AUTOMOC TRUE)
COPY_DLL(DcEventGui)

###############################################################################
# Install the libraries
###############################################################################
INSTALL(
  DIRECTORY . 
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)
DEXTEROUSCOOPERATION_ADD_EXPORT_LIBRARY(DcEventGui)
INSTALL(TARGETS DcEventGui EXPORT DexterousCooperationExport DESTINATION lib/$ENV{MAKEFILE_PLATFORM})
