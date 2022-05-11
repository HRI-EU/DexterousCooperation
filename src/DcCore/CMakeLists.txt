###############################################################################
# DcCore library: Depends only on Rcs
###############################################################################
SET(DCCORE_SRC
  EntityBase.cpp
  GraphComponent.cpp
  IKComponent.cpp
  JointGuiComponent.cpp
  TaskGuiComponent.cpp
  GraphicsWindow.cpp
  PhysicsComponent.cpp
  TTSComponent.cpp
  ColliderComponent.cpp
)

ADD_LIBRARY(DcCore ${DCCORE_SRC})
TARGET_LINK_LIBRARIES(DcCore RcsGraphics RcsPhysics RcsGui)

#TARGET_INCLUDE_DIRECTORIES(DcCore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${ESLIB_INCLUDE_DIR})

TARGET_INCLUDE_DIRECTORIES(DcCore PUBLIC
    # Header source location
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${ESLIB_INCLUDE_DIR}>
    
    # Header install location
    $<INSTALL_INTERFACE:include>
    
    # Regular dependencies
    )
COPY_DLL(DcCore)

###############################################################################
# Install the libraries
###############################################################################
INSTALL(
  DIRECTORY . 
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)
DEXTEROUSCOOPERATION_ADD_EXPORT_LIBRARY(DcCore)
INSTALL(TARGETS DcCore EXPORT DexterousCooperationExport DESTINATION lib/$ENV{MAKEFILE_PLATFORM})
