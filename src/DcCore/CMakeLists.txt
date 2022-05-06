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
TARGET_INCLUDE_DIRECTORIES(DcCore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../../../ESLib/include)
COPY_DLL(DcCore)

