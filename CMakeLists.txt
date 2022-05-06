CMAKE_MINIMUM_REQUIRED(VERSION 3.4)
SET(BST_INSTALL_CATEGORY Libraries) 

################################################################################
# Macro to copy dlls into bin directory
################################################################################
MACRO(COPY_DLL libname)
IF (WIN32)
  ADD_CUSTOM_COMMAND(
      TARGET ${libname} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy
      ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/${libname}.dll
      ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/${libname}.dll)
ENDIF(WIN32)
ENDMACRO()

################################################################################
# Check if project is stand-alone or inside tree with dependencies
################################################################################
SET(STANDALONE_PROJECT OFF)
IF(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  PROJECT(DexterousCooperation)
  LIST(APPEND CMAKE_PREFIX_PATH $ENV{SIT}/Libraries/Rcs/1.0/share/cmake/Rcs)
  LIST(APPEND CMAKE_PREFIX_PATH $ENV{SIT}/Libraries/Tropic/1.0/share/cmake/Tropic)
  LIST(APPEND CMAKE_PREFIX_PATH $ENV{SIT}/Libraries/Gras/1.0/share/cmake/Gras)
  FIND_PACKAGE(Rcs REQUIRED)
  FIND_PACKAGE(Tropic REQUIRED)
  FIND_PACKAGE(Gras REQUIRED)
  MESSAGE(STATUS "Compiling ${PROJECT_NAME} as stand-alone project")
  SET(STANDALONE_PROJECT ON)
  SET(ESLIB_INCLUDE_DIR "$ENV{SIT}/Libraries/ESLib/1.0/include")
  INCLUDE($ENV{SIT}/Libraries/ESLib/1.0/share/ESLibValidation/cmake/EventValidation.cmake)
  FIND_PACKAGE(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
ELSE()
  SET(RCS_CONFIG_DIR "config")
  MESSAGE(STATUS "Compiling ${PROJECT_NAME} as in-tree project")
ENDIF()

SET(BUILD_SHARED_LIBS ON)

################################################################################
# Otherwise no import library is built 
################################################################################
SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${ESLIB_INCLUDE_DIR})

IF (WIN32)
  ADD_DEFINITIONS(-D_USE_MATH_DEFINES)   # M_PI etc. for windows
ENDIF(WIN32)

IF (TARGET RoboComponents)
  MESSAGE(STATUS "Compiling with robot interfaces")
  SET(ROBO_LIBS RoboComponents)
ELSE()
  MESSAGE(STATUS "Compiling without robot interfaces")
ENDIF()


###############################################################################
# Go through all sub-directories
###############################################################################
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(bin)
ADD_SUBDIRECTORY(examples)

################################################################################
# Deep copy of the config directory for windows
################################################################################
SET(CFG_FROM ${CMAKE_CURRENT_SOURCE_DIR}/config)
SET(CFG_TO ${PROJECT_BINARY_DIR}/config/xml/DexterousCooperation)

IF (WIN32 AND (NOT IS_DIRECTORY "${CFG_TO}") AND (EXISTS "${CFG_TO}"))
  MESSAGE(STATUS "Removing non-directory ${CONFIG_DST_DIR}")
  FILE(REMOVE ${CONFIG_DST_DIR})
ENDIF()

IF(NOT EXISTS "${CFG_TO}")

  IF (NOT EXISTS ${PROJECT_BINARY_DIR}/config/xml)
    FILE(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/config/xml)
  ENDIF()
  
  IF(UNIX)
    SET(CONFIGDIR_COPY_COMMAND create_symlink)
    MESSAGE(STATUS "Linking config directory from ${CFG_FROM} to ${CFG_TO}")
  ELSE()
    SET(CONFIGDIR_COPY_COMMAND copy_directory)
    MESSAGE(STATUS "Copying config directory from ${CFG_FROM} to ${CFG_TO}")
  ENDIF()

  EXECUTE_PROCESS (COMMAND
   ${CMAKE_COMMAND} -E ${CONFIGDIR_COPY_COMMAND} ${CFG_FROM} ${CFG_TO})
ELSE()
  MESSAGE(STATUS "Config directory ${CFG_TO} already exists - skipping")
ENDIF()
