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
  SET(RCS_CONFIG_DIR "$ENV{SIT}/Libraries/Rcs/1.0/config")
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

  IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    SET(CMAKE_INSTALL_PREFIX "$ENV{SIT}/Libraries/DexterousCooperation/1.0" CACHE PATH "Default sand box directory" FORCE)
  ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
ELSE()
  SET(RCS_CONFIG_DIR "config")
  SET(ESLIB_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ESLib/include")
  MESSAGE(STATUS "Compiling ${PROJECT_NAME} as in-tree project")
ENDIF()

SET(BUILD_SHARED_LIBS ON)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

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

################################################################################
#
# Add the automatically determined parts of the RPATH which point to directories
# outside the build tree to the install RPATH. Required for Vortex and other
# dependencies from the SIT.
# Note: This only works with CMake >= 3.0. With 2.8, the library directories of
# the dependencies must be added to the LD_LIBRARY_PATH manually in order to
# use the installed executables. This has no effect on using the package in
# another CMake project.
#
################################################################################
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

################################################################################
#
# The RPATH to be used when installing, but only if it's not a system directory
# See CMake wiki:
# (https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/RPATH-handling)
#
################################################################################
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/$ENV{MAKEFILE_PLATFORM}")
ENDIF("${isSystemDir}" STREQUAL "-1")

################################################################################
# Setup exported target collection
################################################################################
SET(DEXTEROUSCOOPERATION_EXPORT_LIBRARIES "" CACHE INTERNAL "")
# Create a macro for adding an export library
FUNCTION(DEXTEROUSCOOPERATION_ADD_EXPORT_LIBRARY library)
  SET(DEXTEROUSCOOPERATION_EXPORT_LIBRARIES ${DEXTEROUSCOOPERATION_EXPORT_LIBRARIES} ${library} CACHE INTERNAL "")
ENDFUNCTION()

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

################################################################################
# Install DcCommon headers
################################################################################
FILE(GLOB DcCommonFiles "${CMAKE_CURRENT_SOURCE_DIR}/src/DcCommon/*")
INSTALL(FILES ${DcCommonFiles} DESTINATION include)

###############################################################################
# Install config files
###############################################################################
INSTALL(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config/
  DESTINATION config/xml/DexterousCooperation
)

################################################################################
# Export script
################################################################################
INCLUDE(ExportDexterousCooperation)
