cmake_minimum_required(VERSION 3.11)
cmake_policy(SET CMP0048 NEW)

# Some generic settings
include(cmake/StandardProjectSettings.cmake)

# Make our cmake functions accessible
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake/modules)

set(ROOT_PROJECT_NAME IFM3D)
set(PROJECT_NAME ${ROOT_PROJECT_NAME})
find_program(GIT_EXECUTABLE git)
include(GetVersionFromGitTag) 

# Create aliases in order to avoid ${${..}} syntax in other files
set(ROOT_PROJECT_VERSION ${${ROOT_PROJECT_NAME}_VERSION})
set(ROOT_PROJECT_VERSION_MAJOR ${${ROOT_PROJECT_NAME}_VERSION_MAJOR})
set(ROOT_PROJECT_VERSION_MINOR ${${ROOT_PROJECT_NAME}_VERSION_MINOR})
set(ROOT_PROJECT_VERSION_PATCH ${${ROOT_PROJECT_NAME}_VERSION_PATCH})
set(ROOT_PROJECT_VERSION_TWEAK "${${ROOT_PROJECT_NAME}_VERSION_TWEAK}")
set(ROOT_PROJECT_VERSION_AHEAD ${${ROOT_PROJECT_NAME}_VERSION_AHEAD})
set(ROOT_PROJECT_VERSION_GIT_SHA ${${ROOT_PROJECT_NAME}_VERSION_GIT_SHA})
set(ROOT_PROJECT_VERSION_FULL_IN_CASE_OF_PATCHES ${${ROOT_PROJECT_NAME}_VERSION_FULL_IN_CASE_OF_PATCHES})
set(ROOT_PROJECT_VERSION_META ${${ROOT_PROJECT_NAME}_VERSION_META})
set(ROOT_PROJECT_VERSION_SIMPLE "${ROOT_PROJECT_VERSION_MAJOR}.${ROOT_PROJECT_VERSION_MINOR}.${ROOT_PROJECT_VERSION_PATCH}")
set(ROOT_PROJECT_DIR ${CMAKE_CURRENT_LIST_DIR})
project(${ROOT_PROJECT_NAME} LANGUAGES C CXX VERSION ${ROOT_PROJECT_VERSION_SIMPLE})

set(GIT_PROJECT_NAME "ifm3d")

# Some generic settings
include(cmake/StandardProjectSettings.cmake)

# Force an out-of-source build of the code
include(MacroOutOfSourceBuild)
macro_ensure_out_of_source_build(
  "Please build ${PROJECT_NAME} out-of-source")

# Conditionally turn on/off parts of the build (global-level)
option(BUILD_TESTS "Build unit tests" OFF)
option(BUILD_MODULE_FRAMEGRABBER "Build the framegrabber module" ON)
option(BUILD_MODULE_DESERIALIZE "Build the deserialize module" ON)
option(BUILD_MODULE_SWUPDATER "Build the swupdater module" ON)
option(BUILD_MODULE_PCICCLIENT "Build the pcicclient module" OFF)
option(BUILD_MODULE_TOOLS "Build the tools module" ON)
option(BUILD_MODULE_PYBIND11 "Build the python bindings module" OFF)
option(BUILD_SDK_PKG "Build install packages for development purposes" ON)
option(BUILD_SHARED_LIBS "Build modules as shared libraries" ON)
option(BUILD_DOC "Build documentation" OFF)
option(CREATE_WIN_INSTALLER "Create windows installer" OFF)
option(CREATE_PYTHON_STUBS "Create the Python stubs" OFF)

# Installation root
if(NOT WIN32)
  set(CPACK_SET_DESTDIR true)
endif()
set(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "CMake install prefix")
set(CPACK_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})

#add postfix d to all target binaries for Debug build
set(CMAKE_DEBUG_POSTFIX d)

# Installation paths
set(_lib lib CACHE STRING "Basename of the library-directory")
set(_bin bin CACHE STRING "Basename of the bin-directory")
set(_include include CACHE STRING "Basename of the include-directory")

# Where to find GTest
set(GTEST_CMAKE_DIR "/usr/src/gtest" CACHE STRING "GTest cmake project dir")

# Global compiler flags
set(CMAKE_CXX_EXTENSIONS OFF) # OFF -> -std=c++17, ON -> -std=gnu++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)
if(WIN32)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif(WIN32)

################################################
## Start deferring work to the submodules
################################################

# Bootstrap gtest
if(BUILD_TESTS)
  add_subdirectory(${GTEST_CMAKE_DIR} gtest_bin EXCLUDE_FROM_ALL)
  add_definitions(-DGTEST_LINKED_AS_SHARED_LIBRARY=1)
  add_custom_target(check)
  #var
  if(WIN32)
    set(TEST_WRAPPER_EXT "bat" )
  else()
    set(TEST_WRAPPER_EXT "sh" )
  endif()
  set(TEST_WRAPPER_IN "test_wrapper.${TEST_WRAPPER_EXT}.in" )
  set(TEST_WRAPPER "test_wrapper.${TEST_WRAPPER_EXT}" )
endif()

# Include third-party depencency libs
if(CREATE_WIN_INSTALLER)
  add_subdirectory(third-party EXCLUDE_FROM_ALL)

  # Copy DLLs and libs to CMAKE_INSTALL_PREFIX
  install(SCRIPT ${CMAKE_SOURCE_DIR}/cmake/windows_installer/cp_dll_script.cmake)

  #Include cpack.cmake script
  include(${CMAKE_SOURCE_DIR}/cmake/windows_installer/cpack.cmake)
else()
  add_subdirectory(third-party)
endif(CREATE_WIN_INSTALLER)

# Build sub-modules
add_subdirectory(modules/common)
list(APPEND DOXYGEN_MODULES "common")

add_subdirectory(modules/device)
list(APPEND DOXYGEN_MODULES "device")

if (BUILD_SHARED_LIBS)
  add_subdirectory(modules/libifm3d)
endif()

if(BUILD_MODULE_FRAMEGRABBER)
  add_definitions(-DBUILD_MODULE_FRAMEGRABBER=ON)
  add_subdirectory(modules/framegrabber)
  list(APPEND DOXYGEN_MODULES "framegrabber")
endif()

if(BUILD_MODULE_SWUPDATER)
  add_definitions(-DBUILD_MODULE_SWUPDATER=ON)
  add_subdirectory(modules/swupdater)
  list(APPEND DOXYGEN_MODULES "swupdater")
endif()

if(BUILD_MODULE_PCICCLIENT)
  add_subdirectory(modules/pcicclient)
  list(APPEND DOXYGEN_MODULES "pcicclient")
endif()

if(BUILD_MODULE_PYBIND11)
  add_definitions(-DBUILD_MODULE_PYBIND11)
  add_subdirectory(modules/pybind11)
endif()

if(BUILD_MODULE_DESERIALIZE)
  add_definitions(-DBUILD_MODULE_DESERIALIZE=ON)
  add_subdirectory(modules/deserialize)
  list(APPEND DOXYGEN_MODULES "deserialize")
endif()

if(BUILD_MODULE_TOOLS)
  add_subdirectory(modules/tools)
endif()

#################################################
# Documentation
#################################################
if(BUILD_DOC)
  add_subdirectory(doc)
endif()

#################################################
# Packaging stuff - for now, we build debs only
#################################################
if(NOT WIN32)
  execute_process(COMMAND uname -m
    OUTPUT_VARIABLE CPACK_DEBIAN_PACKAGE_ARCHITECTURE
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if (CPACK_DEBIAN_PACKAGE_ARCHITECTURE STREQUAL "x86_64")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
  elseif(CPACK_DEBIAN_PACKAGE_ARCHITECTURE STREQUAL "aarch64")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
  else()
    message(FATAL_ERROR "Unsupported architecture: ${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
  endif()

  set(CPACK_GENERATOR "DEB")
  set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

  set(CPACK_DEBIAN_PACKAGE_SECTION Libraries)
  set(CPACK_DEBIAN_PACKAGE_NAME ${GIT_PROJECT_NAME})
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "C++ sensor interface to ifm 3D cameras")
  set(CPACK_PACKAGE_VENDOR "ifm")
  set(CPACK_PACKAGE_CONTACT "support.robotics@ifm.com")
  set(CPACK_PACKAGE_VERSION_MAJOR ${IFM3D_VERSION_MAJOR})
  set(CPACK_PACKAGE_VERSION_MINOR ${IFM3D_VERSION_MINOR})
  set(CPACK_PACKAGE_VERSION_PATCH ${IFM3D_VERSION_PATCH})
  set(CPACK_PACKAGE_FILE_NAME
      "${GIT_PROJECT_NAME}_${IFM3D_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

  include(CPack)
endif(NOT WIN32)
