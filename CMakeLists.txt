cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
# Manage version number here: Major.Minor.Patch (we don't use "tweak")
project(IFM3D VERSION 0.18.0 LANGUAGES CXX)
set(GIT_PROJECT_NAME "ifm3d")

# Make our cmake functions accessible
set(CMAKE_MODULE_PATH
    ${IFM3D_SOURCE_DIR}/cmake/modules
    ${CMAKE_MODULE_PATH})

# Force an out-of-source build of the code
include(MacroOutOfSourceBuild)
macro_ensure_out_of_source_build(
  "Please build ${PROJECT_NAME} out-of-source")

# Conditionally turn on/off parts of the build (global-level)
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_MODULE_FRAMEGRABBER "Build the framegrabber module" ON)
option(BUILD_MODULE_SWUPDATER "Build the swupdater module" ON)
option(BUILD_MODULE_IMAGE "Build the image module" ON)
option(BUILD_MODULE_OPENCV "Build the OpenCV-only image container" OFF)
option(BUILD_MODULE_PCICCLIENT "Build the pcicclient module" OFF)
option(BUILD_MODULE_TOOLS "Build the tools module" ON)
option(BUILD_MODULE_PYBIND11 "Build the python bindings module" OFF)
option(BUILD_SDK_PKG "Build install packages for development purposes" ON)
option(FORCE_OPENCV3 "Force the build to require OpenCV 3" OFF)
option(FORCE_OPENCV2 "Force the build to require OpenCV 2.4" OFF)
option(BUILD_SHARED_LIBS "Build modules as shared libraries" ON)

# Installation root
set(CPACK_SET_DESTDIR true)
set(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "CMake install prefix")
set(CPACK_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})

# Installation paths
set(_lib lib CACHE STRING "Basename of the library-directory")
set(_bin bin CACHE STRING "Basename of the bin-directory")
set(_include include CACHE STRING "Basename of the include-directory")

# Where to find GTest
set(GTEST_CMAKE_DIR "/usr/src/gtest" CACHE STRING "GTest cmake project dir")

# Global compiler flags
set(CMAKE_BUILD_TYPE Release) # Release or Debug
set(CMAKE_CXX_EXTENSIONS OFF) # OFF -> -std=c++14, ON -> -std=gnu++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED true)
if(WIN32)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-DBOOST_ALL_NO_LIB)
else(WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-literal-suffix -s")
endif(WIN32)

# create the cmake-package files
if(BUILD_SDK_PKG)
  include(CMakePackageConfigHelpers)

  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/ifm3d-config-version.cmake
    VERSION ${IFM3D_VERSION}
    COMPATIBILITY AnyNewerVersion
    )

  set(CONFIG_PACKAGE_LOCATION ${_lib}/cmake/ifm3d-${IFM3D_VERSION})
endif()

################################################
## Start deferring work to the submodules
################################################

# Bootstrap gtest
if(BUILD_TESTS)
  add_subdirectory(${GTEST_CMAKE_DIR} gtest_bin EXCLUDE_FROM_ALL)
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

# Build sub-modules
add_subdirectory(modules/camera)

if(BUILD_MODULE_FRAMEGRABBER)
  add_definitions(-DBUILD_MODULE_FRAMEGRABBER=ON)
  add_subdirectory(modules/framegrabber)
endif()
if(BUILD_MODULE_SWUPDATER)
  add_definitions(-DBUILD_MODULE_SWUPDATER=ON)
  add_subdirectory(modules/swupdater)
endif()
if(BUILD_MODULE_IMAGE)
  # `image` relies on `framegrabber`, in the event the user
  # explicitly disabled the `framegrabber` but not `image`
  # we are going to let the compiler convey the message (for now) instead of
  # doing some implicit CMake stuff that will be hard to maintain and
  # confuse the end-user.
  add_definitions(-DBUILD_MODULE_IMAGE=ON)
  add_subdirectory(modules/image)
endif()
if(BUILD_MODULE_OPENCV)
  add_definitions(-DBUILD_MODULE_OPENCV=ON)
  add_subdirectory(modules/opencv)
endif()
if(BUILD_MODULE_PCICCLIENT)
  add_subdirectory(modules/pcicclient)
endif()
if(BUILD_MODULE_TOOLS)
  add_subdirectory(modules/tools)
endif()
if(BUILD_MODULE_PYBIND11)
  add_definitions(-DBUILD_MODULE_PYBIND11)
  add_subdirectory(modules/pybind11)
endif()

################################################
## Sub-modules are done, finish up the cmake package
################################################
if(BUILD_SDK_PKG)
  configure_package_config_file(
    ${CMAKE_CURRENT_LIST_DIR}/cmake/modules/ifm3d-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ifm3d-config.cmake
    INSTALL_DESTINATION ${CONFIG_PACKAGE_LOCATION}
    )

  # NOTE: we use 'COMPONENT camera' here as it is the one
  # "required" module in our stack
  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/ifm3d-config-version.cmake
          ${CMAKE_CURRENT_BINARY_DIR}/ifm3d-config.cmake
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
    COMPONENT camera
    )
endif()

#################################################
# Packaging stuff - for now, we build debs only
#################################################
file(COPY ${IFM3D_SOURCE_DIR}/cmake/utils/ifm3d-dpkg-deps.py.in
  DESTINATION ${IFM3D_BINARY_DIR}
  FILE_PERMISSIONS OWNER_READ
                   OWNER_WRITE
                   OWNER_EXECUTE
                   GROUP_READ
                   GROUP_EXECUTE
                   WORLD_READ
                   WORLD_EXECUTE
  )
configure_file(
  ${IFM3D_BINARY_DIR}/ifm3d-dpkg-deps.py.in
  ${IFM3D_BINARY_DIR}/ifm3d-dpkg-deps.py
  @ONLY
  )

# Turn on component-based installation
set(CPACK_DEB_COMPONENT_INSTALL ON)
set(CPACK_GENERATOR "DEB")

# Package architecture
find_program(DPKG_CMD dpkg)
if(NOT CMAKE_CROSSCOMPILING)
  if(NOT DPKG_CMD)
    message(STATUS
      "Cannot find dpkg in your path, default to ${CMAKE_SYSTEM_PROCESSOR}.")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
  else()
    execute_process(COMMAND "${DPKG_CMD}" --print-architecture
      OUTPUT_VARIABLE CPACK_DEBIAN_PACKAGE_ARCHITECTURE
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )
  endif(NOT DPKG_CMD)
else()
  # big assumption here
  set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
endif()

set(CPACK_DEBIAN_PACKAGE_SECTION Libraries)
set(CPACK_DEBIAN_PACKAGE_NAME ${GIT_PROJECT_NAME})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "C++ sensor interface to ifm 3D cameras")
set(CPACK_PACKAGE_VENDOR "ifm robotics")
set(CPACK_PACKAGE_CONTACT "Sean Kelly <sean.kelly@ifm.com>")
set(CPACK_PACKAGE_VERSION_MAJOR ${IFM3D_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${IFM3D_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${IFM3D_VERSION_PATCH})
set(CPACK_PACKAGE_FILE_NAME
    "${GIT_PROJECT_NAME}_${IFM3D_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

# dynamically create dependencies of the various modules
#set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON) # <-- Good idea, doesn't work for us
add_custom_target(repackage
  COMMAND ${IFM3D_BINARY_DIR}/ifm3d-dpkg-deps.py ${IFM3D_BINARY_DIR}/*.deb
  )

include(CPack)
