project(IFM3D CXX)
cmake_minimum_required(VERSION 2.8.3)
set(GIT_PROJECT_NAME "ifm3d")

# Make our cmake functions accessible
set(CMAKE_MODULE_PATH
    ${IFM3D_SOURCE_DIR}/cmake/modules
    ${CMAKE_MODULE_PATH})

# Force an out-of-source build of the code
include(MacroOutOfSourceBuild)
macro_ensure_out_of_source_build(
  "Please build ${PROJECT_NAME} out-of-source")

# Library version
include(ifm3d_version)

# Conditionally turn on/off parts of the build (global-level)
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_MODULE_FRAMEGRABBER "Build the framegrabber module" ON)
option(BUILD_MODULE_IMAGE "Build the image module" ON)
option(BUILD_MODULE_SIMPLE_IMAGE "Build the simple_image module" ON)
option(BUILD_MODULE_TOOLS "Build the tools module" ON)
option(BUILD_MODULE_EXAMPLES "Build the examples module" ON)
option(BUILD_SDK_PKG "Build install packages for development purposes" ON)

# Installation root
set(CPACK_SET_DESTDIR true)
set(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "CMake install prefix")
set(CPACK_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})

# Where to find GTest
set(GTEST_CMAKE_DIR "/usr/src/gtest" CACHE STRING "GTest cmake project dir")

# Global compiler flags
set(CMAKE_BUILD_TYPE Release) # Release or Debug
set(CMAKE_CXX_EXTENSIONS OFF) # OFF -> -std=c++11, ON -> -std=gnu++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED true)
if(WIN32)
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -std=c++11")
else(WIN32)
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Wno-literal-suffix -s -std=c++11")
endif(WIN32)
# Bootstrap gtest
if(BUILD_TESTS)
  add_subdirectory(${GTEST_CMAKE_DIR} gtest_bin)
  add_custom_target(check)
endif()

if(WIN32)
add_subdirectory(3rdparty)
# This is a hack as long as the xmlrpc-c find script is broken under windows
include_directories(${CMAKE_CURRENT_BINARY_DIR}/3rdParty/xmlrpc-c/include)
include_directories(${CMAKE_SOURCE_DIR}/3rdParty/xmlrpc-c/include)
endif(WIN32)

# Build sub-modules
add_subdirectory(modules/camera)
if(BUILD_MODULE_FRAMEGRABBER)
  add_definitions(-DBUILD_MODULE_FRAMEGRABBER=ON)
  add_subdirectory(modules/framegrabber)
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
if(BUILD_MODULE_SIMPLE_IMAGE)
  add_definitions(-DBUILD_MODULE_SIMPLE_IMAGE=ON)
  add_subdirectory(modules/simple_image)
endif()
if(BUILD_MODULE_TOOLS)
  add_subdirectory(modules/tools)
endif()

if(BUILD_MODULE_EXAMPLES)
  add_subdirectory(modules/examples)
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
set(CPACK_PACKAGE_VENDOR "Love Park Robotics, LLC")
set(CPACK_PACKAGE_CONTACT "Tom Panzarella <tom@loveparkrobotics.com>")
set(CPACK_PACKAGE_VERSION_MAJOR ${IFM3D_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${IFM3D_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${IFM3D_VERSION_PATCH})
set(CPACK_PACKAGE_FILE_NAME
    "${GIT_PROJECT_NAME}_${IFM3D_VERSION_STRING}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

# dynamically create dependencies of the various modules
#set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON) # <-- Good idea, doesn't work for us
add_custom_target(repackage
  COMMAND ${IFM3D_BINARY_DIR}/ifm3d-dpkg-deps.py ${IFM3D_BINARY_DIR}/*.deb
  )

include(CPack)
