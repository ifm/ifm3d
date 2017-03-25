project(IFM3D CXX)
cmake_minimum_required(VERSION 3.5.1)
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
option(BUILD_MODULE_TOOLS "Build the tools module" ON)
option(BUILD_SDK_PKG "Build install packages for development purposes" ON)

# Installation root
set(CPACK_SET_DESTDIR true)
set(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "CMake install prefix")
set(CPACK_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})

# Where to find GTest
set(GTEST_CMAKE_DIR "/usr/src/gtest" CACHE STRING "GTest cmake project dir")

# Global compiler flags
set(CMAKE_BUILD_TYPE Release) # Release or Debug
set(CMAKE_CXX_EXTENSIONS OFF) # OFF -> -std=c++14, ON -> -std=gnu++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Wno-literal-suffix -s") # create stripped binaries

# Bootstrap gtest
if(BUILD_TESTS)
  add_subdirectory(${GTEST_CMAKE_DIR} gtest_bin)
  add_custom_target(check)
endif()

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
  add_subdirectory(modules/image)
endif()
if(BUILD_MODULE_TOOLS)
  add_subdirectory(modules/tools)
endif()

#################################################
# Packaging stuff - for now, we build debs only
#################################################
# file(COPY ${IFM3D_SOURCE_DIR}/cmake/ifm3d/IFM3DConfig.cmake.in
#   DESTINATION ${IFM3D_BINARY_DIR}
#   )
# configure_file(
#   ${IFM3D_BINARY_DIR}/IFM3DConfig.cmake.in
#   ${IFM3D_BINARY_DIR}/IFM3DConfig.cmake
#   @ONLY
#   )
# file(COPY ${IFM3D_SOURCE_DIR}/cmake/ifm3d/IFM3DConfigVersion.cmake.in
#   DESTINATION ${IFM3D_BINARY_DIR}
#   )
# configure_file(
#   ${IFM3D_BINARY_DIR}/IFM3DConfigVersion.cmake.in
#   ${IFM3D_BINARY_DIR}/IFM3DConfigVersion.cmake
#   @ONLY
#   )
# install(FILES
#   ${IFM3D_BINARY_DIR}/IFM3DConfig.cmake
#   ${IFM3D_BINARY_DIR}/IFM3DConfigVersion.cmake
#   DESTINATION lib/ifm3d
#   COMPONENT camera
#   )


# Turn on component-based installation
set(CPACK_DEB_COMPONENT_INSTALL ON)
set(CPACK_GENERATOR "DEB")

# Package architecture
if(NOT CMAKE_CROSSCOMPILING)
  find_program(DPKG_CMD dpkg)
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
string(CONCAT CPACK_PACKAGE_FILE_NAME
  "${GIT_PROJECT_NAME}" "_"
  "${IFM3D_VERSION_STRING}" "_"
  "${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

# dependencies of the various modules
#set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON) <-- good idea, doesn't work
string(CONCAT CAMERA_MODULE_DEPS
  "libc6 (>= 2.14), "
  "libgcc1 (>= 1:3.0), "
  "libgoogle-glog0v5, "
  "libstdc++6 (>= 5.2), "
  "libxmlrpc-c++8v5")
set(CPACK_DEBIAN_CAMERA_PACKAGE_DEPENDS ${CAMERA_MODULE_DEPS})

string(CONCAT FG_MODULE_DEPS
  "libboost-system1.58.0, "
  "libc6 (>= 2.14), "
  "libgcc1 (>= 1:3.0), "
  "libgoogle-glog0v5, "
  "libstdc++6 (>= 5.2), "
  "ifm3d-camera (= ${IFM3D_VERSION_STRING})")
set(CPACK_DEBIAN_FRAMEGRABBER_PACKAGE_DEPENDS ${FG_MODULE_DEPS})

string(CONCAT IMAGE_MODULE_DEPS
  "libc6 (>= 2.4), "
  "libgcc1 (>= 1:3.0), "
  "libgoogle-glog0v5, "
  "libstdc++6 (>= 5.2), "
  "ros-kinetic-opencv3, "
  "ifm3d-camera (= ${IFM3D_VERSION_STRING}), "
  "ifm3d-framegrabber (= ${IFM3D_VERSION_STRING})")
set(CPACK_DEBIAN_IMAGE_PACKAGE_DEPENDS ${IMAGE_MODULE_DEPS})

string(CONCAT TOOLS_MODULE_DEPS
  "libc6 (>= 2.4), "
  "libgcc1 (>= 1:3.0), "
  "libstdc++6 (>= 5.2), "
  "libboost-program-options1.58.0, "
  "ifm3d-camera (= ${IFM3D_VERSION_STRING}), "
  "ifm3d-framegrabber (= ${IFM3D_VERSION_STRING})")
set(CPACK_DEBIAN_TOOLS_PACKAGE_DEPENDS ${TOOLS_MODULE_DEPS})

include(CPack)
