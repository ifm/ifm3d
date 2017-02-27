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
option(BUILD_MODULE_TOOLS "Build the tools module" ON)
option(BUILD_SDK_PKG "Build install packages for development purposes" ON)

# Installation root
set(CPACK_SET_DESTDIR true)
set(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "CMake install prefix")
set(CPACK_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})

# Where to find GTest
set(GTEST_CMAKE_DIR "/usr/src/gtest" CACHE STRING "GTest cmake project dir")

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
  set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
endif()

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
if(BUILD_MODULE_TOOLS)
  add_subdirectory(modules/tools)
endif()
