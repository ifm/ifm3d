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
set(ROOT_PROJECT_VERSION_AHEAD ${${ROOT_PROJECT_NAME}_VERSION_AHEAD})
set(ROOT_PROJECT_VERSION_GIT_SHA ${${ROOT_PROJECT_NAME}_VERSION_GIT_SHA})
set(ROOT_PROJECT_VERSION_FULL_IN_CASE_OF_PATCHES ${${ROOT_PROJECT_NAME}_VERSION_FULL_IN_CASE_OF_PATCHES})
set(ROOT_PROJECT_VERSION_META ${${ROOT_PROJECT_NAME}_VERSION_META})

project(${ROOT_PROJECT_NAME} LANGUAGES C CXX VERSION ${ROOT_PROJECT_VERSION})

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
option(BUILD_EXAMPLES "Build the examples" OFF)
option(BUILD_DOC "Build documentation" OFF)
option(BUILD_IN_DEPS "download and build dependencies" ON)
option(CREATE_WIN_INSTALLER "Create windows installer" OFF)

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
  add_definitions(-DBOOST_ALL_NO_LIB)
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
add_subdirectory(modules/device)
list(APPEND DOXYGEN_MODULES "device")

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

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(BUILD_MODULE_TOOLS)
  add_subdirectory(modules/tools)
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

  # NOTE: we use 'COMPONENT device' here as it is the one
  # "required" module in our stack
  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/ifm3d-config-version.cmake
          ${CMAKE_CURRENT_BINARY_DIR}/ifm3d-config.cmake
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
    COMPONENT device
    )
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
  get_cmake_property(CPACK_COMPONENTS_ALL COMPONENTS)
  list(REMOVE_ITEM CPACK_COMPONENTS_ALL "Unspecified")

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
  set(CPACK_PACKAGE_VENDOR "ifm")
  set(CPACK_PACKAGE_CONTACT "support.robotics@ifm.com")
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
endif(NOT WIN32)
