################################################
## Bring in dependent projects
################################################

# NOTE: The `FindXMLRPC.cmake` file does not respect cross-compilation
# environments or windows builds. so, we make the assumption the includes are
# in a standard directory in your sysroot (for cross compiling) or included via
# a custom xmlrpc-c library on Windows (See Windows build instructions).
if (NOT WIN32 AND NOT CMAKE_CROSSCOMPILING)
  find_package(XMLRPC REQUIRED c++ client)
endif()

find_package(glog QUIET CONFIG NAMES google-glog glog)
if (NOT TARGET glog::glog)
  find_library(LIB_glog NAMES glog)
else()
  set(LIB_glog glog::glog)
endif (NOT TARGET glog::glog)

if(NOT WIN32)
    find_library(LIB_xmlrpcxx NAMES xmlrpc++)
    find_library(LIB_xmlrpc NAMES xmlrpc)
    find_library(LIB_xmlrpc_clientxx NAMES xmlrpc_client++)
    find_library(LIB_xmlrpc_client NAMES xmlrpc_client)
else()
  find_package(
    xmlrpc-c CONFIG
    REQUIRED COMPONENTS xmlrpc xmlrpc_client xmlrpc++ xmlrpc_client++
    )
  set(LIB_xmlrpcxx xmlrpc-c::xmlrpc++)
  set(LIB_xmlrpc xmlrpc-c::xmlrpc)
  set(LIB_xmlrpc_clientxx xmlrpc-c::xmlrpc_client++)
  set(LIB_xmlrpc_client xmlrpc-c::xmlrpc_client)
endif(NOT WIN32)

################################################
## Target: libifm3d_camera.so
################################################
file(GLOB IFM3D_CAMERA_SOURCES *.cpp)
add_library(ifm3d_camera ${IFM3D_CAMERA_SOURCES})
set_property(TARGET ifm3d_camera PROPERTY POSITION_INDEPENDENT_CODE 1)

#------------------
# Compiler settings
#------------------
target_include_directories(ifm3d_camera
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_BINARY_DIR}/include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${XMLRPC_INCLUDE_DIRS}
    )

# NOTE: we cannot use the following due to only requiring cmake 3.5
#
#target_compile_features(ifm3d_camera_shared
#  PUBLIC "cxx_std_${CMAKE_CXX_STANDARD}"
#  )
#
# ... so, we do a more ugly thing here instead
#
target_compile_options(ifm3d_camera
  PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-std=c++${CMAKE_CXX_STANDARD}>
  )

if(WIN32)
  target_compile_definitions(
    ifm3d_camera PRIVATE
    IFM3D_CAMERA_DLL_BUILD=1
    )
endif(WIN32)

#------------------
# Linker settings
#------------------
target_link_libraries(
  ifm3d_camera
  PRIVATE
    ${LIB_glog}
    ${LIB_xmlrpc_clientxx}
    ${LIB_xmlrpc_client}
    ${LIB_xmlrpcxx}
    ${LIB_xmlrpc}
  )

set_target_properties(
  ifm3d_camera PROPERTIES
  VERSION ${IFM3D_VERSION}
  EXPORT_NAME camera
  OUTPUT_NAME ifm3d_camera
  LINKER_LANGUAGE CXX
  )

#------------------
# Installation
#------------------
install(TARGETS ifm3d_camera
  EXPORT ifm3d-camera-targets
  RUNTIME DESTINATION ${_bin} COMPONENT camera
  LIBRARY DESTINATION ${_lib} COMPONENT camera
  ARCHIVE DESTINATION ${_lib} COMPONENT camera
  PUBLIC_HEADER DESTINATION ${_include} COMPONENT camera
  )

if(BUILD_SDK_PKG)
  install(EXPORT ifm3d-camera-targets
    NAMESPACE ifm3d::
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
    COMPONENT camera
    )
endif()
