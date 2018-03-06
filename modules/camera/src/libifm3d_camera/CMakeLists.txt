find_package(glog QUIET CONFIG NAMES google-glog glog)
if (NOT TARGET glog::glog)
  find_library(LIB_glog NAMES glog)
else()
  set(LIB_glog glog::glog)
endif (NOT TARGET glog::glog)


if(NOT WIN32)
    find_library(LIB_xmlrpcxx NAMES xmlrpc++)
    find_library(LIB_xmlrpc_clientxx NAMES xmlrpc_client++)
else()
  find_package(xmlrpc-c CONFIG
    REQUIRED COMPONENTS xmlrpc xmlrpc_client xmlrpc++ xmlrpc_client++)
  set(LIB_xmlrpcxx xmlrpc-c::xmlrpc xmlrpc-c::xmlrpc_client)
  set(LIB_xmlrpc_clientxx xmlrpc-c::xmlrpc++ xmlrpc-c::xmlrpc_client++)
endif(NOT WIN32)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

file(GLOB IFM3D_CAMERA_SOURCES *.cpp)

add_library(ifm3d_camera_shared SHARED ${IFM3D_CAMERA_SOURCES})
set_property(TARGET ifm3d_camera_shared PROPERTY POSITION_INDEPENDENT_CODE 1)

target_link_libraries(
  ifm3d_camera_shared
  ${LIB_glog}
  ${LIB_xmlrpcxx}
  ${LIB_xmlrpc_clientxx}
  )

set_target_properties(ifm3d_camera_shared PROPERTIES
  VERSION ${IFM3D_VERSION}
  OUTPUT_NAME ifm3d_camera
  LINKER_LANGUAGE CXX
  )

if(WIN32)
    target_compile_definitions(ifm3d_camera_shared PRIVATE
        IFM3D_CAMERA_DLL_BUILD=1)
endif(WIN32)

install(TARGETS ifm3d_camera_shared
  RUNTIME DESTINATION ${_bin} COMPONENT camera
  LIBRARY DESTINATION ${_lib} COMPONENT camera
  ARCHIVE DESTINATION ${_lib} COMPONENT camera
  PUBLIC_HEADER DESTINATION ${_include} COMPONENT camera
  )
