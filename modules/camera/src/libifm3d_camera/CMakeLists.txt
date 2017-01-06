find_library(LIB_glog NAMES glog)
find_library(LIB_xmlrpcxx NAMES xmlrpc++)
find_library(LIB_xmlrpc_clientxx NAMES xmlrpc_client++)

file(GLOB IFM3D_CAMERA_SOURCES *.cpp)

add_library(ifm3d_camera OBJECT ${IFM3D_CAMERA_SOURCES})
set_property(TARGET ifm3d_camera PROPERTY POSITION_INDEPENDENT_CODE 1)

add_library(ifm3d_camera_shared SHARED $<TARGET_OBJECTS:ifm3d_camera>)
target_link_libraries(
  ifm3d_camera_shared
  ${LIB_glog}
  ${LIB_xmlrpcxx}
  ${LIB_xmlrpc_clientxx}
  )

set_target_properties(ifm3d_camera_shared PROPERTIES
  VERSION ${IFM3D_VERSION_STRING}
  OUTPUT_NAME ifm3d_camera
  LINKER_LANGUAGE CXX
  )

install(TARGETS ifm3d_camera_shared DESTINATION lib)
