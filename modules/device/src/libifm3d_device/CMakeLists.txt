file(GLOB IFM3D_DEVICE_SOURCES CONFIGURE_DEPENDS *.cpp  ../../include/ifm3d/device)
add_library(ifm3d_device INTERFACE )
target_sources(ifm3d_device INTERFACE ${IFM3D_DEVICE_SOURCES})

target_include_directories(ifm3d_device
  INTERFACE
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_DEVICE_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_DEVICE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  )
    
target_include_directories(ifm3d_device SYSTEM
  INTERFACE
    ${XMLRPC_INCLUDE_DIRS}
)
  
target_link_libraries(
  ifm3d_device
  INTERFACE
    ifm3d_common
    CURL::libcurl
    xmlrpc++
    xmlrpc
    xmlrpc_client++
    xmlrpc_client
    $<BUILD_INTERFACE:asio>
    $<BUILD_INTERFACE:fmt::fmt-header-only>
)