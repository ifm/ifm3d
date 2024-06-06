file(GLOB IFM3D_PCICCLIENT_SOURCES CONFIGURE_DEPENDS *.cpp)
add_library(ifm3d_pcicclient INTERFACE)
target_sources(ifm3d_pcicclient INTERFACE ${IFM3D_PCICCLIENT_SOURCES})

target_include_directories(ifm3d_pcicclient
  INTERFACE
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_PCICCLIENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(
  ifm3d_pcicclient
  INTERFACE
    ifm3d_device
    $<BUILD_INTERFACE:fmt::fmt-header-only>
    $<BUILD_INTERFACE:asio>
  )