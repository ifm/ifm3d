file(GLOB IFM3D_SWUPDATER_SOURCES CONFIGURE_DEPENDS *.cpp)
add_library(ifm3d_swupdater INTERFACE)
target_sources(ifm3d_swupdater INTERFACE ${IFM3D_SWUPDATER_SOURCES})

target_include_directories(ifm3d_swupdater
  INTERFACE
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_SWUPDATER_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(
  ifm3d_swupdater
  INTERFACE 
    ifm3d_device
    CURL::libcurl
    $<BUILD_INTERFACE:asio>
    $<BUILD_INTERFACE:websocketpp_wrapper>
    $<BUILD_INTERFACE:fmt::fmt-header-only>
)