################################################
## Target: libifm3d_framegrabber.so
################################################
file(GLOB IFM3D_FG_SOURCES CONFIGURE_DEPENDS *.cpp *.hpp)
add_library(ifm3d_framegrabber INTERFACE)
target_sources(ifm3d_framegrabber INTERFACE ${IFM3D_FG_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/algo/o3r_uncompress_di.cpp)

#------------------
# Compiler settings
#------------------
target_include_directories(ifm3d_framegrabber
  INTERFACE
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_FG_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/algo>
)

target_link_libraries(
  ifm3d_framegrabber
  INTERFACE
    ifm3d_device
    $<BUILD_INTERFACE:asio>
    $<BUILD_INTERFACE:fmt::fmt-header-only>
)