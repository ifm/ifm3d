file(GLOB IFM3D_TOOLS_SOURCES CONFIGURE_DEPENDS *.cpp common/*.cpp legacy/*.cpp ovp8xx/*.cpp)
add_library(ifm3d_tools INTERFACE)
target_sources(ifm3d_tools INTERFACE ${IFM3D_TOOLS_SOURCES})

target_include_directories(ifm3d_tools
  INTERFACE
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_TOOLS_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_BINARY_DIR}/include>
)

if (BUILD_MODULE_FRAMEGRABBER)
  file(GLOB IFM3D_TOOLS_FG_SOURCES CONFIGURE_DEPENDS common/fg/*.cpp)
  target_sources(ifm3d_tools INTERFACE ${IFM3D_TOOLS_FG_SOURCES})
  target_include_directories(ifm3d_tools
    INTERFACE
      $<BUILD_INTERFACE:${IFM3D_FG_SOURCE_DIR}/include>
  )
endif()

if (BUILD_MODULE_SWUPDATER)
  file(GLOB IFM3D_TOOLS_SWUPDATER_SOURCES CONFIGURE_DEPENDS common/swupdater/*.cpp)
  target_sources(ifm3d_tools INTERFACE ${IFM3D_TOOLS_SWUPDATER_SOURCES})
  target_include_directories(ifm3d_tools
    INTERFACE
      $<BUILD_INTERFACE:${IFM3D_SWUPDATER_SOURCE_DIR}/include>
  )
endif()

target_link_libraries(
  ifm3d_tools
  INTERFACE
    ifm3d_common
    ifm3d_device
    CLI11::CLI11
    $<BUILD_INTERFACE:fmt::fmt-header-only>
    $<BUILD_INTERFACE:tinyxml2>
    $<BUILD_INTERFACE:httplib::httplib>
)

if (BUILD_MODULE_FRAMEGRABBER)
  target_link_libraries(ifm3d_tools INTERFACE ifm3d_framegrabber)
endif()

if (BUILD_MODULE_SWUPDATER)
  target_link_libraries(ifm3d_tools INTERFACE ifm3d_swupdater)
endif()
