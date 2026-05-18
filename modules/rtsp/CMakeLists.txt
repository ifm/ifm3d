ifm3d_module(rtsp
  LINK_LIBRARIES 
    $<BUILD_INTERFACE:asio> 
    $<BUILD_INTERFACE:fmt::fmt-header-only>
  REQUIRED_MODULES
    common
    # device
    # framagrabber
)

add_subdirectory(plugins)
