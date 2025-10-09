ifm3d_module(framegrabber
  LINK_LIBRARIES 
    $<BUILD_INTERFACE:asio> 
    $<BUILD_INTERFACE:fmt::fmt-header-only>
  REQUIRED_MODULES
    device
)
