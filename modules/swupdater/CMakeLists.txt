ifm3d_module(swupdater
  LINK_LIBRARIES 
    $<BUILD_INTERFACE:asio>
    $<BUILD_INTERFACE:websocketpp_wrapper>
    $<BUILD_INTERFACE:fmt::fmt-header-only>
  REQUIRED_MODULES
    device
)