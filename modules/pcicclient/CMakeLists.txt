ifm3d_module(pcicclient
  LINK_LIBRARIES 
    $<BUILD_INTERFACE:fmt::fmt-header-only>
    $<BUILD_INTERFACE:asio>
  REQUIRED_MODULES
    device
)


