ifm3d_module(rtsp
  LINK_LIBRARIES 
    $<BUILD_INTERFACE:asio> 
    $<BUILD_INTERFACE:fmt::fmt-header-only>
  REQUIRED_MODULES
    common
    device
    framegrabber
)

# ---------------------------------------------------------------------------
# Built-in FFmpeg H.264 decoder
#
# When enabled, the ffmpeg decoder is compiled directly into ifm3d_rtsp. It
# needs neither the FFmpeg libraries nor the FFmpeg headers at build time: the
# handful of functions, constants and struct fields it uses are mirrored in
# src/ffmpeg_lib.hpp and resolved at runtime via dlopen/dlsym (see
# src/ffmpeg_lib.cpp), which is validated by a layout probe before the decoder
# reports itself as available. A single binary can therefore run against
# whichever FFmpeg major version is installed, or fall back to the null decoder
# when none is present.
# ---------------------------------------------------------------------------
if(BUILD_MODULE_RTSP_FFMPEG)
  target_compile_definitions(ifm3d_rtsp INTERFACE IFM3D_RTSP_WITH_FFMPEG)
  # dlopen/dlsym live in libdl on older glibc; empty on Windows/macOS.
  target_link_libraries(ifm3d_rtsp INTERFACE ${CMAKE_DL_LIBS})
else()
  # ffmpeg decoder disabled: exclude its sources from the module.
  ifm3d_disable_sources(rtsp src/ffmpeg_decoder.cpp src/ffmpeg_lib.cpp)
endif()
