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
# does NOT link against libavcodec/libavutil: the required functions are
# resolved at runtime via dlopen/dlsym (see src/ffmpeg_lib.cpp), so only the
# FFmpeg *headers* are needed at build time and a single binary can run against
# whichever FFmpeg major version is installed (or fall back to the null decoder
# when none is present).
# ---------------------------------------------------------------------------
if(BUILD_MODULE_RTSP_FFMPEG)
  set(_ffmpeg_include_dirs "")
  set(_ffmpeg_headers_found FALSE)

  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(FFMPEG QUIET libavcodec libavutil)
    if(FFMPEG_FOUND)
      set(_ffmpeg_include_dirs ${FFMPEG_INCLUDE_DIRS})
      set(_ffmpeg_headers_found TRUE)
    endif()
  endif()

  if(NOT _ffmpeg_headers_found)
    find_path(FFMPEG_AVCODEC_INCLUDE_DIR NAMES libavcodec/avcodec.h)
    if(FFMPEG_AVCODEC_INCLUDE_DIR)
      set(_ffmpeg_include_dirs ${FFMPEG_AVCODEC_INCLUDE_DIR})
      set(_ffmpeg_headers_found TRUE)
    endif()
  endif()

  if(NOT _ffmpeg_headers_found)
    message(FATAL_ERROR
      "BUILD_MODULE_RTSP_FFMPEG=ON but the FFmpeg headers "
      "(libavcodec/avcodec.h) were not found")
  endif()

  target_compile_definitions(ifm3d_rtsp INTERFACE IFM3D_RTSP_WITH_FFMPEG)
  if(_ffmpeg_include_dirs)
    target_include_directories(ifm3d_rtsp INTERFACE ${_ffmpeg_include_dirs})
  endif()
  # dlopen/dlsym live in libdl on older glibc; empty on Windows/macOS.
  target_link_libraries(ifm3d_rtsp INTERFACE ${CMAKE_DL_LIBS})
else()
  # ffmpeg decoder disabled: exclude its sources from the module.
  ifm3d_disable_sources(rtsp src/ffmpeg_decoder.cpp src/ffmpeg_lib.cpp)
endif()
