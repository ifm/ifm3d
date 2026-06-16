# ---------------------------------------------------------------------------
# RTSP video decoder plugins
#
# Each plugin lives in its own subdirectory and is built as an independent
# shared library named libifm3d_plugin_<name>.so / ifm3d_plugin_<name>.dll
# that the runtime PluginManager discovers and loads on demand.
# ---------------------------------------------------------------------------

option(IFM3D_RTSP_BUILD_FFMPEG_PLUGIN   "Build FFmpeg RTSP decoder plugin"   ON)
# option(IFM3D_RTSP_BUILD_OPENH264_PLUGIN "Build OpenH264 RTSP decoder plugin" OFF)

if(IFM3D_RTSP_BUILD_FFMPEG_PLUGIN)
  add_subdirectory(ffmpeg)
endif()

# if(IFM3D_RTSP_BUILD_OPENH264_PLUGIN)
#   add_subdirectory(openh264)
# endif()
