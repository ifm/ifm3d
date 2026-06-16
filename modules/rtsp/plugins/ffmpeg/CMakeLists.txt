# ---------------------------------------------------------------------------
# FFmpeg RTSP decoder plugin
#
# Produces:
#   Linux/macOS : libifm3d_plugin_ffmpeg.so
#   Windows     : ifm3d_plugin_ffmpeg.dll
#
# The filename prefix is intentional — PluginManager scans for
# libifm3d_plugin_*.so (POSIX) and ifm3d_plugin_*.dll (Windows).
# ---------------------------------------------------------------------------

set(_ffmpeg_found FALSE)

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(FFMPEG QUIET IMPORTED_TARGET libavcodec libavutil)
  if(FFMPEG_FOUND)
    set(_ffmpeg_found TRUE)
  endif()
endif()

if(NOT _ffmpeg_found)
  find_path(FFMPEG_INCLUDE_DIR NAMES libavcodec/avcodec.h)
  find_library(FFMPEG_AVCODEC_LIB NAMES avcodec avcodec-61 avcodec-60 avcodec-59)
  find_library(FFMPEG_AVUTIL_LIB  NAMES avutil  avutil-59  avutil-58  avutil-57)
  if(FFMPEG_INCLUDE_DIR AND FFMPEG_AVCODEC_LIB AND FFMPEG_AVUTIL_LIB)
    set(_ffmpeg_found TRUE)
  endif()
endif()

if(NOT _ffmpeg_found)
  message(FATAL_ERROR
    "IFM3D_RTSP_BUILD_FFMPEG_PLUGIN=ON but FFmpeg (libavcodec/libavutil) was not found")
endif()

add_library(ifm3d_plugin_ffmpeg SHARED libifm3d_plugin_ffmpeg.cpp)

set_target_properties(ifm3d_plugin_ffmpeg PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  PREFIX ""
  OUTPUT_NAME
    $<IF:$<BOOL:${WIN32}>,ifm3d_plugin_ffmpeg,libifm3d_plugin_ffmpeg>
  RUNTIME_OUTPUT_DIRECTORY ${IFM3D_RTSP_BINARY_DIR}/plugins
  LIBRARY_OUTPUT_DIRECTORY ${IFM3D_RTSP_BINARY_DIR}/plugins
  ARCHIVE_OUTPUT_DIRECTORY ${IFM3D_RTSP_BINARY_DIR}/plugins
)

target_include_directories(ifm3d_plugin_ffmpeg PRIVATE
  ${IFM3D_RTSP_SOURCE_DIR}/include
)

if(FFMPEG_FOUND)
  target_link_libraries(ifm3d_plugin_ffmpeg PRIVATE PkgConfig::FFMPEG)
else()
  target_include_directories(ifm3d_plugin_ffmpeg PRIVATE ${FFMPEG_INCLUDE_DIR})
  target_link_libraries(ifm3d_plugin_ffmpeg PRIVATE
    ${FFMPEG_AVCODEC_LIB}
    ${FFMPEG_AVUTIL_LIB}
  )
endif()

if(BUILD_SDK_PKG)
  install(
    TARGETS ifm3d_plugin_ffmpeg
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT ifm3d
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT ifm3d
  )
endif()