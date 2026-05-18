# ---------------------------------------------------------------------------
# ifm3d sample decoder plugin
#
# Produces:
#   Linux/macOS : libifm3d_plugin_sample.so
#   Windows     : ifm3d_plugin_sample.dll
#
# The filename prefix is intentional — PluginManager scans for
# libifm3d_plugin_*.so (POSIX) and ifm3d_plugin_*.dll (Windows).
# ---------------------------------------------------------------------------

add_library(ifm3d_plugin_sample SHARED libifm3d_plugin_sample.cpp)

# Strip the default "lib" prefix on all platforms and set the correct
# platform-specific output name so PluginManager's naming filter matches.
set_target_properties(ifm3d_plugin_sample PROPERTIES
  PREFIX ""
  OUTPUT_NAME
    $<IF:$<BOOL:${WIN32}>,ifm3d_plugin_sample,libifm3d_plugin_sample>
)

# The plugin only needs the public C-ABI header from the rtsp module.
target_include_directories(ifm3d_plugin_sample PRIVATE
  ${IFM3D_RTSP_SOURCE_DIR}/include
)

# Place the built plugin into <build>/plugins/ so tests and manual runs
# can point PluginManager at a predictable relative path.
set_target_properties(ifm3d_plugin_sample PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${IFM3D_RTSP_BINARY_DIR}/plugins
  LIBRARY_OUTPUT_DIRECTORY ${IFM3D_RTSP_BINARY_DIR}/plugins
  ARCHIVE_OUTPUT_DIRECTORY ${IFM3D_RTSP_BINARY_DIR}/plugins
)

# Optionally install the sample plugin alongside the other ifm3d libraries.
if(BUILD_SDK_PKG)
  install(
    TARGETS ifm3d_plugin_sample
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}  COMPONENT ifm3d
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}  COMPONENT ifm3d
  )
endif()