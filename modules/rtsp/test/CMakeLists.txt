if(NOT CMAKE_CROSSCOMPILING)
  file(COPY ${IFM3D_RTSP_SOURCE_DIR}/test/${TEST_WRAPPER_IN}
    DESTINATION ${IFM3D_RTSP_BINARY_DIR}/test
    FILE_PERMISSIONS 
      OWNER_READ
      OWNER_WRITE
      OWNER_EXECUTE
      GROUP_READ
      GROUP_WRITE
      GROUP_EXECUTE
      WORLD_READ
      WORLD_EXECUTE
  )

  configure_file(
    ${IFM3D_RTSP_BINARY_DIR}/test/${TEST_WRAPPER_IN}
    ${IFM3D_RTSP_BINARY_DIR}/test/${TEST_WRAPPER}
    @ONLY
  )
endif()

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS *.cpp)
add_executable(ifm3d-rtsp-tests ${TEST_SOURCES})

target_compile_definitions(ifm3d-rtsp-tests PRIVATE
  IFM3D_STATIC_LIB
  # $<TARGET_FILE_DIR:...> resolves to the actual output directory of the
  # plugin binary, handling both single-config (Make/Ninja) and multi-config
  # (Visual Studio) generators correctly.
  PLUGIN_TEST_DIR="$<TARGET_FILE_DIR:ifm3d_plugin_ffmpeg>"
)

target_link_libraries(ifm3d-rtsp-tests
  gtest_main
  ifm3d_rtsp
  $<BUILD_INTERFACE:fmt::fmt-header-only>
)

# The test binary must run after the ffmpeg plugin .so/.dll is built.
add_dependencies(ifm3d-rtsp-tests ifm3d_plugin_ffmpeg)

if(NOT CMAKE_CROSSCOMPILING)
  add_custom_target(check_rtsp
    COMMAND ${IFM3D_RTSP_BINARY_DIR}/test/${TEST_WRAPPER} DEPENDS ifm3d-rtsp-tests
  )
  add_dependencies(check check_rtsp)
endif()
