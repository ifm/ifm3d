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
  $<$<BOOL:${BUILD_MODULE_RTSP_FFMPEG}>:IFM3D_RTSP_WITH_FFMPEG>
)

target_link_libraries(ifm3d-rtsp-tests
  gtest_main
  ifm3d_rtsp
  ifm3d_deserialize
  $<BUILD_INTERFACE:fmt::fmt-header-only>
)

if(NOT CMAKE_CROSSCOMPILING)
  add_custom_target(check_rtsp
    COMMAND ${IFM3D_RTSP_BINARY_DIR}/test/${TEST_WRAPPER} DEPENDS ifm3d-rtsp-tests
  )
  add_dependencies(check check_rtsp)
endif()
