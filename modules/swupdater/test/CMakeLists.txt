if(NOT CMAKE_CROSSCOMPILING)
  file(COPY ${IFM3D_SWUPDATER_SOURCE_DIR}/test/${TEST_WRAPPER_IN}
    DESTINATION ${IFM3D_SWUPDATER_BINARY_DIR}/test
    FILE_PERMISSIONS OWNER_READ
                     OWNER_WRITE
                     OWNER_EXECUTE
                     GROUP_READ
                     GROUP_WRITE
                     GROUP_EXECUTE
                     WORLD_READ
                     WORLD_EXECUTE
                     )

  configure_file(
    ${IFM3D_SWUPDATER_BINARY_DIR}/test/${TEST_WRAPPER_IN}
    ${IFM3D_SWUPDATER_BINARY_DIR}/test/${TEST_WRAPPER}
    @ONLY
    )
endif()

file(GLOB TEST_SOURCES *.cpp)
add_executable(ifm3d-swupdater-tests ${TEST_SOURCES})
target_link_libraries(ifm3d-swupdater-tests
                      gtest_main
                      ifm3d_common
                      ifm3d_device
                      ifm3d_swupdater
                      $<BUILD_INTERFACE:fmt::fmt-header-only>
                      )

if(NOT CMAKE_CROSSCOMPILING)
  add_custom_target(check_swupdater
    COMMAND ${IFM3D_SWUPDATER_BINARY_DIR}/test/${TEST_WRAPPER} DEPENDS ifm3d-swupdater-tests
    )
  add_dependencies(check check_swupdater)
endif()
