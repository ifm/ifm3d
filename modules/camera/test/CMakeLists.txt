if(NOT CMAKE_CROSSCOMPILING)
  file(COPY ${IFM3D_CAMERA_SOURCE_DIR}/test/test_wrapper.sh.in
    DESTINATION ${IFM3D_CAMERA_BINARY_DIR}/test
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
    ${IFM3D_CAMERA_BINARY_DIR}/test/test_wrapper.sh.in
    ${IFM3D_CAMERA_BINARY_DIR}/test/test_wrapper.sh
    @ONLY
    )
endif()

enable_testing()
add_subdirectory(${GTEST_CMAKE_DIR} gtest_bin)
file(GLOB TEST_SOURCES *.cpp)
add_executable(ifm3d-camera-tests ${TEST_SOURCES})
target_link_libraries(ifm3d-camera-tests
                      gtest_main
                      ifm3d_camera_shared
                      )

if(NOT CMAKE_CROSSCOMPILING)
  add_test(NAME ./test_wrapper.sh COMMAND ifm3d-camera-tests)
  add_custom_target(check COMMAND ./test_wrapper.sh DEPENDS ifm3d-camera-tests)
  add_custom_target(stress COMMAND ./test_wrapper.sh --gtest_repeat=100
                    DEPENDS ifm3d-camera-tests)
endif()
