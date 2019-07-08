if(NOT CMAKE_CROSSCOMPILING)
  file(COPY ${IFM3D_OPENCV_SOURCE_DIR}/test/${TEST_WRAPPER_IN}
    DESTINATION ${IFM3D_OPENCV_BINARY_DIR}/test
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
    ${IFM3D_OPENCV_BINARY_DIR}/test/${TEST_WRAPPER_IN}
    ${IFM3D_OPENCV_BINARY_DIR}/test/${TEST_WRAPPER}
    @ONLY
    )
endif()

if (NOT TARGET glog::glog)
  find_library(LIB_glog NAMES glog)
else()
  set(LIB_glog glog::glog)
endif (NOT TARGET glog::glog)

file(GLOB TEST_SOURCES *.cpp)
add_executable(ifm3d-opencv-tests ${TEST_SOURCES})
target_link_libraries(ifm3d-opencv-tests
                      gtest_main
                      ifm3d_opencv
                      ifm3d_framegrabber
                      ifm3d_camera
                      ${LIB_glog}
                      )

if(NOT CMAKE_CROSSCOMPILING)
  add_custom_target(check_opencv
    COMMAND ${IFM3D_OPENCV_BINARY_DIR}/test/${TEST_WRAPPER} DEPENDS ifm3d-opencv-tests
    )
  add_dependencies(check check_opencv)
endif()
