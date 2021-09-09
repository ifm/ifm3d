if(NOT CMAKE_CROSSCOMPILING)
  file(COPY ${IFM3D_STLIMAGE_SOURCE_DIR}/test/${TEST_WRAPPER_IN}
    DESTINATION ${IFM3D_STLIMAGE_BINARY_DIR}/test
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
    ${IFM3D_STLIMAGE_BINARY_DIR}/test/${TEST_WRAPPER_IN}
    ${IFM3D_STLIMAGE_BINARY_DIR}/test/${TEST_WRAPPER}
    @ONLY
    )
endif()

if(NOT BUILD_IN_DEPS)
find_package(glog QUIET CONFIG NAMES google-glog glog)
if (NOT TARGET glog::glog)
  find_library(LIB_glog NAMES glog)
else()
  set(LIB_glog glog::glog)
endif (NOT TARGET glog::glog)
else()
set( LIB_glog glog)
endif(NOT BUILD_IN_DEPS)

add_executable(ifm3d-stlimage-tests 
               ${CMAKE_CURRENT_SOURCE_DIR}/ifm3d-stlimage-testrunner.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/ifm3d-stlimage-tests.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/ifm3d-stlimage-image-tests.cpp
              )
target_link_libraries(ifm3d-stlimage-tests
                      gtest_main
                      ifm3d_stlimage
                      ifm3d_framegrabber
                      ifm3d_camera
                      ${LIB_glog}
                      )	

if(NOT CMAKE_CROSSCOMPILING)
  add_custom_target(check_stlimage
    COMMAND ${IFM3D_STLIMAGE_BINARY_DIR}/test/${TEST_WRAPPER} DEPENDS ifm3d-stlimage-tests
    )
  add_dependencies(check check_stlimage)
endif()
