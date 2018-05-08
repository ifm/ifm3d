project(IFM3D_CAMERA CXX)
set(IFM3D_CAMERA_MODULE_NAME "libifm3d-camera")

file(COPY ${IFM3D_CAMERA_SOURCE_DIR}/include
  DESTINATION ${IFM3D_CAMERA_BINARY_DIR}
  )
configure_file(
  ${IFM3D_CAMERA_BINARY_DIR}/include/ifm3d/camera/version.h.in
  ${IFM3D_CAMERA_BINARY_DIR}/include/ifm3d/camera/version.h
  @ONLY
  )

################################################
## Process child CMakeLists.txt files
################################################
add_subdirectory(src)
if(BUILD_TESTS)
  add_subdirectory(test)
else()
  message(WARNING "Skipping ${IFM3D_CAMERA_MODULE_NAME} unit tests!")
endif()

################################################
## Manage installation process
################################################
if(BUILD_SDK_PKG)
  install(DIRECTORY ${IFM3D_CAMERA_BINARY_DIR}/include
    DESTINATION ${CPACK_INSTALL_PREFIX}
    COMPONENT camera
    PATTERN "*.in" EXCLUDE
    PATTERN ".git" EXCLUDE
    PATTERN "*~" EXCLUDE
    )
endif()
