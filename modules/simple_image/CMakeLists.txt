project(IFM3D_SIMPLE_IMAGE CXX)
set(IFM3D_SIMPLE_IMAGE_MODULE_NAME "libifm3d-simple-image")

file(COPY ${IFM3D_SIMPLE_IMAGE_SOURCE_DIR}/include
  DESTINATION ${IFM3D_SIMPLE_IMAGE_BINARY_DIR}
  )

################################################
## Compiler and linker settings
################################################
include_directories(
  ${IFM3D_SIMPLE_IMAGE_BINARY_DIR}/include
  ${IFM3D_FG_BINARY_DIR}/include
  ${IFM3D_CAMERA_BINARY_DIR}/include
  )

################################################
## Process child CMakeLists.txt files
################################################
add_subdirectory(src)

################################################
## Manage installation process
################################################
if(BUILD_SDK_PKG)
  install(DIRECTORY ${IFM3D_SIMPLE_IMAGE_BINARY_DIR}/include
    DESTINATION ${CPACK_INSTALL_PREFIX}
    COMPONENT simple_image
    PATTERN "*.in" EXCLUDE
    PATTERN ".git" EXCLUDE
    PATTERN "*~" EXCLUDE
    )
endif()
