project(IFM3D_IMAGE CXX)
set(IFM3D_IMAGE_MODULE_NAME "libifm3d-image")

if(FORCE_OPENCV3)
  find_package(OpenCV 3 REQUIRED COMPONENTS core)
elseif(FORCE_OPENCV2)
  find_package(OpenCV 2.4 REQUIRED COMPONENTS core)
else()
  find_package(OpenCV REQUIRED COMPONENTS core)
endif()

file(COPY ${IFM3D_IMAGE_SOURCE_DIR}/include
  DESTINATION ${IFM3D_IMAGE_BINARY_DIR}
  )

################################################
## Process child CMakeLists.txt files
################################################
add_subdirectory(src)
if(BUILD_TESTS)
  add_subdirectory(test)
else()
  message(WARNING "Skipping ${IFM3D_IMAGE_MODULE_NAME} unit tests!")
endif()

################################################
## Manage installation process
################################################
if(BUILD_SDK_PKG)
  install(DIRECTORY ${IFM3D_IMAGE_BINARY_DIR}/include
    DESTINATION ${CPACK_INSTALL_PREFIX}
    COMPONENT image
    PATTERN "*.in" EXCLUDE
    PATTERN ".git" EXCLUDE
    PATTERN "*~" EXCLUDE
    )
endif()
