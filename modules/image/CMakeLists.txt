project(IFM3D_IMAGE CXX)
set(IFM3D_IMAGE_MODULE_NAME "libifm3d-image")
option(FORCE_OPENCV3 "Force the build to require OpenCV 3" OFF)
option(FORCE_OPENCV2 "Force the build to require OpenCV 2.4" OFF)

file(COPY ${IFM3D_IMAGE_SOURCE_DIR}/include
  DESTINATION ${IFM3D_IMAGE_BINARY_DIR}
  )

################################################
## Bring in dependent projects
################################################
find_package(PCL 1.7 REQUIRED COMPONENTS common)
if(FORCE_OPENCV3)
  find_package(OpenCV 3 REQUIRED)
elseif(FORCE_OPENCV2)
  find_package(OpenCV 2.4 REQUIRED)
else()
  find_package(OpenCV REQUIRED)
endif()

################################################
## Compiler and linker settings
################################################
include_directories(
  ${IFM3D_IMAGE_BINARY_DIR}/include
  ${IFM3D_FG_BINARY_DIR}/include
  ${IFM3D_CAMERA_BINARY_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  )

add_definitions(
  ${PCL_DEFINITIONS}
  ${OpenCV_DEFINITIONS}
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
