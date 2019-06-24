project(IFM3D_OPENCV CXX)
set(IFM3D_OPENCV_MODULE_NAME "libifm3d-opencv")

file(COPY ${IFM3D_OPENCV_SOURCE_DIR}/include
  DESTINATION ${IFM3D_OPENCV_BINARY_DIR}
  )

################################################
## "build" for header-only library
################################################

# deps
if(FORCE_OPENCV3)
  find_package(OpenCV 3 REQUIRED COMPONENTS core)
elseif(FORCE_OPENCV2)
  find_package(OpenCV 2.4 REQUIRED COMPONENTS core)
else()
  find_package(OpenCV REQUIRED COMPONENTS core)
endif()

# needs to be done before the target is declared
link_directories(${OpenCV_LIBRARY_DIRS})

add_library(ifm3d_opencv INTERFACE)
target_include_directories(ifm3d_opencv
  INTERFACE
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_OPENCV_BINARY_DIR}/include>
    ${OpenCV_INCLUDE_DIRS}
  )

target_compile_options(ifm3d_opencv
  INTERFACE $<$<COMPILE_LANGUAGE:CXX>:-std=c++${CMAKE_CXX_STANDARD}>
  )

target_link_libraries(ifm3d_opencv
  INTERFACE
    ifm3d_framegrabber
    ${OpenCV_LIBRARIES}
    )

set_target_properties(ifm3d_opencv PROPERTIES
  EXPORT_NAME opencv
  )

################################################
## Process child CMakeLists.txt files
################################################
if(BUILD_TESTS)
  add_subdirectory(test)
else()
  message(WARNING "Skipping ${IFM3D_OPENCV_MODULE_NAME} unit tests!")
endif()

################################################
## Manage installation process
################################################
if(BUILD_SDK_PKG)
  install(TARGETS ifm3d_opencv
    EXPORT ifm3d-opencv-targets
    )

  install(DIRECTORY ${IFM3D_OPENCV_BINARY_DIR}/include
    DESTINATION ${CPACK_INSTALL_PREFIX}
    COMPONENT opencv
    PATTERN "*.in" EXCLUDE
    PATTERN ".git" EXCLUDE
    PATTERN "*~" EXCLUDE
    )

  install(EXPORT ifm3d-opencv-targets
    NAMESPACE ifm3d::
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
    COMPONENT opencv
    )
endif()
