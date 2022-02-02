################################################
## Bring in dependent projects
################################################
find_package(PCL 1.7 REQUIRED COMPONENTS common)
add_library(pcl_headers INTERFACE IMPORTED)
set_target_properties(pcl_headers PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}")

# opencv deps
if(FORCE_OPENCV3)
  find_package(OpenCV 3 REQUIRED COMPONENTS core)
elseif(FORCE_OPENCV2)
  find_package(OpenCV 2.4 REQUIRED COMPONENTS core)
else()
  find_package(OpenCV REQUIRED COMPONENTS core)
endif()

find_package(glog QUIET CONFIG NAMES google-glog glog)
if (NOT TARGET glog::glog)
  find_library(LIB_glog NAMES glog)
else()
  set(LIB_glog glog::glog)
endif (NOT TARGET glog::glog)

################################################
## Target: libifm3d_image.so
################################################
file(GLOB IFM3D_IMAGE_SOURCES *.cpp)
add_library(ifm3d_image ${IFM3D_IMAGE_SOURCES})
set_property(TARGET ifm3d_image PROPERTY POSITION_INDEPENDENT_CODE 1)

#------------------
# Compiler settings
#------------------
target_include_directories(ifm3d_image
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_IMAGE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_FG_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_BINARY_DIR}/include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

if(WIN32)
  target_compile_definitions(ifm3d_image
    PUBLIC
      ${PCL_DEFINITIONS}
      ${OpenCV_DEFINITIONS}
    PRIVATE
      IFM3D_IMAGE_DLL_BUILD=1)
endif(WIN32)

#------------------
# Linker settings
#------------------
target_link_libraries(
  ifm3d_image
  PUBLIC
    ifm3d_framegrabber
    ifm3d_camera
    ${OpenCV_LIBS}
    pcl_headers
  PRIVATE
    ${LIB_glog}
  )

set_target_properties(ifm3d_image PROPERTIES
  VERSION ${IFM3D_VERSION}
  EXPORT_NAME image
  OUTPUT_NAME ifm3d_image
  LINKER_LANGUAGE CXX
  )

#------------------
# Installation
#------------------
install(TARGETS ifm3d_image
  EXPORT ifm3d-image-targets
  RUNTIME DESTINATION ${_bin} COMPONENT image
  LIBRARY DESTINATION ${_lib} COMPONENT image
  ARCHIVE DESTINATION ${_lib} COMPONENT image
  PUBLIC_HEADER DESTINATION ${_include} COMPONENT image
  )

if(BUILD_SDK_PKG)
  install(EXPORT ifm3d-image-targets
    NAMESPACE ifm3d::
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
    COMPONENT image
    )
endif()
