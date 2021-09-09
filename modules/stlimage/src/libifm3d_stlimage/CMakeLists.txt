################################################
## Bring in dependent projects
################################################
if(NOT BUILD_IN_DEPS)
find_package(glog QUIET CONFIG NAMES google-glog glog)
if(NOT TARGET glog::glog)
  find_library(LIB_glog NAMES glog)
else(NOT TARGET glog::glog)
  set(LIB_glog glog::glog)
endif (NOT TARGET glog::glog)
else(NOT TARGET glog::glog)
set( LIB_glog glog)
endif()

################################################
## Target: libifm3d_image.so
################################################
#file(GLOB IFM3D_STLIMAGE_SOURCES *.cpp)
add_library(ifm3d_stlimage
            ${CMAKE_CURRENT_SOURCE_DIR}/image.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/stl_image_buffer.cpp
)
set_property(TARGET ifm3d_stlimage PROPERTY POSITION_INDEPENDENT_CODE 1)

#------------------
# Compiler settings
#------------------
target_include_directories(ifm3d_stlimage
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_STLIMAGE_SOURCE_DIR}/include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

if(WIN32)
  target_compile_definitions(ifm3d_stlimage
    PRIVATE
      IFM3D_IMAGE_DLL_BUILD=1)
endif(WIN32)

#------------------
# Linker settings
#------------------
target_link_libraries(
  ifm3d_stlimage
  PUBLIC
    ifm3d_framegrabber
    ifm3d_camera
  PRIVATE
    ${LIB_glog}
  )

set_target_properties(ifm3d_stlimage PROPERTIES
  VERSION ${IFM3D_VERSION}
  EXPORT_NAME stlimage
  OUTPUT_NAME ifm3d_stlimage
  LINKER_LANGUAGE CXX
  )

#------------------
# Installation
#------------------
install(TARGETS ifm3d_stlimage
  EXPORT ifm3d-stlimage-targets
  RUNTIME DESTINATION ${_bin} COMPONENT stlimage
  LIBRARY DESTINATION ${_lib} COMPONENT stlimage
  ARCHIVE DESTINATION ${_lib} COMPONENT stlimage
  PUBLIC_HEADER DESTINATION ${_include} COMPONENT stlimage
  )

if(BUILD_SDK_PKG)
  install(EXPORT ifm3d-stlimage-targets
    NAMESPACE ifm3d::
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
    COMPONENT stlimage
    )
endif()
