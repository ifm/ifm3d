################################################
## Bring in dependent projects
################################################
find_package(glog QUIET CONFIG NAMES google-glog glog)
if (NOT TARGET glog::glog)
  find_library(LIB_glog NAMES glog)
else()
  set(LIB_glog glog::glog)
endif (NOT TARGET glog::glog)
find_package(CURL REQUIRED)

################################################
## Target: libifm3d_swupdater.so
################################################
file(GLOB IFM3D_SWUPDATER_SOURCES *.cpp)
add_library(ifm3d_swupdater ${IFM3D_SWUPDATER_SOURCES})
set_property(TARGET ifm3d_swupdater PROPERTY POSITION_INDEPENDENT_CODE 1)

#------------------
# Compiler settings
#------------------
target_include_directories(ifm3d_swupdater
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_SWUPDATER_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_BINARY_DIR}/include>
    ${CURL_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

target_compile_options(ifm3d_swupdater
  PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-std=c++${CMAKE_CXX_STANDARD}>
  )

if(WIN32)
    target_compile_definitions(ifm3d_swupdater PRIVATE
        IFM3D_SWUPDATER_DLL_BUILD=1)
endif(WIN32)

#------------------
# Linker settings
#------------------
target_link_libraries(
  ifm3d_swupdater
  PUBLIC
    ${LIB_glog}
    ${CURL_LIBRARIES}
  PRIVATE
    ifm3d_camera
  )

set_target_properties(
  ifm3d_swupdater PROPERTIES
  VERSION ${IFM3D_VERSION}
  EXPORT_NAME swupdater
  OUTPUT_NAME ifm3d_swupdater
  LINKER_LANGUAGE CXX
  )

#------------------
# Installation
#------------------
install(TARGETS ifm3d_swupdater
  EXPORT ifm3d-swupdater-targets
  RUNTIME DESTINATION ${_bin} COMPONENT swupdater
  LIBRARY DESTINATION ${_lib} COMPONENT swupdater
  ARCHIVE DESTINATION ${_lib} COMPONENT swupdater
  PUBLIC_HEADER DESTINATION ${_include} COMPONENT swupdater
  )

if(BUILD_SDK_PKG)
  install(EXPORT ifm3d-swupdater-targets
    NAMESPACE ifm3d::
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
    COMPONENT swupdater
    )
endif()
