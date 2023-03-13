################################################
## Bring in dependent projects
################################################
find_package(Threads REQUIRED)
if(NOT BUILD_IN_DEPS)
  find_package(glog QUIET CONFIG NAMES google-glog glog)
  if(NOT TARGET glog::glog)
    find_library(LIB_glog NAMES glog)
  else(NOT TARGET glog::glog)
    set(LIB_glog glog::glog)
  endif (NOT TARGET glog::glog)
else(NOT BUILD_IN_DEPS)
  set(LIB_glog glog)
endif()
################################################
## Target: libifm3d_framegrabber.so
################################################
file(GLOB IFM3D_FG_SOURCES CONFIGURE_DEPENDS *.cpp *.hpp)
add_library(ifm3d_framegrabber 
            ${IFM3D_FG_SOURCES}
            ${CMAKE_CURRENT_SOURCE_DIR}/algo/o3r_uncompress_di.cpp
            )
set_property(TARGET ifm3d_framegrabber PROPERTY POSITION_INDEPENDENT_CODE 1)

#------------------
# Compiler settings
#------------------
target_include_directories(ifm3d_framegrabber
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_FG_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_SOURCE_DIR}/include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/algo
  )

if(WIN32)
   if(${BUILD_SHARED_LIBS})
    target_compile_definitions(ifm3d_framegrabber
      PRIVATE
        IFM3D_FRAME_GRABBER_DLL_BUILD=1)
   else()
    target_compile_definitions(ifm3d_framegrabber
      PUBLIC
        IFM3D_FRAME_GRABBER_STATIC_LIB=1)
   endif()
endif(WIN32) 

#------------------
# Linker settings
#------------------
# Note : $<BUILD_INTERFACE:asio> needed to avoid install export error
# during static linkage of framgrabber for python binding as asio
# is header only target.
target_link_libraries(
  ifm3d_framegrabber
  PUBLIC
    ${LIB_glog}
  PRIVATE
    ifm3d_device
    Threads::Threads
    $<BUILD_INTERFACE:asio>
    $<BUILD_INTERFACE:fmt::fmt-header-only>
  )

set_target_properties(
  ifm3d_framegrabber PROPERTIES
  VERSION ${ROOT_PROJECT_VERSION_SIMPLE}
  EXPORT_NAME framegrabber
  OUTPUT_NAME ifm3d_framegrabber
  LINKER_LANGUAGE CXX
  )


if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang" )
  set_target_properties(ifm3d_framegrabber PROPERTIES CXX_VISIBILITY_PRESET hidden)
endif()
#------------------
# Installation
#------------------
install(TARGETS ifm3d_framegrabber
  EXPORT ifm3d-framegrabber-targets
  RUNTIME DESTINATION ${_bin} COMPONENT framegrabber
  LIBRARY DESTINATION ${_lib} COMPONENT framegrabber
  ARCHIVE DESTINATION ${_lib} COMPONENT framegrabber
  PUBLIC_HEADER DESTINATION ${_include} COMPONENT framegrabber
  )

if(BUILD_SDK_PKG)
  install(EXPORT ifm3d-framegrabber-targets
    NAMESPACE ifm3d::
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
    COMPONENT framegrabber
    )
endif()
