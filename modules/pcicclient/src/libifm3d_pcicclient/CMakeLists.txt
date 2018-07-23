################################################
## Dependent projects
################################################
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(glog QUIET CONFIG NAMES google-glog glog)
if(NOT TARGET glog::glog)
  find_library(LIB_glog NAMES glog)
else()
  set(LIB_glog glog::glog)
endif(NOT TARGET glog::glog)


################################################
## Target: libifm3d_pcicclient.so
################################################
file(GLOB IFM3D_PCICCLIENT_SOURCES *.cpp)
add_library(ifm3d_pcicclient_shared SHARED ${IFM3D_PCICCLIENT_SOURCES})

#------------------
# Compiler settings
#------------------


target_include_directories(ifm3d_pcicclient_shared
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_PCICCLIENT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_BINARY_DIR}/include>
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${Boost_INCLUDE_DIRS}
  )

target_compile_options(ifm3d_framegrabber_shared
  PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-std=c++${CMAKE_CXX_STANDARD}>
  )

if(WIN32)
    target_compile_definitions(ifm3d_pcicclient_shared PRIVATE
      IFM3D_PCICCLIENT_DLL_BUILD=1)
endif(WIN32)

#------------------
# Linker settings
#------------------

target_link_libraries(
  ifm3d_pcicclient_shared
  PRIVATE
    ifm3d_camera_shared
    ${LIB_glog}
    Boost::system
    Threads::Threads
  )

set_target_properties(ifm3d_pcicclient_shared PROPERTIES
  VERSION ${IFM3D_VERSION}
  EXPORT_NAME pcicclient
  OUTPUT_NAME ifm3d_pcicclient
  LINKER_LANGUAGE CXX
  )

#------------------
# Installation
#------------------
install(TARGETS ifm3d_pcicclient_shared
  EXPORT ifm3d-pcicclient-targets
  RUNTIME DESTINATION ${_bin} COMPONENT pcicclient
  LIBRARY DESTINATION ${_lib} COMPONENT pcicclient
  ARCHIVE DESTINATION ${_lib} COMPONENT pcicclient
  PUBLIC_HEADER DESTINATION ${_include} COMPONENT pcicclient
  )

if(BUILD_SDK_PKG)
  install(EXPORT ifm3d-pcicclient-targets
    NAMESPACE ifm3d::
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
    COMPONENT pcicclient
    )
endif()
