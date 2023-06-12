################################################
## Target: libifm3d_pcicclient.so
################################################
file(GLOB IFM3D_PCICCLIENT_SOURCES CONFIGURE_DEPENDS *.cpp)
add_library(ifm3d_pcicclient ${IFM3D_PCICCLIENT_SOURCES})
set_property(TARGET ifm3d_pcicclient PROPERTY POSITION_INDEPENDENT_CODE 1)

#------------------
# Compiler settings
#------------------


target_include_directories(ifm3d_pcicclient
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_PCICCLIENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_SOURCE_DIR}/include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

if(WIN32)
    target_compile_definitions(ifm3d_pcicclient PRIVATE
      IFM3D_PCICCLIENT_DLL_BUILD=1)
endif(WIN32)

#------------------
# Linker settings
#------------------

target_link_libraries(
  ifm3d_pcicclient
  PUBLIC
    ifm3d_device
  PRIVATE
    $<BUILD_INTERFACE:fmt::fmt-header-only>
    $<BUILD_INTERFACE:asio>
  )

set_target_properties(ifm3d_pcicclient PROPERTIES
  VERSION ${ROOT_PROJECT_VERSION_SIMPLE}
  EXPORT_NAME pcicclient
  OUTPUT_NAME ifm3d_pcicclient
  LINKER_LANGUAGE CXX
  )

if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
  set_target_properties(ifm3d_pcicclient PROPERTIES CXX_VISIBILITY_PRESET hidden)
endif()

#------------------
# Installation
#------------------
install(TARGETS ifm3d_pcicclient
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
