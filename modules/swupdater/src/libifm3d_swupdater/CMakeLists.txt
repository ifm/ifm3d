################################################
## Bring in dependent projects
################################################
if(NOT BUILD_IN_DEPS)
  find_package(CURL REQUIRED)
else(NOT BUILD_IN_DEPS)
  set(CURL_LIBRARIES libcurl)
endif(NOT BUILD_IN_DEPS)

################################################
## Target: libifm3d_swupdater.so
################################################
file(GLOB IFM3D_SWUPDATER_SOURCES CONFIGURE_DEPENDS *.cpp)
add_library(ifm3d_swupdater ${IFM3D_SWUPDATER_SOURCES})
set_property(TARGET ifm3d_swupdater PROPERTY POSITION_INDEPENDENT_CODE 1)

#------------------
# Compiler settings
#------------------
target_include_directories(ifm3d_swupdater
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_SWUPDATER_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_SOURCE_DIR}/include>
    ${CURL_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

if(WIN32)
   if(${BUILD_SHARED_LIBS})
    target_compile_definitions(ifm3d_swupdater
      PRIVATE
        IFM3D_SWUPDATER_DLL_BUILD=1)
   else()
    target_compile_definitions(ifm3d_swupdater
      PUBLIC
        IFM3D_SWUPDATER_STATIC_LIB=1)
   endif()
endif(WIN32)

#------------------
# Linker settings
#------------------
target_link_libraries(
  ifm3d_swupdater
  PUBLIC 
    ifm3d_device
  PRIVATE
    ${CURL_LIBRARIES}
    $<BUILD_INTERFACE:asio>
    $<BUILD_INTERFACE:websocketpp_wrapper>
    $<BUILD_INTERFACE:fmt::fmt-header-only>
    )

set_target_properties(
  ifm3d_swupdater PROPERTIES
  VERSION ${ROOT_PROJECT_VERSION_SIMPLE}
  EXPORT_NAME swupdater
  OUTPUT_NAME ifm3d_swupdater
  LINKER_LANGUAGE CXX
  )

if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
  set_target_properties(ifm3d_swupdater PROPERTIES CXX_VISIBILITY_PRESET hidden)
endif()

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
