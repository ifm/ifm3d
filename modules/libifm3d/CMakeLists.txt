################################################
## Target: libifm3d.so
################################################
add_library(libifm3d)

#------------------
# Compiler settings
#------------------
if(WIN32)
    target_compile_definitions(libifm3d PRIVATE IFM3D_DLL_BUILD=1)
endif(WIN32)

#------------------
# Linker settings
#------------------
target_link_libraries(
  libifm3d
  PRIVATE
    ifm3d_common
    ifm3d_device
    $<BUILD_INTERFACE:fmt::fmt-header-only>
    $<BUILD_INTERFACE:asio>
)

target_include_directories(libifm3d
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_COMMON_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_COMMON_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_DEVICE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_DEVICE_BINARY_DIR}/include>
)

if(BUILD_MODULE_FRAMEGRABBER)
    target_link_libraries(libifm3d PRIVATE ifm3d_framegrabber)
    target_include_directories(libifm3d
      INTERFACE
        $<BUILD_INTERFACE:${IFM3D_FG_SOURCE_DIR}/include>
    )
endif()

if(BUILD_MODULE_SWUPDATER)
    target_link_libraries(libifm3d PRIVATE ifm3d_swupdater)
    target_include_directories(libifm3d
      INTERFACE
        $<BUILD_INTERFACE:${IFM3D_SWUPDATER_SOURCE_DIR}/include>
    )
endif()

if(BUILD_MODULE_PCICCLIENT)
    target_link_libraries(libifm3d PRIVATE ifm3d_pcicclient)
    target_include_directories(libifm3d
      INTERFACE
        $<BUILD_INTERFACE:${IFM3D_PCICCLIENT_SOURCE_DIR}/include>
    )
endif()

if(BUILD_MODULE_DESERIALIZE)
    target_link_libraries(libifm3d PRIVATE ifm3d_deserialize)
    target_include_directories(libifm3d
      INTERFACE
        $<BUILD_INTERFACE:${IFM3D_DESERIALIZE_SOURCE_DIR}/include>
    )
endif()

set_target_properties(libifm3d PROPERTIES
  VERSION ${ROOT_PROJECT_VERSION_SIMPLE}
  EXPORT_NAME ifm3d
  OUTPUT_NAME ifm3d
  LINKER_LANGUAGE CXX
  POSITION_INDEPENDENT_CODE 1
)

if(WIN32)
   if(${BUILD_SHARED_LIBS})
    target_compile_definitions(
      libifm3d PRIVATE
      IFM3D_DLL_BUILD=1
    )
   else()
    target_compile_definitions(
      libifm3d PUBLIC
      IFM3D_STATIC_LIB=1
    )
   endif()
endif(WIN32)

if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
  set_target_properties(libifm3d PROPERTIES CXX_VISIBILITY_PRESET hidden)
endif()

#------------------
# Installation
#------------------
install(TARGETS libifm3d
  EXPORT ifm3d
  COMPONENT ifm3d
  RUNTIME DESTINATION ${_bin}
  LIBRARY DESTINATION ${_lib}
  ARCHIVE DESTINATION ${_lib}
  PUBLIC_HEADER DESTINATION ${_include}
)

# create the cmake-package files
if(BUILD_SDK_PKG)
  install(EXPORT ifm3d
    FILE ifm3d-targets.cmake
    NAMESPACE ifm3d::
    DESTINATION ${_lib}/cmake/ifm3d-${IFM3D_VERSION}
    COMPONENT ifm3d
  )

  include(CMakePackageConfigHelpers)
  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/ifm3d-config-version.cmake
    VERSION ${IFM3D_VERSION}
    COMPATIBILITY SameMinorVersion
  )

  configure_package_config_file(
    ${ROOT_PROJECT_DIR}/cmake/modules/ifm3d-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ifm3d-config.cmake
    INSTALL_DESTINATION ${_lib}/cmake/ifm3d-${IFM3D_VERSION}
  )

  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/ifm3d-config-version.cmake
          ${CMAKE_CURRENT_BINARY_DIR}/ifm3d-config.cmake
    DESTINATION ${_lib}/cmake/ifm3d-${IFM3D_VERSION}
    COMPONENT ifm3d
  )
endif()