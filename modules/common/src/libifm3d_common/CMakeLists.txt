################################################
## Target: libifm3d_common.so
################################################
file(GLOB_RECURSE IFM3D_COMMON_SOURCES CONFIGURE_DEPENDS *.cpp  ../../include/ifm3d/common)
add_library(ifm3d_common ${IFM3D_COMMON_SOURCES})
set_property(TARGET ifm3d_common PROPERTY POSITION_INDEPENDENT_CODE 1)

#------------------
# Compiler settings
#------------------
target_include_directories(ifm3d_common
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_COMMON_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_COMMON_BINARY_DIR}/include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    )
    
target_compile_definitions(ifm3d_common
INTERFACE
  IFM3D_COMMON_LOGGING_STRIP_PREFIX="${ROOT_PROJECT_PATH}"
)

if(WIN32)
   if(${BUILD_SHARED_LIBS})
    target_compile_definitions(
      ifm3d_common PRIVATE
      IFM3D_COMMON_DLL_BUILD=1
      )
   else()
    target_compile_definitions(
      ifm3d_common PUBLIC
      IFM3D_COMMON_STATIC_LIB=1
      )
   endif()
endif(WIN32)
  
#------------------
# Linker settings
#------------------
target_link_libraries(
  ifm3d_common
  PRIVATE
    $<BUILD_INTERFACE:fmt::fmt-header-only>
  )

set_target_properties(
  ifm3d_common PROPERTIES
  VERSION ${IFM3D_VERSION}
  EXPORT_NAME common
  OUTPUT_NAME ifm3d_common
  LINKER_LANGUAGE CXX
  )
  
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang" )
  set_target_properties(ifm3d_common PROPERTIES CXX_VISIBILITY_PRESET hidden)
endif()


#------------------
# Installation
#------------------
install(TARGETS ifm3d_common
  EXPORT ifm3d-common-targets
  RUNTIME DESTINATION ${_bin} COMPONENT common
  LIBRARY DESTINATION ${_lib} COMPONENT common
  ARCHIVE DESTINATION ${_lib} COMPONENT common
  PUBLIC_HEADER DESTINATION ${_include} COMPONENT common
)

if(BUILD_SDK_PKG)
  install(EXPORT ifm3d-common-targets
    NAMESPACE ifm3d::
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
    COMPONENT common
  )
endif()
