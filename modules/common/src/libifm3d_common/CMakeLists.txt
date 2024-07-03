################################################
## Target: libifm3d_common.so
################################################
file(GLOB_RECURSE IFM3D_COMMON_SOURCES CONFIGURE_DEPENDS *.cpp  ../../include/ifm3d/common)
add_library(ifm3d_common INTERFACE)
target_sources(ifm3d_common INTERFACE ${IFM3D_COMMON_SOURCES})

#------------------
# Compiler settings
#------------------
target_include_directories(ifm3d_common
  INTERFACE
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_COMMON_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_COMMON_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  )
    
target_compile_definitions(ifm3d_common
INTERFACE
  IFM3D_COMMON_LOGGING_STRIP_PREFIX="${ROOT_PROJECT_PATH}"
)
  
target_link_libraries(
  ifm3d_common
  INTERFACE
    $<BUILD_INTERFACE:fmt::fmt-header-only>
  )