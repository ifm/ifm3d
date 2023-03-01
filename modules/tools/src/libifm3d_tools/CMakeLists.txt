################################################
## Target: libifm3d_tools.so
################################################
file(GLOB IFM3D_TOOLS_SOURCES CONFIGURE_DEPENDS *.cpp)
if (BUILD_MODULE_FRAMEGRABBER)
  file(GLOB IFM3D_TOOLS_FG_SOURCES CONFIGURE_DEPENDS fg/*.cpp)
  set(IFM3D_TOOLS_SOURCES ${IFM3D_TOOLS_SOURCES} ${IFM3D_TOOLS_FG_SOURCES})
endif()
if (BUILD_MODULE_SWUPDATER)
  file(GLOB IFM3D_TOOLS_SWUPDATER_SOURCES CONFIGURE_DEPENDS swupdater/*.cpp)
  set(
    IFM3D_TOOLS_SOURCES
    ${IFM3D_TOOLS_SOURCES} ${IFM3D_TOOLS_SWUPDATER_SOURCES}
    )
endif()
add_library(ifm3d_tools_shared ${IFM3D_TOOLS_SOURCES})

#------------------
# Compiler settings
#------------------
include_directories(${CURL_INCLUDE_DIRS})
target_include_directories(ifm3d_tools_shared
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_TOOLS_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_BINARY_DIR}/include>
    ${CURL_INCLUDE_DIRS}
    )

if (BUILD_MODULE_FRAMEGRABBER)
  target_include_directories(ifm3d_tools_shared
    PUBLIC
      $<BUILD_INTERFACE:${IFM3D_FG_SOURCE_DIR}/include>
    )
endif()

if (BUILD_MODULE_SWUPDATER)
  target_include_directories(ifm3d_tools_shared
    PUBLIC
      $<BUILD_INTERFACE:${IFM3D_SWUPDATER_SOURCE_DIR}/include>
    )
endif()


if(WIN32)
   if(${BUILD_SHARED_LIBS})
    target_compile_definitions(ifm3d_tools_shared
      PRIVATE
        IFM3D_TOOLS_DLL_BUILD=1)
   else()
    target_compile_definitions(ifm3d_tools_shared
      PUBLIC
        IFM3D_TOOLS_STATIC_LIB=1)
   endif()
endif(WIN32)

#------------------
# Linker settings
#------------------
target_link_libraries(
  ifm3d_tools_shared
  ifm3d_common
  ifm3d_device
  ${CURL_LIBRARIES}
  cxxopts
  $<BUILD_INTERFACE:fmt::fmt-header-only>
  )

if (BUILD_MODULE_FRAMEGRABBER)
  target_link_libraries(
    ifm3d_tools_shared
    ifm3d_framegrabber
    )
endif()

if (BUILD_MODULE_SWUPDATER)
  target_link_libraries(
    ifm3d_tools_shared
    ifm3d_swupdater
    )
endif()

set_target_properties(ifm3d_tools_shared PROPERTIES
  VERSION ${ROOT_PROJECT_VERSION_SIMPLE}
  OUTPUT_NAME ifm3d_tools
  LINKER_LANGUAGE CXX
)

#------------------
# Installation
#------------------

#
# NOTE: right now we are not exporting anything from the 'tools'
# module. The tools library is for the exclusive use of our supported `ifm3d`
# command line tool. Maybe at some point we come up w/ a plugin architecture,
# but, today that is not the case.
#
install(TARGETS ifm3d_tools_shared
  RUNTIME DESTINATION ${_bin} COMPONENT tools
  LIBRARY DESTINATION ${_lib} COMPONENT tools
  ARCHIVE DESTINATION ${_lib} COMPONENT tools
  PUBLIC_HEADER DESTINATION ${_include} COMPONENT tools
  )
