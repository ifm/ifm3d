################################################
## Bring in dependent projects
################################################
find_package(glog QUIET CONFIG NAMES google-glog glog)
if (NOT TARGET glog::glog)
  find_library(LIB_glog NAMES glog)
else()
  set(LIB_glog glog::glog)
endif (NOT TARGET glog::glog)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(CURL REQUIRED)

################################################
## Target: libifm3d_tools.so
################################################
file(GLOB IFM3D_TOOLS_SOURCES *.cpp)
if (BUILD_MODULE_FRAMEGRABBER)
  file(GLOB IFM3D_TOOLS_FG_SOURCES fg/*.cpp)
  set(IFM3D_TOOLS_SOURCES ${IFM3D_TOOLS_SOURCES} ${IFM3D_TOOLS_FG_SOURCES})
endif()
if (BUILD_MODULE_SWUPDATER)
  file(GLOB IFM3D_TOOLS_SWUPDATER_SOURCES swupdater/*.cpp)
  set(
    IFM3D_TOOLS_SOURCES
    ${IFM3D_TOOLS_SOURCES} ${IFM3D_TOOLS_SWUPDATER_SOURCES}
    )
endif()
add_library(ifm3d_tools_shared SHARED ${IFM3D_TOOLS_SOURCES})

#------------------
# Compiler settings
#------------------
include_directories(${CURL_INCLUDE_DIRS})
target_include_directories(ifm3d_tools_shared
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_TOOLS_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_BINARY_DIR}/include>
    ${Boost_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIRS}
    )

if (BUILD_MODULE_FRAMEGRABBER)
  target_include_directories(ifm3d_tools_shared
    PUBLIC
      $<BUILD_INTERFACE:${IFM3D_FG_BINARY_DIR}/include>
    )
endif()

if (BUILD_MODULE_SWUPDATER)
  target_include_directories(ifm3d_tools_shared
    PUBLIC
      $<BUILD_INTERFACE:${IFM3D_SWUPDATER_BINARY_DIR}/include>
    )
endif()

if (BUILD_MODULE_IMAGE)
  target_include_directories(ifm3d_tools_shared
    PUBLIC
      $<BUILD_INTERFACE:${IFM3D_IMAGE_BINARY_DIR}/include>
    )
endif()

if (BUILD_MODULE_OPENCV)
  target_include_directories(ifm3d_tools_shared
    PUBLIC
      $<BUILD_INTERFACE:${IFM3D_OPENCV_BINARY_DIR}/include>
    )
endif()


target_compile_options(ifm3d_tools_shared
  PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-std=c++${CMAKE_CXX_STANDARD}>
  )

if(WIN32)
    target_compile_definitions(ifm3d_tools_shared PRIVATE
        IFM3D_TOOLS_DLL_BUILD=1)
endif(WIN32)

#------------------
# Linker settings
#------------------
target_link_libraries(
  ifm3d_tools_shared
  ifm3d_camera
  ${LIB_glog}
  Boost::program_options
  ${CURL_LIBRARIES}
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

if (BUILD_MODULE_IMAGE)
  target_link_libraries(
    ifm3d_tools_shared
    ifm3d_image
    )
endif()

if (BUILD_MODULE_OPENCV)
  target_link_libraries(
    ifm3d_tools_shared
    ifm3d_opencv
    )
endif()

set_target_properties(ifm3d_tools_shared PROPERTIES
  VERSION ${IFM3D_VERSION}
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
