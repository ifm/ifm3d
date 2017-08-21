find_library(LIB_glog NAMES glog)
find_package(Boost REQUIRED COMPONENTS program_options system)

file(GLOB IFM3D_TOOLS_SOURCES *.cpp)
if (BUILD_MODULE_FRAMEGRABBER)
  file(GLOB IFM3D_TOOLS_FG_SOURCES fg/*.cpp)
  set(IFM3D_TOOLS_SOURCES ${IFM3D_TOOLS_SOURCES} ${IFM3D_TOOLS_FG_SOURCES})
endif()
if (BUILD_MODULE_IMAGE)
  file(GLOB IFM3D_TOOLS_IMAGE_SOURCES image/*.cpp)
  set(IFM3D_TOOLS_SOURCES ${IFM3D_TOOLS_SOURCES} ${IFM3D_TOOLS_IMAGE_SOURCES})
endif()

add_library(ifm3d_tools OBJECT ${IFM3D_TOOLS_SOURCES})
set_property(TARGET ifm3d_tools PROPERTY POSITION_INDEPENDENT_CODE 1)

add_library(ifm3d_tools_shared SHARED $<TARGET_OBJECTS:ifm3d_tools>)
if(BUILD_MODULE_IMAGE)
    target_link_libraries(
      ifm3d_tools_shared
      ifm3d_camera_shared
      ifm3d_framegrabber_shared
      ifm3d_image_shared
      ${PCL_VISUALIZATION_LIBRARIES}
      ${LIB_glog}
      ${Boost_LIBRARIES}
    )
elseif(BUILD_MODULE_FRAMEGRABBER)
  target_link_libraries(
    ifm3d_tools_shared
    ifm3d_camera_shared
    ifm3d_framegrabber_shared
    ${LIB_glog}
    ${Boost_LIBRARIES}
    )
else()
  target_link_libraries(
    ifm3d_tools_shared
    ifm3d_camera_shared
    ${LIB_glog}
    ${Boost_LIBRARIES}
    )
endif()

if (BUILD_MODULE_IMAGE)
  set_target_properties(ifm3d_tools_shared PROPERTIES
    VERSION ${IFM3D_VERSION_STRING}
    OUTPUT_NAME ifm3d_tools
    LINKER_LANGUAGE CXX
    )
  set_target_properties(ifm3d_tools PROPERTIES
    AUTOMOC TRUE
    COMPILE_FLAGS "-Wno-deprecated -Wno-deprecated-declarations"
    )
else()
  set_target_properties(ifm3d_tools_shared PROPERTIES
    VERSION ${IFM3D_VERSION_STRING}
    OUTPUT_NAME ifm3d_tools
    LINKER_LANGUAGE CXX
  )
endif()

install(TARGETS ifm3d_tools_shared
  DESTINATION lib
  COMPONENT tools
  )
