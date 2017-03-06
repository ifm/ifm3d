find_library(LIB_glog NAMES glog)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

file(GLOB IFM3D_IMAGE_SOURCES *.cpp)
add_library(ifm3d_image OBJECT ${IFM3D_IMAGE_SOURCES})
set_property(TARGET ifm3d_image PROPERTY POSITION_INDEPENDENT_CODE 1)

add_library(
  ifm3d_image_shared SHARED $<TARGET_OBJECTS:ifm3d_image>)

target_link_libraries(
  ifm3d_image_shared
  ifm3d_framegrabber_shared
  ifm3d_camera_shared
  ${OpenCV_LIBRARIES}
  ${LIB_glog}
  )

set_target_properties(ifm3d_image_shared PROPERTIES
  VERSION ${IFM3D_VERSION_STRING}
  OUTPUT_NAME ifm3d_image
  LINKER_LANGUAGE CXX
  )

install(TARGETS ifm3d_image_shared DESTINATION lib)
