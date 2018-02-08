project(IFM3D_EXAMPLES)
cmake_minimum_required(VERSION 2.8.12)

set(CMAKE_BUILD_TYPE Release) # Release or Debug

set(CMAKE_MODULE_PATH
    ${ifm3d_EXAMPLES_SOURCE_DIR}/../../cmake/modules
    ${CMAKE_MODULE_PATH}
    )

# force an out-of-source build of the code
include(MacroOutOfSourceBuild)
macro_ensure_out_of_source_build(
  "Please build ${PROJECT_NAME} out-of-source")

################################################
## Bring in dependent projects
################################################
include(ifm3d_version)
find_package(PCL 1.7.1 REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_library(LIB_boost_system NAMES boost_system)


# Fix this: https://bugs.launchpad.net/ubuntu/+source/vtk6/+bug/1573234
list(FIND PCL_IO_LIBRARIES "vtkproj4" VTKPROJ4_REQUIRED)
if(NOT VTKPROJ4_REQUIRED EQUAL -1)
    find_library(LIBVTKPROJ4_LIBRARY NAMES vtkproj4)
    if(NOT ${LIBVTKPROJ4_LIBRARY})
        list(REMOVE_ITEM PCL_IO_LIBRARIES "vtkproj4")
    endif()
endif()

################################################
## Manage our compiler and linker flags
################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
  ${IFM3D_IMAGE_BINARY_DIR}/include
  ${IFM3D_FG_BINARY_DIR}/include
  ${IFM3D_CAMERA_BINARY_DIR}/include
  ${IFM3D_PCICCLIENT_BINARY_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  )
add_definitions(
  ${PCL_DEFINITIONS}
  ${OpenCV_DEFINITIONS}
  )

################################################
## Build the examples
################################################

add_executable(ex-file_io ex-file_io.cpp)
target_link_libraries(ex-file_io
                      ifm3d_image_shared
                      ifm3d_framegrabber_shared
                      ifm3d_camera_shared
                      ${PCL_IO_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      )

add_executable(ex-getmac  ex-getmac.cpp)
target_link_libraries(ex-getmac
                      ifm3d_camera_shared
                      )

add_executable(ex-fast_app_switch ex-fast_app_switch.cpp)
target_link_libraries(ex-fast_app_switch
                      ifm3d_image_shared
                      ifm3d_framegrabber_shared
                      ifm3d_camera_shared
                      ifm3d_pcicclient_shared
                      ${OpenCV_LIBRARIES}
                      ${Boost_LIBRARIES}
                      )

add_executable(ex-pcicclient_set_io ex-pcicclient_set_io.cpp)
target_link_libraries(ex-pcicclient_set_io
                      ifm3d_camera_shared
                      ifm3d_pcicclient_shared
                      ${Boost_LIBRARIES}
                      )

add_executable(ex-pcicclient_async_messages ex-pcicclient_async_messages.cpp)
target_link_libraries(ex-pcicclient_async_messages
                      ifm3d_camera_shared
                      ifm3d_pcicclient_shared
                      ${Boost_LIBRARIES}
                      )

add_executable(ex-timestamp  ex-timestamp.cpp)
target_link_libraries(ex-timestamp
                      ifm3d_image_shared
                      ifm3d_framegrabber_shared
                      ifm3d_camera_shared
                      ${PCL_IO_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      )

add_executable(ex-exposure_times ex-exposure_times.cpp)
target_link_libraries(ex-exposure_times
                      ifm3d_camera_shared
                      ifm3d_framegrabber_shared
                      ifm3d_image_shared
                      ${PCL_IO_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      )
