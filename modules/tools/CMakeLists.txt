project(IFM3D_TOOLS CXX)
set(IFM3D_TOOLS_MODULE_NAME "libifm3d-tools")

file(COPY ${IFM3D_TOOLS_SOURCE_DIR}/include
  DESTINATION ${IFM3D_TOOLS_BINARY_DIR}
  )
if (NOT BUILD_MODULE_FRAMEGRABBER)
  file(REMOVE_RECURSE ${IFM3D_TOOLS_BINARY_DIR}/include/ifm3d/tools/fg)
endif()
if (NOT BUILD_MODULE_IMAGE)
  file(REMOVE_RECURSE ${IFM3D_TOOLS_BINARY_DIR}/include/ifm3d/tools/image)
endif()

################################################
## Bring in dependent projects
################################################
find_package(Boost REQUIRED COMPONENTS program_options)

################################################
## Compiler and linker settings
################################################
include_directories(
  ${IFM3D_TOOLS_BINARY_DIR}/include
  ${IFM3D_CAMERA_BINARY_DIR}/include
  ${Boost_INCLUDE_DIRS}
  )
if (BUILD_MODULE_FRAMEGRABBER)
  include_directories(${IFM3D_FG_BINARY_DIR}/include)
endif()
if (BUILD_MODULE_IMAGE)
  find_package(PCL 1.7 REQUIRED COMPONENTS visualization)
  # Fix this: https://bugs.launchpad.net/ubuntu/+source/vtk6/+bug/1573234
  list(FIND PCL_VISUALIZATION_LIBRARIES "vtkproj4" VTKPROJ4_REQUIRED)
  if(NOT VTKPROJ4_REQUIRED EQUAL -1)
    find_library(LIBVTKPROJ4_LIBRARY NAMES vtkproj4)
    if(NOT ${LIBVTKPROJ4_LIBRARY})
      list(REMOVE_ITEM PCL_VISUALIZATION_LIBRARIES "vtkproj4")
    endif()
  endif()

  if(FORCE_OPENCV3)
    find_package(OpenCV 3 REQUIRED)
  elseif(FORCE_OPENCV2)
    find_package(OpenCV 2.4 REQUIRED)
  else()
    find_package(OpenCV REQUIRED)
  endif()

  include_directories(
    ${IFM3D_IMAGE_BINARY_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )
endif()

################################################
## Process child CMakeLists.txt files
################################################
add_subdirectory(src)
add_subdirectory(contrib)

################################################
## Manage installation process
################################################
if(BUILD_SDK_PKG)
  install(DIRECTORY ${IFM3D_TOOLS_BINARY_DIR}/include
    DESTINATION ${CPACK_INSTALL_PREFIX}
    COMPONENT tools
    PATTERN "*.in" EXCLUDE
    PATTERN ".git" EXCLUDE
    PATTERN "*~" EXCLUDE
    )
endif()
