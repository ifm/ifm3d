################################################
## Setup
################################################

# Bring in pybind11, including cmake macros and variables
find_package(pybind11 REQUIRED)

# Query the selected interpreter for the installation path, or use the override
if(DEFINED PYTHON_PACKAGES_PATH)
  set(_python_install_path ${PYTHON_PACKAGES_PATH})
else()
  execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_lib; print(get_python_lib(prefix=''))"
    OUTPUT_VARIABLE _site_packages
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )

  set(_python_install_path "${CMAKE_INSTALL_PREFIX}/${_site_packages}")
endif()

# Set the component name based on selected python version. This is primarily
# to honor debian python packge naming convetions.
if(PYTHON_VERSION_MAJOR EQUAL 2)
  set(_component_name "python")
elseif(PYTHON_VERSION_MAJOR EQUAL 3)
  set(_component_name "python3")
else()
  message(
    FATAL_ERROR "Unexpected PYTHON_VERSION_MAJOR: ${PYTHON_VERSION_MAJOR}"
    )
endif()

################################################
## Target: ifm3dpy.so
## (Note: Name of the target maps directly to
## the name of the resulting python module, so
## we break from the naming convention found in
## other ifm3d components here.
################################################
add_library(ifm3dpy MODULE main.cpp)

#------------------
# Compiler settings
#------------------
target_include_directories(ifm3dpy
  PRIVATE
    $<BUILD_INTERFACE:${IFM3D_FG_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CAMERA_BINARY_DIR}/include>
    ${IFM3D_SOURCE_DIR}/modules/opencv/include
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

#------------------
# Linker Settings
#------------------
target_link_libraries(
  ifm3dpy
  PRIVATE
    ifm3d_framegrabber
    ifm3d_camera
    ${OpenCV_LIBRARIES}
    pybind11::module
  )

#------------------
# Properties
#------------------
set_target_properties(
  ifm3dpy PROPERTIES
  EXPORT_NAME ${_component_name}
  PREFIX "${PYTHON_MODULE_PREFIX}"
  SUFFIX "${PYTHON_MODULE_EXTENSION}"
  )

#------------------
# Installation
#------------------
install(TARGETS ifm3dpy
  RUNTIME DESTINATION ${_python_install_path} COMPONENT ${_component_name}
  LIBRARY DESTINATION ${_python_install_path} COMPONENT ${_component_name}
  ARCHIVE DESTINATION ${_python_install_path} COMPONENT ${_component_name}
  PUBLIC_HEADER DESTINATION ${_include} COMPONENT ${_component_name}
  )
