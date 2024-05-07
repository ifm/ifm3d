################################################
## Setup
################################################
if(NOT BUILD_IN_DEPS)
  # Bring in pybind11, including cmake macros and variables
  find_package(pybind11 REQUIRED)
endif()

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
    $<BUILD_INTERFACE:${IFM3D_DEVICE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_DEVICE_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_FG_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_TOOLS_SOURCE_DIR}/include>
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

target_compile_options(ifm3dpy PRIVATE
  $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-Wno-deprecated-declarations>$<$<CXX_COMPILER_ID:MSVC>:/wd4996>)
  

#------------------
# Linker Settings
#------------------
target_link_libraries(
  ifm3dpy
  PRIVATE 
    ifm3d_common
    ifm3d_framegrabber
    ifm3d_device
    pybind11::module
    ifm3d_tools_shared
    ifm3d_swupdater
    $<BUILD_INTERFACE:fmt::fmt-header-only>
    ifm3d_deserialize
  )

#------------------
# Properties
#------------------
set_target_properties(
  ifm3dpy PROPERTIES
  EXPORT_NAME ${_component_name}
  PREFIX "${PYTHON_MODULE_PREFIX}"
  SUFFIX "${PYTHON_MODULE_EXTENSION}"
  DEBUG_POSTFIX ""
  CXX_VISIBILITY_PRESET "hidden"
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

#------------------
# Stubs creation
#------------------

if(CREATE_PYTHON_STUBS)

  if(BUILD_SHARED_LIBS)
    message (
      FATAL_ERROR
        "Error while creating stubs as BUILD_SHARED_LIBS is ON")
  endif()

  # Create a venv
  set(venv ${CMAKE_CURRENT_BINARY_DIR}/stubgen-venv)
  if(NOT IS_DIRECTORY ${venv})
    execute_process(
      COMMAND ${PYTHON_EXECUTABLE} -m venv ${venv}
      RESULT_VARIABLE status
      OUTPUT_VARIABLE output
      ERROR_VARIABLE output)
    if(status AND NOT status EQUAL 0)
      message(
        FATAL_ERROR
          "Error while creating venv in ${venv}:\n${output}")
    endif()
  endif()

  find_program(venv_python python PATHS ${venv}/bin/ ${venv}/Scripts REQUIRED NO_DEFAULT_PATH)

  # Install pybind11-stubgen in the venv
  execute_process(
    COMMAND ${venv_python} -m pip install pybind11-stubgen==0.16.2
    RESULT_VARIABLE status
    OUTPUT_VARIABLE output
    ERROR_VARIABLE output)
  if(status AND NOT status EQUAL 0)
    message(
      FATAL_ERROR
        "Error while installing pybind11-stubgen in ${venv}:\n${output}"
    )
  endif()

  # Generate the stubs
  add_custom_command(
    TARGET ifm3dpy
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E env PYTHONPATH="$<TARGET_FILE_DIR:ifm3dpy>"
      ${CMAKE_COMMAND} -E env PYTHONHOME=""
      ${CMAKE_COMMAND} -E env VIRTUAL_ENV="${venv}"
        ${venv_python} -m pybind11_stubgen --ignore-invalid=all --skip-signature-downgrade --no-setup-py --output-dir=$<TARGET_FILE_DIR:ifm3dpy> --root-module-suffix="" ifm3dpy
  )
endif()
