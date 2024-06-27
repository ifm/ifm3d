################################################
## Setup
################################################

# Query the selected interpreter for the installation path, or use the override
if(DEFINED PYTHON_PACKAGES_PATH)
  set(_python_install_path ${PYTHON_PACKAGES_PATH})
else()
  execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c "from sysconfig import get_paths; print(get_paths()['stdlib'].removeprefix('/usr'))"
    OUTPUT_VARIABLE _site_packages
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  set(_python_install_path "${CMAKE_INSTALL_PREFIX}/${_site_packages}")
endif()
set(_python_metadata_path "${_python_install_path}/ifm3dpy-${ROOT_PROJECT_VERSION}.dist-info")

configure_file(METADATA.in ${CMAKE_CURRENT_BINARY_DIR}/METADATA @ONLY)

if(NOT PYTHON_VERSION_MAJOR EQUAL 3)
  message(FATAL_ERROR "Unsupported PYTHON_VERSION_MAJOR: ${PYTHON_VERSION_MAJOR}")
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
  $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-Wno-deprecated-declarations>$<$<CXX_COMPILER_ID:MSVC>:/wd4996>
)  

target_compile_definitions(ifm3dpy PRIVATE IFM3D_STATIC_LIB=1)

#------------------
# Linker Settings
#------------------
target_link_libraries(
  ifm3dpy
  PRIVATE 
    ifm3d_common
    ifm3d_framegrabber
    ifm3d_device
    ifm3d_tools
    ifm3d_swupdater
    ifm3d_deserialize
    pybind11::module
    $<BUILD_INTERFACE:fmt::fmt-header-only>
)

#------------------
# Properties
#------------------
set_target_properties(
  ifm3dpy PROPERTIES
  EXPORT_NAME ifm3dpy
  PREFIX "${PYTHON_MODULE_PREFIX}"
  SUFFIX "${PYTHON_MODULE_EXTENSION}"
  DEBUG_POSTFIX ""
  CXX_VISIBILITY_PRESET "hidden"
)

#------------------
# Installation
#------------------
install(TARGETS ifm3dpy
  RUNTIME DESTINATION ${_python_install_path} COMPONENT ifm3dpy
  LIBRARY DESTINATION ${_python_install_path} COMPONENT ifm3dpy
  ARCHIVE DESTINATION ${_python_install_path} COMPONENT ifm3dpy
  PUBLIC_HEADER DESTINATION ${_include} 
  COMPONENT ifm3dpy
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/METADATA 
  DESTINATION ${_python_metadata_path}
  COMPONENT ifm3dpy
)

set(CPACK_DEBIAN_IFM3DPY_PACKAGE_SECTION python CACHE INTERNAL "")
set(CPACK_DEBIAN_IFM3DPY_PACKAGE_DEPENDS "python3-numpy"  CACHE INTERNAL "")
set(CPACK_DEBIAN_IFM3DPY_PACKAGE_NAME ifm3dpy CACHE INTERNAL "")
set(CPACK_DEBIAN_IFM3DPY_DESCRIPTION "Python sensor interface to ifm 3D cameras" CACHE INTERNAL "")

#------------------
# Stubs creation
#------------------

if(CREATE_PYTHON_STUBS)
  if(PYTHON_VERSION_MINOR LESS 7)
    message ( FATAL_ERROR "CREATE_PYTHON_STUBS requires Python 3.7 or later")
  endif()

  if(BUILD_SHARED_LIBS AND WIN32)
    message (FATAL_ERROR "CREATE_PYTHON_STUBS on WIN32 currently is only supported with BUILD_SHARED_LIBS=OFF")
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

  # Install requirements in the venv
  execute_process(
    COMMAND ${venv_python} -m pip install -r ${ROOT_PROJECT_DIR}/requirements.txt
    RESULT_VARIABLE status
    OUTPUT_VARIABLE output
    ERROR_VARIABLE output)
  if(status AND NOT status EQUAL 0)
    message(
      FATAL_ERROR
        "Error while installing requirements in ${venv}:\n${output}"
    )
  endif()

  # Generate the stubs
  add_custom_command(
    TARGET ifm3dpy
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E env --unset=PYTHONHOME PYTHONPATH="$<TARGET_FILE_DIR:ifm3dpy>" VIRTUAL_ENV="${venv}" 
        ${venv_python} -m pybind11_stubgen --ignore-invalid-expressions='.*' -o=$<TARGET_FILE_DIR:ifm3dpy> --root-suffix="" ifm3dpy
  )

  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ifm3dpy 
    DESTINATION ${_python_install_path}
    COMPONENT ifm3dpy
  )
endif()
