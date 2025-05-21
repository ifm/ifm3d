configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/ifm3dpy_version.py.in
  ${CMAKE_CURRENT_BINARY_DIR}/ifm3dpy_version.py
  @ONLY
  )

get_target_property(DOXYGEN_DIR ifm3d_doxygen BINARY_DIR)
get_target_property(IFM3DPY_DIR ifm3dpy BINARY_DIR)
if(NOT EXISTS ${PROJECT_SOURCE_DIR}/doc/sphinx/content/ifm3d-examples)
    execute_process(
        COMMAND git clone https://github.com/ifm/ifm3d-examples ${PROJECT_SOURCE_DIR}/doc/sphinx/content/ifm3d-examples
    )
else()
    execute_process(COMMAND git -C ${PROJECT_SOURCE_DIR}/doc/sphinx/content/ifm3d-examples checkout -q .)
    execute_process(COMMAND git -C ${PROJECT_SOURCE_DIR}/doc/sphinx/content/ifm3d-examples pull)
endif()


# Create a venv
set(venv ${CMAKE_CURRENT_BINARY_DIR}/docs-venv)
if(NOT IS_DIRECTORY ${venv})
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -m venv ${venv}
        RESULT_VARIABLE status
        OUTPUT_VARIABLE output
        ERROR_VARIABLE output
    )
    if(status AND NOT status EQUAL 0)
        message(
        FATAL_ERROR
            "Error while creating venv in ${venv}:\n${output}"
    )
    endif()
endif()

find_program(venv_python python PATHS ${venv}/bin/ ${venv}/Scripts REQUIRED NO_DEFAULT_PATH)

# Install requirements in the venv
execute_process(
    COMMAND ${venv_python} -m pip install -r ${ROOT_PROJECT_DIR}/requirements.txt -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt
    RESULT_VARIABLE status
    OUTPUT_VARIABLE output
    ERROR_VARIABLE output
)
if(status AND NOT status EQUAL 0)
    message(FATAL_ERROR "Error while installing requirements in ${venv}:\n${output}")
endif()

add_custom_target(ifm3d_sphinx
    DEPENDS ifm3d_doxygen
    DEPENDS ifm3dpy
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/docker ${CMAKE_CURRENT_BINARY_DIR}/docker   
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/README.md ${CMAKE_CURRENT_BINARY_DIR}/content/README.md       
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/xyz.png ${CMAKE_CURRENT_BINARY_DIR}/content/xyz.png    
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/distance.png ${CMAKE_CURRENT_BINARY_DIR}/content/distance.png
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/jpeg.png ${CMAKE_CURRENT_BINARY_DIR}/content/jpeg.png
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/ChangeLog.md ${CMAKE_CURRENT_BINARY_DIR}/content/ChangeLog.md       
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${IFM3DPY_DIR}"
        ${venv_python} -m sphinx.cmd.build -b html -D html_extra_path=${DOXYGEN_DIR}/html "${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/html"
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating python documentation with Sphinx"
    VERBATIM)