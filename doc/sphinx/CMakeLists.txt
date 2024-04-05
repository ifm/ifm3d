configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/ifm3dpy_version.py.in
  ${CMAKE_CURRENT_BINARY_DIR}/ifm3dpy_version.py
  @ONLY
  )

get_target_property(DOXYGEN_DIR ifm3d_doxygen BINARY_DIR)
get_target_property(IFM3DPY_DIR ifm3dpy BINARY_DIR)
execute_process(
    COMMAND git clone https://github.com/ifm/ifm3d-examples ${PROJECT_SOURCE_DIR}/doc/sphinx/content/ifm3d-examples
)
add_custom_target(ifm3d_sphinx
    DEPENDS ifm3d_doxygen
    DEPENDS ifm3dpy
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/docker ${CMAKE_CURRENT_BINARY_DIR}/docker   
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/README.md ${CMAKE_CURRENT_BINARY_DIR}/content/README.md       
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/ChangeLog.md ${CMAKE_CURRENT_BINARY_DIR}/content/ChangeLog.md       
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${IFM3DPY_DIR}"
        ${PYTHON_EXECUTABLE} -m sphinx.cmd.build -b html -D html_extra_path=${DOXYGEN_DIR}/html "${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/html"
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating python documentation with Sphinx"
    VERBATIM)