if(NOT CMAKE_CROSSCOMPILING)
  file(COPY ${IFM3D_PYBIND11_SOURCE_DIR}/test/${TEST_WRAPPER_IN}
    DESTINATION ${IFM3D_PYBIND11_BINARY_DIR}/test
    FILE_PERMISSIONS OWNER_READ
                     OWNER_WRITE
                     OWNER_EXECUTE
                     GROUP_READ
                     GROUP_WRITE
                     GROUP_EXECUTE
                     WORLD_READ
                     WORLD_EXECUTE
                     )

  configure_file(
    ${IFM3D_PYBIND11_BINARY_DIR}/test/${TEST_WRAPPER_IN}
    ${IFM3D_PYBIND11_BINARY_DIR}/test/${TEST_WRAPPER}
    @ONLY
    )

  file(GLOB IFM3D_PYBIND11_TEST_FILES
    "${IFM3D_PYBIND11_SOURCE_DIR}/test/*.py"
    )

  file(COPY ${IFM3D_PYBIND11_TEST_FILES}
    DESTINATION ${IFM3D_PYBIND11_BINARY_DIR}/test
    )

  add_custom_target(check_pybind11
    COMMAND ${IFM3D_PYBIND11_BINARY_DIR}/test/${TEST_WRAPPER}
    )
  add_dependencies(check check_pybind11)
endif()
