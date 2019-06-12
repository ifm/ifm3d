if(NOT CMAKE_CROSSCOMPILING)
  file(COPY ${IFM3D_PCICCLIENT_SOURCE_DIR}/test/${TEST_WRAPPER_IN}
    DESTINATION ${IFM3D_PCICCLIENT_BINARY_DIR}/test
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
    ${IFM3D_PCICCLIENT_BINARY_DIR}/test/${TEST_WRAPPER_IN}
    ${IFM3D_PCICCLIENT_BINARY_DIR}/test/${TEST_WRAPPER}
    @ONLY
    )
endif()

file(GLOB TEST_SOURCES *.cpp)
add_executable(ifm3d-pcicclient-tests ${TEST_SOURCES})
target_link_libraries(ifm3d-pcicclient-tests
                      gtest_main
                      ifm3d_pcicclient
                      ifm3d_camera
                      ${LIB_glog}
                      )

if(NOT CMAKE_CROSSCOMPILING)
  add_custom_target(check_pcicclient
    COMMAND ${IFM3D_PCICCLIENT_BINARY_DIR}/test/${TEST_WRAPPER} DEPENDS ifm3d-pcicclient-tests
    )
  add_dependencies(check check_pcicclient)
endif()
