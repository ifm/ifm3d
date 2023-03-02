if(NOT CMAKE_CROSSCOMPILING)
  file(COPY ${IFM3D_DESERIALIZE_SOURCE_DIR}/test/${TEST_WRAPPER_IN}
    DESTINATION ${IFM3D_DESERIALIZE_BINARY_DIR}/test
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
    ${IFM3D_DESERIALIZE_BINARY_DIR}/test/${TEST_WRAPPER_IN}
    ${IFM3D_DESERIALIZE_BINARY_DIR}/test/${TEST_WRAPPER}
    @ONLY
    )
endif()

file(GLOB TEST_DATA_FILES data/*.data)
file(COPY ${TEST_DATA_FILES}
    DESTINATION ${IFM3D_DESERIALIZE_BINARY_DIR}/test)

file(GLOB TEST_SOURCES *.cpp)
add_executable(ifm3d-deserialize-tests ${TEST_SOURCES})

target_include_directories(ifm3d-deserialize-tests PRIVATE
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/data>
 )
target_link_libraries(ifm3d-deserialize-tests 
                      gtest_main
                      ifm3d_common
                      ifm3d_deserialize
                      ifm3d_framegrabber
                      ifm3d_device
                      $<BUILD_INTERFACE:fmt::fmt-header-only>
                      )

if(NOT CMAKE_CROSSCOMPILING)
  add_custom_target(check_deserialize
    COMMAND ${IFM3D_DESERIALIZE_BINARY_DIR}/test/${TEST_WRAPPER} DEPENDS ifm3d-deserialize-tests
    )
  add_dependencies(check check_deserialize)
endif()
