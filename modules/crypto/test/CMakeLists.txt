if(NOT CMAKE_CROSSCOMPILING)
  file(COPY ${IFM3D_CRYPTO_SOURCE_DIR}/test/${TEST_WRAPPER_IN}
    DESTINATION ${IFM3D_CRYPTO_BINARY_DIR}/test
    FILE_PERMISSIONS 
      OWNER_READ
      OWNER_WRITE
      OWNER_EXECUTE
      GROUP_READ
      GROUP_WRITE
      GROUP_EXECUTE
      WORLD_READ
      WORLD_EXECUTE
)

  configure_file(
    ${IFM3D_CRYPTO_BINARY_DIR}/test/${TEST_WRAPPER_IN}
    ${IFM3D_CRYPTO_BINARY_DIR}/test/${TEST_WRAPPER}
    @ONLY
  )
endif()

file(GLOB TEST_SOURCES *.cpp)
add_executable(ifm3d-crypto-tests ${TEST_SOURCES})
target_compile_definitions(ifm3d-crypto-tests PRIVATE IFM3D_STATIC_LIB)
target_link_libraries(ifm3d-crypto-tests
  gtest_main
  ifm3d_crypto
  sodium
)

if(NOT CMAKE_CROSSCOMPILING)
  add_custom_target(check_crypto
    COMMAND ${IFM3D_CRYPTO_BINARY_DIR}/test/${TEST_WRAPPER} DEPENDS ifm3d-crypto-tests
  )
  add_dependencies(check check_crypto)
endif()

