file(GLOB IFM3D_CRYPTO_SOURCES CONFIGURE_DEPENDS *.cpp)
add_library(ifm3d_crypto INTERFACE)
target_sources(ifm3d_crypto INTERFACE ${IFM3D_CRYPTO_SOURCES})

target_include_directories(ifm3d_crypto
  INTERFACE
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${IFM3D_CRYPTO_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_CRYPTO_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  )
  
target_link_libraries(
  ifm3d_crypto
  INTERFACE
    ifm3d_common
    sodium
)