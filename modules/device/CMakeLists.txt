project(IFM3D_DEVICE CXX)
set(IFM3D_DEVICE_MODULE_NAME "libifm3d-device")

configure_file(
  ${IFM3D_DEVICE_SOURCE_DIR}/include/ifm3d/device/version.h.in
  ${IFM3D_DEVICE_BINARY_DIR}/include/ifm3d/device/version.h
  @ONLY
  )

file(READ ${IFM3D_DEVICE_SOURCE_DIR}/include/ifm3d/device/json_impl.hpp.in IFM_JSON_HPP)
string(REPLACE INCLUDE_NLOHMANN_JSON_HPP_ IFM_JSON_IMPL_HPP IFM_JSON_HPP "${IFM_JSON_HPP}") 
string(REPLACE NLOHMANN_JSON_ IFM3D_JSON_ IFM_JSON_HPP "${IFM_JSON_HPP}")
string(REPLACE NLOHMANN_ IFM3D_JSON_ IFM_JSON_HPP "${IFM_JSON_HPP}")
string(REPLACE JSON_ IFM3D_JSON_ IFM_JSON_HPP "${IFM_JSON_HPP}")
string(REPLACE nlohmann:: ifm3d:: IFM_JSON_HPP "${IFM_JSON_HPP}")
string(REPLACE "namespace nlohmann" "namespace ifm3d" IFM_JSON_HPP "${IFM_JSON_HPP}")
file(GENERATE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/include/ifm3d/device/json_impl.hpp CONTENT "${IFM_JSON_HPP}")


include_directories(${IFM3D_DEVICE_SOURCE_DIR}/include ${IFM3D_DEVICE_BINARY_DIR}/include)


################################################
## Process child CMakeLists.txt files
################################################
add_subdirectory(src)
if(BUILD_TESTS)
  add_subdirectory(test)
else()
  message(WARNING "Skipping ${IFM3D_DEVICE_MODULE_NAME} unit tests!")
endif()

################################################
## Manage installation process
################################################
if(BUILD_SDK_PKG)
  install(DIRECTORY ${IFM3D_DEVICE_SOURCE_DIR}/include/ ${IFM3D_DEVICE_BINARY_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT device
    PATTERN "*.in" EXCLUDE
    PATTERN ".git" EXCLUDE
    PATTERN "*~" EXCLUDE
    )
endif()
