project(IFM3D_DESERIALIZE CXX)
set(IFM3D_DESERIALIZE_MODULE_NAME "deserialize")

################################################
## Process child CMakeLists.txt files
################################################
add_library(ifm3d_deserialize INTERFACE)
target_include_directories(ifm3d_deserialize INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_FG_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(ifm3d_deserialize INTERFACE ifm3d_framegrabber)

set_target_properties(
  ifm3d_deserialize PROPERTIES
  EXPORT_NAME deserialize
)

if(BUILD_TESTS)
  add_subdirectory(test)
else()
  message(WARNING "Skipping ${IFM3D_DESERIALIZE_MODULE_NAME} unit tests!")
endif()

################################################
## Manage installation process
################################################
install(TARGETS ifm3d_deserialize
  EXPORT ifm3d-deserialize-targets
  RUNTIME DESTINATION ${_bin} COMPONENT deserialize
  LIBRARY DESTINATION ${_lib} COMPONENT deserialize
  ARCHIVE DESTINATION ${_lib} COMPONENT deserialize
  PUBLIC_HEADER DESTINATION ${_include} COMPONENT deserialize
)

install(DIRECTORY ${IFM3D_DESERIALIZE_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  COMPONENT deserialize
  PATTERN "*.in" EXCLUDE
  PATTERN ".git" EXCLUDE
  PATTERN "*~" EXCLUDE
)

if(BUILD_SDK_PKG)
  install(DIRECTORY ${IFM3D_DESERIALIZE_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT deserialize
    PATTERN "*.in" EXCLUDE
    PATTERN ".git" EXCLUDE
    PATTERN "*~" EXCLUDE
  )

  install(EXPORT ifm3d-deserialize-targets
    NAMESPACE ifm3d::
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
    COMPONENT deserialize
  )
endif()

