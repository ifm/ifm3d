project(IFM3D_DESERIALIZE CXX)
set(IFM3D_DESERIALIZE_MODULE_NAME "deserialize")
add_library(ifm3d_deserialize INTERFACE)

target_include_directories(ifm3d_deserialize 
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${IFM3D_FG_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(ifm3d_deserialize INTERFACE ifm3d_framegrabber)

if(BUILD_TESTS)
  add_subdirectory(test)
else()
  message(WARNING "Skipping ${IFM3D_DESERIALIZE_MODULE_NAME} unit tests!")
endif()

################################################
## Manage installation process
################################################
if(BUILD_SDK_PKG)
  install(DIRECTORY ${IFM3D_DESERIALIZE_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT ifm3d
    PATTERN "*.in" EXCLUDE
    PATTERN ".git" EXCLUDE
    PATTERN "*~" EXCLUDE
  )
endif()
