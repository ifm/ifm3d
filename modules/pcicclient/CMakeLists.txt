project(IFM3D_PCICCLIENT CXX)
set(IFM3D_PCICCLIENT_MODULE_NAME "libifm3d-pcicclient")


include_directories(${IFM3D_PCICCLIENT_SOURCE_DIR}/include)


################################################
## Process child CMakeLists.txt files
################################################
add_subdirectory(src)
if(BUILD_TESTS)
  add_subdirectory(test)
else()
  message(WARNING "Skipping ${IFM3D_PCICCLIENT_MODULE_NAME} unit tests!")
endif()

################################################
## Manage installation process
################################################
if(BUILD_SDK_PKG)
  install(DIRECTORY ${IFM3D_PCICCLIENT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT ifm3d
    PATTERN "*.in" EXCLUDE
    PATTERN ".git" EXCLUDE
    PATTERN "*~" EXCLUDE
    )
endif()
